"""Drive the ch_robot MuJoCo model from live ESP32/BNO085 IMU packets.

This is the direct, single-process visualization path:

    ESP32/BNO085 UDP -> calibration -> lower-body pose -> ch_robot qpos -> MuJoCo

Usage (from wearable_imu/):
  python demos/demo_mujoco_ch_robot_live.py --config full
  python demos/demo_mujoco_ch_robot_live.py --config shanks --yaw-mode strip
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil
import sys
import threading
import time

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

PROJECT_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PROJECT_ROOT.parent
for path in (PROJECT_ROOT, REPO_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from demos.demo_live_retarget import (  # noqa: E402
    PARTIAL_CONFIGS,
    calibrate,
    segment_orientations,
    wait_for_segments,
)
from demos.demo_mujoco_ch_robot_replay import (  # noqa: E402
    DEFAULT_MODEL_CACHE,
    ensure_ch_robot_model,
)
from ik.ch_robot_retarget import (  # noqa: E402
    CH_ROBOT_JOINT_NAMES,
    QPOS_WIDTH,
    QVEL_WIDTH,
    QvelFiniteDifferencer,
    legposes_to_qpos,
)
from ik.lower_body_aggregation import aggregate_lower_body_skeleton  # noqa: E402
from sensor.filtering import QuaternionPacketFilter  # noqa: E402
from sensor.packet import SegmentId  # noqa: E402
from sensor.udp_receiver import LatestPacketBuffer, receive_quaternion_packets  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0", help="Incoming IMU UDP bind host.")
    parser.add_argument("--port", type=int, default=5005, help="Incoming IMU UDP bind port.")
    parser.add_argument("--config", choices=tuple(PARTIAL_CONFIGS), default="full")
    parser.add_argument("--fps", type=float, default=100.0, help="Retarget and viewer update rate.")
    parser.add_argument("--max-age-ms", type=float, default=150.0)
    parser.add_argument("--min-samples", type=int, default=20)
    parser.add_argument("--base-height", type=float, default=0.765)
    parser.add_argument("--yaw-mode", choices=("keep", "strip"), default="keep")
    parser.add_argument("--no-calibration", action="store_true", help="Use raw mounted segment orientations.")
    parser.add_argument("--model-dir", type=Path, default=DEFAULT_MODEL_CACHE)
    parser.add_argument("--refresh-model", action="store_true")
    parser.add_argument("--status-every", type=int, default=100)
    parser.add_argument("--no-show", action="store_true", help="Retarget live frames without opening MuJoCo.")
    parser.add_argument("--max-frames", type=int, default=0, help="Stop after N valid frames; primarily for testing.")
    return parser.parse_args()


def relaunch_with_mjpython_if_needed(args: argparse.Namespace) -> None:
    if args.no_show or sys.platform != "darwin":
        return
    if Path(sys.executable).name == "mjpython" or os.environ.get("MJPYTHON_BIN"):
        return
    if os.environ.get("CH_ROBOT_LIVE_MJPYTHON") == "1":
        return
    sibling_mjpython = Path(sys.executable).with_name("mjpython")
    mjpython = str(sibling_mjpython) if sibling_mjpython.exists() else shutil.which("mjpython")
    if mjpython is None:
        raise RuntimeError(
            "MuJoCo viewer on macOS requires mjpython. Try: "
            "mjpython demos/demo_mujoco_ch_robot_live.py --config full"
        )
    env = os.environ.copy()
    env["CH_ROBOT_LIVE_MJPYTHON"] = "1"
    print(f"macOS MuJoCo viewer requires mjpython; relaunching with {mjpython}")
    os.execvpe(mjpython, [mjpython, *sys.argv], env)


def apply_robot_state(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    qpos: np.ndarray,
    qvel: np.ndarray,
) -> None:
    if qpos.shape != (QPOS_WIDTH,):
        raise ValueError(f"expected qpos shape {(QPOS_WIDTH,)}, got {qpos.shape}")
    if qvel.shape != (QVEL_WIDTH,):
        raise ValueError(f"expected qvel shape {(QVEL_WIDTH,)}, got {qvel.shape}")
    data.qpos[:] = qpos
    data.qvel[:] = qvel
    data.ctrl[:] = qpos[7:]
    mujoco.mj_forward(model, data)


def main() -> None:
    args = parse_args()
    if args.fps <= 0.0:
        raise ValueError("--fps must be greater than zero")

    relaunch_with_mjpython_if_needed(args)
    xml_path = ensure_ch_robot_model(args.model_dir, refresh=args.refresh_model)
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    if model.nq != QPOS_WIDTH:
        raise RuntimeError(f"expected ch_robot nq={QPOS_WIDTH}, got {model.nq}")
    if model.nv != QVEL_WIDTH:
        raise RuntimeError(f"expected ch_robot nv={QVEL_WIDTH}, got {model.nv}")
    if model.nu != len(CH_ROBOT_JOINT_NAMES):
        raise RuntimeError(f"expected ch_robot nu={len(CH_ROBOT_JOINT_NAMES)}, got {model.nu}")

    required_segments = PARTIAL_CONFIGS[args.config]
    max_age_s = args.max_age_ms / 1000.0
    frame_period_s = 1.0 / args.fps
    buffer = LatestPacketBuffer()
    packet_filter = QuaternionPacketFilter()
    filtered: dict[SegmentId, Rotation] = {}
    filter_lock = threading.Lock()
    stop_event = threading.Event()
    setup_error: list[BaseException] = []
    recv_thread: threading.Thread | None = None

    def receive_loop() -> None:
        try:
            for packet in receive_quaternion_packets(host=args.host, port=args.port, timeout_s=0.25):
                if stop_event.is_set():
                    break
                buffer.update(packet)
                result = packet_filter.update(packet)
                if result.rotation is not None:
                    with filter_lock:
                        filtered[packet.segment_id] = result.rotation
        except BaseException as exc:
            setup_error.append(exc)
            stop_event.set()

    qvel_diff = QvelFiniteDifferencer()
    valid_frames = 0
    stale_frames = 0
    profile = None
    stream_ready = threading.Event()

    def prepare_stream() -> None:
        nonlocal profile
        try:
            wait_for_segments(buffer, required_segments)
            if stop_event.is_set():
                return
            profile = None if args.no_calibration else calibrate(buffer, required_segments, args.min_samples)
            stream_ready.set()
            print("Live retargeting active. Move to drive ch_robot.", flush=True)
        except BaseException as exc:
            setup_error.append(exc)
            stop_event.set()

    def start_stream_workers() -> None:
        nonlocal recv_thread
        recv_thread = threading.Thread(target=receive_loop, daemon=True)
        recv_thread.start()
        threading.Thread(target=prepare_stream, daemon=True).start()

    def next_frame() -> tuple[np.ndarray, np.ndarray] | None:
        nonlocal valid_frames, stale_frames
        if not stream_ready.is_set():
            return None
        with filter_lock:
            filtered_snapshot = dict(filtered)
        orientations = segment_orientations(
            filtered_snapshot,
            buffer,
            profile,
            required_segments,
            max_age_s,
        )
        if orientations is None:
            stale_frames += 1
            return None
        skeleton = aggregate_lower_body_skeleton(orientations)
        qpos = legposes_to_qpos(
            skeleton.joint_rotations,
            skeleton.segment_orientations[SegmentId.PELVIS],
            base_height=args.base_height,
            yaw_mode=args.yaw_mode,
        )
        qvel = qvel_diff.update(qpos, time.monotonic())
        valid_frames += 1
        if args.status_every > 0 and valid_frames % args.status_every == 0:
            print(f"frames={valid_frames} stale={stale_frames}")
        return qpos, qvel

    try:
        print(f"Live ch_robot MuJoCo: {args.config} IMUs on {args.host}:{args.port}")
        print(f"Model: {xml_path}")
        print("The robot base is fixed; wearable IMUs provide orientation, not global position.")

        home_qpos = np.zeros(QPOS_WIDTH, dtype=np.float64)
        home_qpos[:7] = [0.0, 0.0, args.base_height, 1.0, 0.0, 0.0, 0.0]
        apply_robot_state(model, data, home_qpos, np.zeros(QVEL_WIDTH, dtype=np.float64))

        if args.no_show:
            start_stream_workers()
            while args.max_frames <= 0 or valid_frames < args.max_frames:
                if setup_error:
                    raise setup_error[0]
                frame = next_frame()
                if frame is not None:
                    apply_robot_state(model, data, *frame)
                time.sleep(frame_period_s)
            return

        from mujoco import viewer as mujoco_viewer

        print("\nOpening MuJoCo in the standing pose. Sensor reception starts after the window opens.\n")
        next_tick_s = time.monotonic()
        with mujoco_viewer.launch_passive(model, data) as viewer:
            viewer.sync()
            start_stream_workers()
            print("Waiting for IMUs. Hold a neutral pose for calibration.", flush=True)
            while viewer.is_running() and (args.max_frames <= 0 or valid_frames < args.max_frames):
                if setup_error:
                    raise setup_error[0]
                now = time.monotonic()
                if now >= next_tick_s:
                    next_tick_s = now + frame_period_s
                    frame = next_frame()
                    if frame is not None:
                        apply_robot_state(model, data, *frame)
                viewer.sync()
                time.sleep(0.002)
    except KeyboardInterrupt:
        print("Stopping live ch_robot viewer.")
    finally:
        stop_event.set()
        if recv_thread is not None:
            recv_thread.join(timeout=1.0)
        print(f"Summary: frames={valid_frames}, stale={stale_frames}")


if __name__ == "__main__":
    main()
