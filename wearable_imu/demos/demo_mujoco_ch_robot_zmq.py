"""Run ch_robot MuJoCo viewer from mock-live human joints over ZeroMQ.

Pair this with ``demo_zmq_human_joint_publisher.py``:

Terminal 1:
  python demos/demo_zmq_human_joint_publisher.py ../data/human_joint_clip_20260601_231345.npz --fps 50

Terminal 2:
  python demos/demo_mujoco_ch_robot_zmq.py
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil
import sys
import time

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

PROJECT_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PROJECT_ROOT.parent
for path in (PROJECT_ROOT, REPO_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from demos.demo_mujoco_ch_robot_replay import (  # noqa: E402
    DEFAULT_MODEL_CACHE,
    ensure_ch_robot_model,
)
from ik.ch_robot_retarget import (  # noqa: E402
    CH_ROBOT_JOINT_NAMES,
    QPOS_WIDTH,
    QvelFiniteDifferencer,
    base_position_from_joint_points,
    joint_positions_to_qpos,
)
from ik.zmq_human_joint_stream import (  # noqa: E402
    DEFAULT_HUMAN_JOINT_ENDPOINT,
    HUMAN_JOINT_TOPIC,
    decode_human_joint_frame,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", default=DEFAULT_HUMAN_JOINT_ENDPOINT)
    parser.add_argument("--model-dir", type=Path, default=DEFAULT_MODEL_CACHE)
    parser.add_argument("--refresh-model", action="store_true")
    parser.add_argument("--yaw-mode", choices=("keep", "strip"), default="keep")
    parser.add_argument(
        "--base-motion",
        choices=("root_xy_forward", "root_xy", "fixed", "root_xyz"),
        default="root_xy_forward",
        help="How human root translation drives the MuJoCo freejoint base.",
    )
    parser.add_argument("--base-height", type=float, default=0.765)
    parser.add_argument("--poll-timeout-ms", type=int, default=100)
    parser.add_argument("--status-every", type=int, default=50)
    parser.add_argument("--no-show", action="store_true", help="Subscribe and convert frames without opening viewer.")
    parser.add_argument("--max-frames", type=int, default=0, help="Stop after N received frames; useful with --no-show.")
    return parser.parse_args()


def relaunch_with_mjpython_if_needed(args: argparse.Namespace) -> None:
    if args.no_show or sys.platform != "darwin":
        return
    if Path(sys.executable).name == "mjpython":
        return
    if os.environ.get("CH_ROBOT_ZMQ_MJPYTHON") == "1":
        return
    mjpython = shutil.which("mjpython")
    if mjpython is None:
        raise RuntimeError(
            "MuJoCo viewer on macOS requires mjpython. Try: "
            "mjpython demos/demo_mujoco_ch_robot_zmq.py"
        )
    env = os.environ.copy()
    env["CH_ROBOT_ZMQ_MJPYTHON"] = "1"
    print(f"macOS MuJoCo viewer requires mjpython; relaunching with {mjpython}")
    os.execvpe(mjpython, [mjpython, *sys.argv], env)


def _rotation_from_wxyz(quat_wxyz: list[float] | tuple[float, ...] | np.ndarray) -> Rotation:
    qw, qx, qy, qz = np.asarray(quat_wxyz, dtype=np.float64)
    return Rotation.from_quat([qx, qy, qz, qw])


def _recv_latest(socket, flags: int = 0):
    import zmq

    parts = socket.recv_multipart(flags=flags)
    while True:
        try:
            parts = socket.recv_multipart(flags=zmq.NOBLOCK)
        except zmq.Again:
            break
    return decode_human_joint_frame(parts)


def _frame_to_qpos(
    header: dict,
    points: np.ndarray,
    *,
    base_height: float,
    base_motion: str,
    root_origin: np.ndarray,
    yaw_mode: str,
) -> np.ndarray:
    pelvis_orientation = None
    if "root_quat_wxyz" in header:
        pelvis_orientation = _rotation_from_wxyz(header["root_quat_wxyz"])
    base_position = base_position_from_joint_points(
        points,
        root_origin=root_origin,
        base_height=base_height,
        base_motion=base_motion,
    )
    return joint_positions_to_qpos(
        points,
        pelvis_orientation,
        base_height=base_height,
        base_position=base_position,
        yaw_mode=yaw_mode,
    )


def main() -> None:
    try:
        import zmq
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError("pyzmq is required. Install with: python -m pip install pyzmq") from exc

    args = parse_args()
    relaunch_with_mjpython_if_needed(args)

    xml_path = ensure_ch_robot_model(args.model_dir, refresh=args.refresh_model)
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    if model.nq != QPOS_WIDTH:
        raise RuntimeError(f"expected ch_robot nq={QPOS_WIDTH}, got {model.nq}")
    if model.nu != len(CH_ROBOT_JOINT_NAMES):
        raise RuntimeError(f"expected ch_robot nu={len(CH_ROBOT_JOINT_NAMES)}, got {model.nu}")

    context = zmq.Context.instance()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.SUBSCRIBE, HUMAN_JOINT_TOPIC)
    socket.connect(args.endpoint)
    poller = zmq.Poller()
    poller.register(socket, zmq.POLLIN)

    print("MuJoCo ch_robot ZMQ subscriber")
    print(f"  endpoint : {args.endpoint}")
    print(f"  topic    : {HUMAN_JOINT_TOPIC.decode()}")
    print(f"  model    : {xml_path}")
    print(f"  nq/nu    : {model.nq}/{model.nu}")
    print(f"  base     : {args.base_motion}")
    print("Waiting for frames...")

    qvel_diff = QvelFiniteDifferencer()
    received = 0
    root_origin: np.ndarray | None = None
    last_qpos = np.zeros(QPOS_WIDTH, dtype=np.float64)
    last_qpos[:7] = [0.0, 0.0, args.base_height, 1.0, 0.0, 0.0, 0.0]

    def handle_available_frame() -> tuple[np.ndarray, np.ndarray] | None:
        nonlocal received, root_origin, last_qpos
        events = dict(poller.poll(args.poll_timeout_ms))
        if socket not in events:
            return None
        header, points = _recv_latest(socket)
        if root_origin is None:
            root_origin = points[0].copy()
        qpos = _frame_to_qpos(
            header,
            points,
            base_height=args.base_height,
            base_motion=args.base_motion,
            root_origin=root_origin,
            yaw_mode=args.yaw_mode,
        )
        timestamp_s = float(header.get("timestamp_s", time.time()))
        qvel = qvel_diff.update(qpos, timestamp_s)
        received += 1
        last_qpos = qpos
        if args.status_every > 0 and received % args.status_every == 0:
            print(
                f"  received={received} frame={header.get('frame_index')} "
                f"clip_t={float(header.get('clip_time_s', 0.0)):.3f}"
            )
        return qpos, qvel

    try:
        if args.no_show:
            while args.max_frames <= 0 or received < args.max_frames:
                frame = handle_available_frame()
                if frame is None:
                    continue
            print(f"Converted {received} ZMQ frames.")
            return

        from mujoco import viewer as mujoco_viewer

        data.qpos[:] = last_qpos
        data.ctrl[:] = last_qpos[7:]
        mujoco.mj_forward(model, data)
        print("\nMuJoCo viewer running from ZMQ. Close the viewer to exit.\n")
        with mujoco_viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                frame = handle_available_frame()
                if frame is not None:
                    qpos, qvel = frame
                    data.qpos[:] = qpos
                    data.qvel[:] = qvel
                    data.ctrl[:] = qpos[7:]
                    mujoco.mj_forward(model, data)
                viewer.sync()
                time.sleep(0.002)
    except KeyboardInterrupt:
        print("Stopping ZMQ subscriber.")
    finally:
        socket.close(linger=0)


if __name__ == "__main__":
    main()
