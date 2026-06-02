"""Live IMU-to-ch_robot retargeting bridge.

The bridge receives ESP32/BNO085 quaternion UDP packets, applies the same
sensor-to-segment mount and neutral calibration path as the live skeleton
viewer, converts the resulting ``LegPose`` rotations to ch_robot qpos/qvel, and
emits each frame as JSON over stdout or UDP.

Usage (from wearable_imu/):
  python demos/demo_live_retarget.py --config shanks --output stdout
  python demos/demo_live_retarget.py --config full --output udp --target-host 127.0.0.1 --target-port 6010
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import socket
import sys
import threading
import time
from typing import Literal

import numpy as np
from scipy.spatial.transform import Rotation

PROJECT_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PROJECT_ROOT.parent
for path in (PROJECT_ROOT, REPO_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from calibration.neutral import CalibrationProfile, NeutralCalibrationAccumulator  # noqa: E402
from ik.ch_robot_retarget import QvelFiniteDifferencer, legposes_to_qpos  # noqa: E402
from ik.imu_orientation import default_lower_limb_mounts, front_pelvis_mount  # noqa: E402
from ik.lower_body_aggregation import aggregate_lower_body_skeleton  # noqa: E402
from sensor.filtering import QuaternionPacketFilter  # noqa: E402
from sensor.packet import SegmentId  # noqa: E402
from sensor.udp_receiver import LatestPacketBuffer, receive_quaternion_packets  # noqa: E402


OutputMode = Literal["stdout", "udp", "none"]

PARTIAL_CONFIGS: dict[str, tuple[SegmentId, ...]] = {
    "thighs": (
        SegmentId.PELVIS,
        SegmentId.LEFT_THIGH,
        SegmentId.RIGHT_THIGH,
    ),
    "shanks": (
        SegmentId.PELVIS,
        SegmentId.LEFT_THIGH,
        SegmentId.RIGHT_THIGH,
        SegmentId.LEFT_SHANK,
        SegmentId.RIGHT_SHANK,
    ),
    "full": (
        SegmentId.PELVIS,
        SegmentId.LEFT_THIGH,
        SegmentId.LEFT_SHANK,
        SegmentId.LEFT_FOOT,
        SegmentId.RIGHT_THIGH,
        SegmentId.RIGHT_SHANK,
        SegmentId.RIGHT_FOOT,
    ),
}

_LEFT = default_lower_limb_mounts("left")
_RIGHT = default_lower_limb_mounts("right")
_SEGMENT_MOUNTS: dict[SegmentId, Rotation] = {
    SegmentId.PELVIS: front_pelvis_mount(),
    SegmentId.LEFT_THIGH: _LEFT["thigh"],
    SegmentId.LEFT_SHANK: _LEFT["shank"],
    SegmentId.LEFT_FOOT: _LEFT["foot"],
    SegmentId.RIGHT_THIGH: _RIGHT["thigh"],
    SegmentId.RIGHT_SHANK: _RIGHT["shank"],
    SegmentId.RIGHT_FOOT: _RIGHT["foot"],
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0", help="Incoming IMU UDP bind host.")
    parser.add_argument("--port", type=int, default=5005, help="Incoming IMU UDP bind port.")
    parser.add_argument("--config", choices=tuple(PARTIAL_CONFIGS), default="shanks")
    parser.add_argument("--fps", type=float, default=100.0, help="Retarget output rate.")
    parser.add_argument("--duration-s", type=float, default=0.0, help="Stop after N seconds; 0 runs until Ctrl-C.")
    parser.add_argument("--max-age-ms", type=float, default=150.0)
    parser.add_argument("--min-samples", type=int, default=20)
    parser.add_argument("--base-height", type=float, default=0.765)
    parser.add_argument("--yaw-mode", choices=("keep", "strip"), default="keep")
    parser.add_argument("--no-calibration", action="store_true", help="Use raw mounted segment orientations.")
    parser.add_argument("--output", choices=("stdout", "udp", "none"), default="stdout")
    parser.add_argument("--target-host", default="127.0.0.1", help="UDP target host when --output udp.")
    parser.add_argument("--target-port", type=int, default=6010, help="UDP target port when --output udp.")
    parser.add_argument("--print-every", type=int, default=25, help="Status interval in emitted frames.")
    return parser.parse_args()


def calibrate(
    buffer: LatestPacketBuffer,
    required_segments: tuple[SegmentId, ...],
    min_samples: int,
) -> CalibrationProfile:
    accumulator = NeutralCalibrationAccumulator(
        required_segments=required_segments,
        min_samples_per_segment=min_samples,
    )
    last_sequences: dict[SegmentId, int] = {}
    last_print = 0.0

    print(f"Calibrating neutral pose ({min_samples} samples per segment)", file=sys.stderr)
    while not accumulator.ready():
        for segment in required_segments:
            packet = buffer.packets.get(segment)
            if packet is None or last_sequences.get(segment) == packet.sequence:
                continue
            last_sequences[segment] = packet.sequence
            accumulator.add_packet(packet)

        now = time.monotonic()
        if now - last_print >= 0.25:
            counts = accumulator.sample_counts()
            progress = " | ".join(
                f"{segment.name.lower()}: {counts.get(segment, 0)}/{min_samples}"
                for segment in required_segments
            )
            print(f"  {progress}", end="\r", file=sys.stderr, flush=True)
            last_print = now
        time.sleep(0.005)

    print("\nCalibration complete.", file=sys.stderr)
    return accumulator.build_profile()


def segment_orientations(
    filtered: dict[SegmentId, Rotation],
    buffer: LatestPacketBuffer,
    profile: CalibrationProfile | None,
    required_segments: tuple[SegmentId, ...],
    max_age_s: float,
) -> dict[SegmentId, Rotation] | None:
    now = time.monotonic()
    orientations: dict[SegmentId, Rotation] = {}

    for segment in required_segments:
        packet = buffer.packets.get(segment)
        if packet is None or (now - packet.receive_time_s) > max_age_s:
            return None

        raw = filtered.get(segment)
        if raw is None:
            return None

        mount = _SEGMENT_MOUNTS[segment]
        live_segment = raw * mount.inv()
        if profile is None:
            orientations[segment] = live_segment
            continue

        neutral_segment = profile.neutral_orientations[segment] * mount.inv()
        orientations[segment] = neutral_segment.inv() * live_segment

    return orientations


def wait_for_segments(buffer: LatestPacketBuffer, required_segments: tuple[SegmentId, ...]) -> None:
    print("Waiting for required IMU segments...", file=sys.stderr)
    last_print = 0.0
    while not all(segment in buffer.packets for segment in required_segments):
        now = time.monotonic()
        if now - last_print >= 1.0:
            missing = [segment.name.lower() for segment in required_segments if segment not in buffer.packets]
            print(f"  missing: {', '.join(missing)}", file=sys.stderr)
            last_print = now
        time.sleep(0.05)
    print("All required segments are streaming.", file=sys.stderr)


def make_payload(
    *,
    frame_index: int,
    timestamp_s: float,
    qpos: np.ndarray,
    qvel: np.ndarray,
    solve_us: float,
    config: str,
    yaw_mode: str,
) -> bytes:
    payload = {
        "frame_index": frame_index,
        "timestamp_s": timestamp_s,
        "config": config,
        "yaw_mode": yaw_mode,
        "qpos": [float(value) for value in qpos],
        "qvel": [float(value) for value in qvel],
        "solve_us": solve_us,
    }
    return (json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8")


def main() -> None:
    args = parse_args()
    if args.fps <= 0.0:
        raise ValueError("--fps must be greater than zero")

    required_segments = PARTIAL_CONFIGS[args.config]
    max_age_s = args.max_age_ms / 1000.0
    frame_period_s = 1.0 / args.fps

    print(
        f"Live ch_robot retarget: {args.config} on {args.host}:{args.port}, "
        f"output={args.output}",
        file=sys.stderr,
    )

    buffer = LatestPacketBuffer()
    packet_filter = QuaternionPacketFilter()
    filtered: dict[SegmentId, Rotation] = {}
    filter_lock = threading.Lock()
    stop_event = threading.Event()

    def receive_loop() -> None:
        for packet in receive_quaternion_packets(host=args.host, port=args.port, timeout_s=0.25):
            if stop_event.is_set():
                break
            buffer.update(packet)
            result = packet_filter.update(packet)
            if result.rotation is not None:
                with filter_lock:
                    filtered[packet.segment_id] = result.rotation

    recv_thread = threading.Thread(target=receive_loop, daemon=True)
    recv_thread.start()

    udp_socket: socket.socket | None = None
    udp_target = (args.target_host, args.target_port)
    if args.output == "udp":
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"Sending retarget frames to udp://{args.target_host}:{args.target_port}", file=sys.stderr)

    qvel_diff = QvelFiniteDifferencer()
    frame_index = 0
    valid_frames = 0
    stale_frames = 0
    solve_times_us: list[float] = []
    started_s = time.monotonic()
    next_tick_s = started_s

    try:
        wait_for_segments(buffer, required_segments)
        profile = None if args.no_calibration else calibrate(buffer, required_segments, args.min_samples)
        print("Retarget loop started.", file=sys.stderr)
        started_s = time.monotonic()
        next_tick_s = started_s

        while args.duration_s <= 0.0 or (time.monotonic() - started_s) < args.duration_s:
            now = time.monotonic()
            if now < next_tick_s:
                time.sleep(min(next_tick_s - now, 0.002))
                continue
            next_tick_s += frame_period_s

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
                continue

            skeleton = aggregate_lower_body_skeleton(orientations)
            t0 = time.perf_counter()
            qpos = legposes_to_qpos(
                skeleton.joint_rotations,
                skeleton.segment_orientations[SegmentId.PELVIS],
                base_height=args.base_height,
                yaw_mode=args.yaw_mode,
            )
            qvel = qvel_diff.update(qpos, time.monotonic())
            solve_us = (time.perf_counter() - t0) * 1_000_000.0
            solve_times_us.append(solve_us)

            payload = make_payload(
                frame_index=frame_index,
                timestamp_s=time.time(),
                qpos=qpos,
                qvel=qvel,
                solve_us=solve_us,
                config=args.config,
                yaw_mode=args.yaw_mode,
            )
            if args.output == "stdout":
                sys.stdout.buffer.write(payload)
                sys.stdout.buffer.flush()
            elif udp_socket is not None:
                udp_socket.sendto(payload, udp_target)

            valid_frames += 1
            frame_index += 1
            if args.print_every > 0 and valid_frames % args.print_every == 0:
                recent = solve_times_us[-args.print_every :]
                print(
                    f"frames={valid_frames} stale={stale_frames} "
                    f"solve_us_mean={np.mean(recent):.1f} solve_us_max={np.max(recent):.1f}",
                    file=sys.stderr,
                )
    except KeyboardInterrupt:
        print("Stopping live retarget bridge.", file=sys.stderr)
    finally:
        stop_event.set()
        recv_thread.join(timeout=1.0)
        if udp_socket is not None:
            udp_socket.close()
        if solve_times_us:
            print(
                f"Summary: frames={valid_frames}, stale={stale_frames}, "
                f"solve_us_mean={np.mean(solve_times_us):.1f}, solve_us_max={np.max(solve_times_us):.1f}",
                file=sys.stderr,
            )


if __name__ == "__main__":
    main()
