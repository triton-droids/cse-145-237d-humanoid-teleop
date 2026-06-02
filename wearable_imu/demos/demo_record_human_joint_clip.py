"""Record live IMU-derived human joint positions for ML retargeting.

Usage (from project root):
  conda run --no-capture-output -n humanoid-sim python demos/demo_record_human_joint_clip.py --config shanks --duration-s 10
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from calibration.neutral import CalibrationProfile, NeutralCalibrationAccumulator  # noqa: E402
from ik.imu_orientation import default_lower_limb_mounts, front_pelvis_mount  # noqa: E402
from ik.lower_body_aggregation import LowerBodySkeleton, aggregate_lower_body_skeleton  # noqa: E402
from sensor.filtering import QuaternionPacketFilter  # noqa: E402
from sensor.packet import SegmentId  # noqa: E402
from sensor.udp_receiver import LatestPacketBuffer, receive_quaternion_packets  # noqa: E402


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

JOINT_NAMES = (
    "Spine1",
    "LeftUpLeg",
    "LeftLeg",
    "LeftFoot",
    "LeftToeBase",
    "RightUpLeg",
    "RightLeg",
    "RightFoot",
    "RightToeBase",
)

SKELETON_JOINT_KEYS = (
    "pelvis",
    "left_hip",
    "left_knee",
    "left_ankle",
    "left_toe",
    "right_hip",
    "right_knee",
    "right_ankle",
    "right_toe",
)

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
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--config", choices=list(PARTIAL_CONFIGS), default="shanks")
    parser.add_argument("--duration-s", type=float, default=10.0)
    parser.add_argument("--fps", type=float, default=50.0)
    parser.add_argument("--max-age-ms", type=float, default=150.0)
    parser.add_argument("--min-samples", type=int, default=20)
    parser.add_argument("--calibration-delay-s", type=float, default=1.0)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("data/recordings/human_joint_clip.npz"),
        help="Output .npz file for ML retargeting experiments.",
    )
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

    print(f"Calibrating neutral pose ({min_samples} samples per segment)")
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
            print(f"  {progress}", end="\r", flush=True)
            last_print = now
        time.sleep(0.01)

    print("\nCalibration complete.")
    return accumulator.build_profile()


def calibrated_segment_orientations(
    filtered: dict[SegmentId, Rotation],
    buffer: LatestPacketBuffer,
    profile: CalibrationProfile,
    required_segments: tuple[SegmentId, ...],
    max_age_s: float,
) -> dict[SegmentId, Rotation] | None:
    now = time.monotonic()
    segment_orientations: dict[SegmentId, Rotation] = {}

    for segment in required_segments:
        packet = buffer.packets.get(segment)
        if packet is None or (now - packet.receive_time_s) > max_age_s:
            return None

        raw = filtered.get(segment)
        if raw is None:
            return None

        mount = _SEGMENT_MOUNTS[segment]
        neutral_segment = profile.neutral_orientations[segment] * mount.inv()
        live_segment = raw * mount.inv()
        segment_orientations[segment] = neutral_segment.inv() * live_segment

    return segment_orientations


def ml_joint_positions_w(skeleton: LowerBodySkeleton) -> np.ndarray:
    """Return one frame of ML joints in ``JOINT_NAMES`` order."""

    joints = skeleton.joints
    return np.vstack([joints[key] for key in SKELETON_JOINT_KEYS])


def pelvis_ground_origin_w(frame0_points_w: np.ndarray) -> np.ndarray:
    """Return the frame-0 pelvis projected onto the floor."""

    origin = np.array(frame0_points_w[0], dtype=float)
    origin[2] = 0.0
    return origin


def origin_relative_points(points_w: np.ndarray, origin_w: np.ndarray) -> np.ndarray:
    """Express a point frame relative to the fixed ML origin."""

    return points_w - origin_w


def frame_as_dict(points: np.ndarray) -> dict[str, list[float]]:
    """Return one frame as the JSON-like mapping requested by ML."""

    return {
        name: [float(value) for value in point]
        for name, point in zip(JOINT_NAMES, points)
    }


def save_ml_joint_clip(
    *,
    output: Path,
    point_frames_w: list[np.ndarray],
    point_frames_origin: list[np.ndarray],
    root_pos_frames_w: list[np.ndarray],
    root_quat_frames_wxyz: list[np.ndarray],
    timestamps_s: list[float],
    fps: float,
    config: str,
    required_segments: tuple[SegmentId, ...],
    pelvis_ground_origin: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Save an ML handoff clip and return ``(joint_pos_w, joint_pos_origin)``."""

    if not point_frames_w:
        raise ValueError("cannot save an empty ML joint clip")

    output.parent.mkdir(parents=True, exist_ok=True)
    point_array_w = np.stack(point_frames_w)
    point_array_origin = np.stack(point_frames_origin)
    root_pos_array_w = np.stack(root_pos_frames_w)
    root_quat_array_wxyz = np.stack(root_quat_frames_wxyz)

    np.savez(
        output,
        point_names=np.array(JOINT_NAMES),
        joint_names=np.array(JOINT_NAMES),
        joint_pos_origin=point_array_origin,
        joint_pos_w=point_array_w,
        pelvis_ground_origin_w=pelvis_ground_origin,
        frame0_pelvis_w=point_array_w[0, 0].copy(),
        root_pos_w=root_pos_array_w,
        root_quat_wxyz=root_quat_array_wxyz,
        timestamps_s=np.array(timestamps_s, dtype=float),
        fps=np.array([fps], dtype=float),
        config=np.array([config]),
        required_segments=np.array([segment.name.lower() for segment in required_segments]),
        foot_definition=np.array(["LeftToeBase/RightToeBase are generated toe points from the body model"]),
    )
    return point_array_w, point_array_origin


def wait_for_segments(buffer: LatestPacketBuffer, required_segments: tuple[SegmentId, ...]) -> None:
    print("Waiting for required IMU segments...")
    last_print = 0.0
    while not all(segment in buffer.packets for segment in required_segments):
        now = time.monotonic()
        if now - last_print >= 1.0:
            missing = [segment.name.lower() for segment in required_segments if segment not in buffer.packets]
            print(f"  missing: {', '.join(missing)}")
            last_print = now
        time.sleep(0.05)
    print("All required segments are streaming.")


def main() -> None:
    args = parse_args()
    required_segments = PARTIAL_CONFIGS[args.config]
    max_age_s = args.max_age_ms / 1000.0

    print("Human joint clip recorder")
    print(f"  config : {args.config}")
    print(f"  address: {args.host}:{args.port}")
    print(f"  output : {args.output}")
    print(f"  points : {', '.join(JOINT_NAMES)}")

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

    receiver_thread = threading.Thread(target=receive_loop, daemon=True)
    receiver_thread.start()

    try:
        wait_for_segments(buffer, required_segments)
        print(f"Stand still in neutral pose. Calibration starts in {args.calibration_delay_s:.1f}s.")
        time.sleep(max(0.0, args.calibration_delay_s))
        profile = calibrate(buffer, required_segments, args.min_samples)

        frame_period_s = 1.0 / args.fps
        start_time = time.monotonic()
        next_sample_time = start_time
        target_frames = max(1, int(round(args.duration_s * args.fps)))

        timestamps_s: list[float] = []
        point_frames_w: list[np.ndarray] = []
        point_frames_origin: list[np.ndarray] = []
        root_pos_frames_w: list[np.ndarray] = []
        root_quat_frames_wxyz: list[np.ndarray] = []
        origin_w: np.ndarray | None = None
        skipped = 0

        print(f"Recording {target_frames} frames at {args.fps:g} FPS. Move now.")
        while len(point_frames_w) < target_frames:
            now = time.monotonic()
            if now < next_sample_time:
                time.sleep(min(0.005, next_sample_time - now))
                continue
            next_sample_time += frame_period_s

            with filter_lock:
                filtered_snapshot = dict(filtered)

            segment_orientations = calibrated_segment_orientations(
                filtered_snapshot,
                buffer,
                profile,
                required_segments,
                max_age_s,
            )
            if segment_orientations is None:
                skipped += 1
                continue

            skeleton = aggregate_lower_body_skeleton(segment_orientations)
            points_w = ml_joint_positions_w(skeleton)
            if origin_w is None:
                origin_w = pelvis_ground_origin_w(points_w)
            root_pos_w = skeleton.joints["pelvis"]
            root_rotation = skeleton.segment_orientations[SegmentId.PELVIS]
            root_quat_xyzw = root_rotation.as_quat()
            root_quat_wxyz = np.array(
                [root_quat_xyzw[3], root_quat_xyzw[0], root_quat_xyzw[1], root_quat_xyzw[2]],
                dtype=float,
            )

            timestamps_s.append(now - start_time)
            point_frames_w.append(points_w)
            point_frames_origin.append(origin_relative_points(points_w, origin_w))
            root_pos_frames_w.append(root_pos_w.copy())
            root_quat_frames_wxyz.append(root_quat_wxyz)

            if len(point_frames_w) % max(1, int(args.fps)) == 0:
                print(f"  recorded {len(point_frames_w)}/{target_frames} frames")

        if origin_w is None:
            raise RuntimeError("recording finished without a valid frame")

        point_array_w, point_array_origin = save_ml_joint_clip(
            output=args.output,
            point_frames_w=point_frames_w,
            point_frames_origin=point_frames_origin,
            root_pos_frames_w=root_pos_frames_w,
            root_quat_frames_wxyz=root_quat_frames_wxyz,
            timestamps_s=timestamps_s,
            fps=args.fps,
            config=args.config,
            required_segments=required_segments,
            pelvis_ground_origin=origin_w,
        )
        print(f"Saved clip: {args.output}")
        print(f"  joint_pos_origin: {point_array_origin.shape}")
        print(f"  joint_pos_w     : {point_array_w.shape}")
        print(f"  sample frame    : {frame_as_dict(point_array_origin[0])}")
        print(f"  skipped stale frames: {skipped}")
    finally:
        stop_event.set()


if __name__ == "__main__":
    main()
