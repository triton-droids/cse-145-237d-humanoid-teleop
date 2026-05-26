"""Aggregate seven segment orientations into a lower-body skeleton."""

# Usage (from project root):
#   conda run -p .\.conda python demos\demo_lower_body_aggregation.py

from __future__ import annotations

from pathlib import Path
import sys

from scipy.spatial.transform import Rotation

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from ik.lower_body_aggregation import aggregate_lower_body_skeleton  # noqa: E402
from sensor.packet import SegmentId  # noqa: E402


def main() -> None:
    pelvis = Rotation.from_euler("xyz", [1.0, -2.0, 4.0], degrees=True)

    left_hip = Rotation.from_euler("xyz", [5.0, 18.0, -2.0], degrees=True)
    left_knee = Rotation.from_euler("xyz", [0.0, 44.0, 0.0], degrees=True)
    left_ankle = Rotation.from_euler("xyz", [1.0, -11.0, 3.0], degrees=True)

    right_hip = Rotation.from_euler("xyz", [3.0, 12.0, 2.0], degrees=True)
    right_knee = Rotation.from_euler("xyz", [0.0, 29.0, 0.0], degrees=True)
    right_ankle = Rotation.from_euler("xyz", [-1.0, -7.0, -2.0], degrees=True)

    segment_orientations = {
        SegmentId.PELVIS: pelvis,
        SegmentId.LEFT_THIGH: pelvis * left_hip,
        SegmentId.LEFT_SHANK: pelvis * left_hip * left_knee,
        SegmentId.LEFT_FOOT: pelvis * left_hip * left_knee * left_ankle,
        SegmentId.RIGHT_THIGH: pelvis * right_hip,
        SegmentId.RIGHT_SHANK: pelvis * right_hip * right_knee,
        SegmentId.RIGHT_FOOT: pelvis * right_hip * right_knee * right_ankle,
    }

    skeleton = aggregate_lower_body_skeleton(segment_orientations)

    print("Aggregated lower-body skeleton")
    print()
    print("Joint positions, meters")
    for name in (
        "pelvis",
        "left_hip",
        "left_knee",
        "left_ankle",
        "left_heel",
        "left_toe",
        "right_hip",
        "right_knee",
        "right_ankle",
        "right_heel",
        "right_toe",
    ):
        x, y, z = skeleton.joints[name]
        print(f"  {name:<12} x={x: .3f} y={y: .3f} z={z: .3f}")

    print()
    print("Relative joint rotations, xyz Euler degrees")
    for side, pose in skeleton.joint_rotations.items():
        for joint_name, rotation in (
            ("hip", pose.hip),
            ("knee", pose.knee),
            ("ankle", pose.ankle),
        ):
            x, y, z = rotation.as_euler("xyz", degrees=True)
            print(f"  {side:<5} {joint_name:<5} x={x: 7.2f} y={y: 7.2f} z={z: 7.2f}")


if __name__ == "__main__":
    main()
