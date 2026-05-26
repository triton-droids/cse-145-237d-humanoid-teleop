"""Solve lower-limb joint rotations from segment-mounted IMU orientations."""

# Usage (from project root):
#   conda run -p .\.conda python demos\demo_imu_orientation_ik.py

from __future__ import annotations

from pathlib import Path
import sys

from scipy.spatial.transform import Rotation

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from ik.imu_orientation import (  # noqa: E402
    default_lower_limb_mounts,
    front_pelvis_mount,
    imu_orientation_from_segment,
    solve_lower_limb_joints_from_imus,
)


def main() -> None:
    side = "left"

    pelvis_world = Rotation.identity()
    hip = Rotation.from_euler("xyz", [6.0, 18.0, 2.0], degrees=True)
    knee = Rotation.from_euler("xyz", [0.0, 52.0, 0.0], degrees=True)
    ankle = Rotation.from_euler("xyz", [0.0, -14.0, 4.0], degrees=True)

    thigh_world = pelvis_world * hip
    shank_world = thigh_world * knee
    foot_world = shank_world * ankle

    mounts = default_lower_limb_mounts(side)
    mounts["pelvis"] = front_pelvis_mount()

    imu_world_sensor = {
        "pelvis": imu_orientation_from_segment(pelvis_world, mounts["pelvis"]),
        "thigh": imu_orientation_from_segment(thigh_world, mounts["thigh"]),
        "shank": imu_orientation_from_segment(shank_world, mounts["shank"]),
        "foot": imu_orientation_from_segment(foot_world, mounts["foot"]),
    }

    solution = solve_lower_limb_joints_from_imus(
        imu_world_sensor,
        side=side,
        mounts=mounts,
    )

    print("Orientation IK from mounted IMUs")
    print(f"side: {side}")
    print()
    print("Recovered relative joint rotations, xyz Euler degrees")
    for joint, angles in solution.joints.euler_xyz_degrees().items():
        if angles is None:
            print(f"  {joint}: no pelvis IMU")
        else:
            x, y, z = angles
            print(f"  {joint}: x={x:8.3f}, y={y:8.3f}, z={z:8.3f}")


if __name__ == "__main__":
    main()
