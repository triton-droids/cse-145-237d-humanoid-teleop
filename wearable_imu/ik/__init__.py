"""Inverse kinematics helpers for the lower-body pose workflow."""

from .planar_leg import (
    FootTarget,
    JointAngles,
    LegIKSolution,
    LegSegmentLengths,
    forward_kinematics,
    solve_leg_ik,
)
from .imu_orientation import (
    LowerLimbJointRotations,
    LowerLimbOrientationSolution,
    default_lower_limb_mounts,
    front_pelvis_mount,
    imu_orientation_from_segment,
    relative_rotation,
    segment_orientation_from_imu,
    solve_lower_limb_joints_from_imus,
)

__all__ = [
    "FootTarget",
    "JointAngles",
    "LegIKSolution",
    "LegSegmentLengths",
    "forward_kinematics",
    "LowerLimbJointRotations",
    "LowerLimbOrientationSolution",
    "default_lower_limb_mounts",
    "front_pelvis_mount",
    "imu_orientation_from_segment",
    "relative_rotation",
    "segment_orientation_from_imu",
    "solve_leg_ik",
    "solve_lower_limb_joints_from_imus",
]
