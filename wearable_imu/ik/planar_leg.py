"""Planar lower-leg inverse kinematics.

This module gives us a small, testable IK target before the MuJoCo body model
exists. The leg lives in the sagittal plane with x forward and z upward.
Joint angles are measured from the vertical-down direction, with positive
rotation moving the segment toward +x.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import acos, atan2, cos, degrees, pi, sin

import numpy as np
from numpy.typing import NDArray


Vector2 = NDArray[np.float64]


@dataclass(frozen=True)
class LegSegmentLengths:
    """Segment lengths in meters."""

    thigh: float = 0.5000
    shank: float = 0.3900
    foot: float = 0.1760

    def __post_init__(self) -> None:
        for name, value in (
            ("thigh", self.thigh),
            ("shank", self.shank),
            ("foot", self.foot),
        ):
            if value <= 0:
                raise ValueError(f"{name} length must be positive")


@dataclass(frozen=True)
class JointAngles:
    """Planar hip, knee, and ankle angles in radians."""

    hip: float
    knee: float
    ankle: float

    @property
    def foot_angle(self) -> float:
        return self.hip + self.knee + self.ankle

    def as_array(self) -> NDArray[np.float64]:
        return np.array([self.hip, self.knee, self.ankle], dtype=float)

    def as_degrees(self) -> dict[str, float]:
        return {
            "hip": degrees(self.hip),
            "knee": degrees(self.knee),
            "ankle": degrees(self.ankle),
        }


@dataclass(frozen=True)
class FootTarget:
    """Desired foot-tip target relative to the hip frame."""

    x: float
    z: float
    angle: float

    @property
    def position(self) -> Vector2:
        return np.array([self.x, self.z], dtype=float)


@dataclass(frozen=True)
class LegIKSolution:
    """Result from the planar IK solve."""

    angles: JointAngles
    reachable: bool
    requested_ankle_position: Vector2
    solved_foot_position: Vector2
    position_error: float
    orientation_error: float


def _segment_vector(length: float, angle: float) -> Vector2:
    return np.array([length * sin(angle), -length * cos(angle)], dtype=float)


def _wrap_angle(angle: float) -> float:
    return (angle + pi) % (2.0 * pi) - pi


def forward_kinematics(
    angles: JointAngles,
    lengths: LegSegmentLengths = LegSegmentLengths(),
) -> dict[str, Vector2 | float]:
    """Return hip, knee, ankle, and foot-tip positions for joint angles."""

    hip = np.array([0.0, 0.0], dtype=float)
    thigh_angle = angles.hip
    shank_angle = angles.hip + angles.knee
    foot_angle = angles.foot_angle

    knee = hip + _segment_vector(lengths.thigh, thigh_angle)
    ankle = knee + _segment_vector(lengths.shank, shank_angle)
    foot = ankle + _segment_vector(lengths.foot, foot_angle)

    return {
        "hip": hip,
        "knee": knee,
        "ankle": ankle,
        "foot": foot,
        "thigh_angle": thigh_angle,
        "shank_angle": shank_angle,
        "foot_angle": foot_angle,
    }


def solve_leg_ik(
    target: FootTarget,
    lengths: LegSegmentLengths = LegSegmentLengths(),
    *,
    knee_direction: int = 1,
) -> LegIKSolution:
    """Solve planar hip, knee, and ankle angles for a target foot pose.

    The requested foot angle fixes the foot segment orientation, so the solver
    first backs out the ankle target and then solves the two-link hip/knee
    problem analytically.

    Args:
        target: Desired foot-tip position and foot angle relative to the hip.
        lengths: Segment lengths in meters.
        knee_direction: Selects the IK branch. Use ``1`` for positive knee
            bend and ``-1`` for the mirrored branch.
    """

    if knee_direction not in (-1, 1):
        raise ValueError("knee_direction must be either 1 or -1")

    foot_vector = _segment_vector(lengths.foot, target.angle)
    ankle_target = target.position - foot_vector
    ankle_distance = float(np.linalg.norm(ankle_target))

    max_reach = lengths.thigh + lengths.shank
    min_reach = abs(lengths.thigh - lengths.shank)
    reachable = min_reach <= ankle_distance <= max_reach

    if ankle_distance == 0.0:
        ankle_direction = np.array([0.0, -1.0], dtype=float)
    else:
        ankle_direction = ankle_target / ankle_distance

    effective_distance = float(np.clip(ankle_distance, min_reach, max_reach))
    effective_ankle = ankle_direction * effective_distance

    cos_knee = (
        effective_distance**2 - lengths.thigh**2 - lengths.shank**2
    ) / (2.0 * lengths.thigh * lengths.shank)
    knee = knee_direction * acos(float(np.clip(cos_knee, -1.0, 1.0)))

    ankle_angle_from_vertical = atan2(effective_ankle[0], -effective_ankle[1])
    triangle_offset = atan2(
        lengths.shank * sin(knee),
        lengths.thigh + lengths.shank * cos(knee),
    )
    hip = ankle_angle_from_vertical - triangle_offset
    ankle = target.angle - hip - knee

    angles = JointAngles(hip=hip, knee=knee, ankle=ankle)
    solved = forward_kinematics(angles, lengths)
    solved_foot = solved["foot"]
    assert isinstance(solved_foot, np.ndarray)

    position_error = float(np.linalg.norm(solved_foot - target.position))
    orientation_error = abs(_wrap_angle(angles.foot_angle - target.angle))

    return LegIKSolution(
        angles=angles,
        reachable=reachable,
        requested_ankle_position=ankle_target,
        solved_foot_position=solved_foot,
        position_error=position_error,
        orientation_error=orientation_error,
    )
