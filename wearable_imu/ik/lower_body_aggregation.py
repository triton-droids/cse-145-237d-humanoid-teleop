"""Aggregate calibrated lower-body segment orientations into one skeleton."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

import numpy as np
from scipy.spatial.transform import Rotation

from ik.imu_orientation import relative_rotation
from model.lower_body import LowerBodyDimensions, LegPose, SegmentFrame, Side
from sensor.packet import SegmentId


@dataclass(frozen=True)
class LowerBodySkeleton:
    """Aggregated lower-body skeleton from partial or full segment orientations.

    ``available_segments`` records which segments had real IMU data.  All other
    segments were filled with their proximal neighbour's orientation (neutral
    joint assumption) and their ``LegPose`` joint rotation is ``None``.
    """

    pelvis_center: np.ndarray
    hip_centers: dict[Side, np.ndarray]
    joints: dict[str, np.ndarray]
    segments: dict[str, SegmentFrame]
    joint_rotations: dict[Side, LegPose]
    segment_orientations: dict[SegmentId, Rotation]
    available_segments: frozenset[SegmentId]


def aggregate_lower_body_skeleton(
    segment_orientations: Mapping[SegmentId, Rotation],
    *,
    dimensions: LowerBodyDimensions = LowerBodyDimensions(),
    pelvis_center: np.ndarray | None = None,
) -> LowerBodySkeleton:
    """Build a lower-body skeleton from available calibrated segment orientations.

    Missing distal segments are filled with their proximal neighbour's
    orientation (straight/neutral joint).  At minimum ``SegmentId.PELVIS`` plus
    one thigh per side should be provided; if even those are absent the pelvis
    defaults to identity and the thigh inherits it.

    The returned skeleton records which segments had real IMU data in
    ``available_segments``; the corresponding ``LegPose`` knee/ankle fields are
    ``None`` for estimated joints.
    """

    available = frozenset(segment_orientations.keys())
    orientations = _fill_missing_orientations(segment_orientations)

    pelvis_rotation = orientations[SegmentId.PELVIS]
    pelvis_origin = (
        np.array([0.0, 0.0, 1.0], dtype=float)
        if pelvis_center is None
        else np.asarray(pelvis_center, dtype=float)
    )

    hip_centers: dict[Side, np.ndarray] = {
        "left": pelvis_origin
        + pelvis_rotation.apply([0.0, dimensions.hip_spacing / 2.0, 0.0]),
        "right": pelvis_origin
        + pelvis_rotation.apply([0.0, -dimensions.hip_spacing / 2.0, 0.0]),
    }
    joints: dict[str, np.ndarray] = {
        "pelvis": pelvis_origin,
        "left_hip": hip_centers["left"],
        "right_hip": hip_centers["right"],
    }
    segments: dict[str, SegmentFrame] = {}
    joint_rotations: dict[Side, LegPose] = {}

    _add_leg(
        side="left",
        hip_center=hip_centers["left"],
        thigh_rotation=orientations[SegmentId.LEFT_THIGH],
        shank_rotation=orientations[SegmentId.LEFT_SHANK],
        foot_rotation=orientations[SegmentId.LEFT_FOOT],
        dimensions=dimensions,
        joints=joints,
        segments=segments,
        joint_rotations=joint_rotations,
        pelvis_rotation=pelvis_rotation,
        has_shank=SegmentId.LEFT_SHANK in available,
        has_foot=SegmentId.LEFT_FOOT in available,
    )
    _add_leg(
        side="right",
        hip_center=hip_centers["right"],
        thigh_rotation=orientations[SegmentId.RIGHT_THIGH],
        shank_rotation=orientations[SegmentId.RIGHT_SHANK],
        foot_rotation=orientations[SegmentId.RIGHT_FOOT],
        dimensions=dimensions,
        joints=joints,
        segments=segments,
        joint_rotations=joint_rotations,
        pelvis_rotation=pelvis_rotation,
        has_shank=SegmentId.RIGHT_SHANK in available,
        has_foot=SegmentId.RIGHT_FOOT in available,
    )

    return LowerBodySkeleton(
        pelvis_center=pelvis_origin,
        hip_centers=hip_centers,
        joints=joints,
        segments=segments,
        joint_rotations=joint_rotations,
        segment_orientations=dict(orientations),
        available_segments=available,
    )


def _fill_missing_orientations(
    provided: Mapping[SegmentId, Rotation],
) -> dict[SegmentId, Rotation]:
    """Fill absent distal segments with parent orientation (neutral joint)."""
    filled: dict[SegmentId, Rotation] = dict(provided)
    pelvis = filled.setdefault(SegmentId.PELVIS, Rotation.identity())
    for thigh_id, shank_id, foot_id in (
        (SegmentId.LEFT_THIGH, SegmentId.LEFT_SHANK, SegmentId.LEFT_FOOT),
        (SegmentId.RIGHT_THIGH, SegmentId.RIGHT_SHANK, SegmentId.RIGHT_FOOT),
    ):
        thigh = filled.setdefault(thigh_id, pelvis)
        shank = filled.setdefault(shank_id, thigh)
        filled.setdefault(foot_id, shank)
    return filled


def _add_leg(
    *,
    side: Side,
    hip_center: np.ndarray,
    thigh_rotation: Rotation,
    shank_rotation: Rotation,
    foot_rotation: Rotation,
    dimensions: LowerBodyDimensions,
    joints: dict[str, np.ndarray],
    segments: dict[str, SegmentFrame],
    joint_rotations: dict[Side, LegPose],
    pelvis_rotation: Rotation,
    has_shank: bool,
    has_foot: bool,
) -> None:
    knee_center = hip_center + thigh_rotation.apply([0.0, 0.0, -dimensions.thigh_length])
    ankle_center = knee_center + shank_rotation.apply([0.0, 0.0, -dimensions.shank_length])
    heel_point = ankle_center + foot_rotation.apply([-dimensions.foot_heel_length, 0.0, 0.0])
    toe_point = ankle_center + foot_rotation.apply([dimensions.foot_toe_length, 0.0, 0.0])

    joints[f"{side}_knee"] = knee_center
    joints[f"{side}_ankle"] = ankle_center
    joints[f"{side}_heel"] = heel_point
    joints[f"{side}_toe"] = toe_point

    thigh_name = f"{side}_thigh"
    shank_name = f"{side}_shank"
    foot_name = f"{side}_foot"

    segments[thigh_name] = SegmentFrame(
        name=thigh_name,
        origin=hip_center,
        rotation=thigh_rotation,
        distal_point=knee_center,
        visual_points=np.vstack([hip_center, knee_center]),
    )
    segments[shank_name] = SegmentFrame(
        name=shank_name,
        origin=knee_center,
        rotation=shank_rotation,
        distal_point=ankle_center,
        visual_points=np.vstack([knee_center, ankle_center]),
    )
    segments[foot_name] = SegmentFrame(
        name=foot_name,
        origin=ankle_center,
        rotation=foot_rotation,
        distal_point=toe_point,
        visual_points=np.vstack([heel_point, ankle_center, toe_point]),
    )
    joint_rotations[side] = LegPose(
        hip=relative_rotation(pelvis_rotation, thigh_rotation),
        knee=relative_rotation(thigh_rotation, shank_rotation) if has_shank else None,
        ankle=relative_rotation(shank_rotation, foot_rotation) if has_foot else None,
    )
