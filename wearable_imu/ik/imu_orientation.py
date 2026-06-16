"""Orientation-based joint solving from IMUs mounted on body segments.

Frame convention used here:
- segment/body axes: +X forward, +Y left, +Z up
- sensor axes: +X device top, +Y device left edge, +Z outward from the face

The user-facing mounting assumption is:
- thigh and shank IMUs are on the outside/lateral face
- left leg lateral face points +Y, right leg lateral face points -Y
- thigh/shank device top points upward toward the hip
- foot IMU sits on the top of the foot, face pointing +Z, top pointing forward
- pelvis IMU is centered on the front of the pelvis, face pointing +X,
  top pointing +Z
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation


Vector3 = NDArray[np.float64]
MountMap = Mapping[str, Rotation]
OrientationMap = Mapping[str, Rotation]


X_FORWARD = np.array([1.0, 0.0, 0.0], dtype=float)
Y_LEFT = np.array([0.0, 1.0, 0.0], dtype=float)
Z_UP = np.array([0.0, 0.0, 1.0], dtype=float)


@dataclass(frozen=True)
class LowerLimbJointRotations:
    """Relative joint rotations recovered from segment orientations.

    ``knee`` and ``ankle`` are ``None`` when the corresponding distal IMU
    (shank or foot) was absent; the joint is assumed neutral in that case.
    """

    hip: Rotation | None = None
    knee: Rotation | None = None
    ankle: Rotation | None = None

    def euler_xyz_degrees(self) -> dict[str, tuple[float, float, float] | None]:
        return {
            name: tuple(rot.as_euler("xyz", degrees=True)) if rot is not None else None
            for name, rot in (("hip", self.hip), ("knee", self.knee), ("ankle", self.ankle))
        }


@dataclass(frozen=True)
class LowerLimbOrientationSolution:
    """Segment orientations and relative joint rotations from IMU data."""

    segment_orientations: dict[str, Rotation]
    joints: LowerLimbJointRotations


def sensor_to_segment_from_axes(
    *,
    sensor_x_axis_in_segment: Vector3,
    sensor_z_axis_in_segment: Vector3,
) -> Rotation:
    """Build the fixed sensor-to-segment rotation from two mounted axes.

    The returned rotation maps vectors in sensor coordinates into segment
    coordinates. The sensor +Y axis is inferred to keep a right-handed frame.
    """

    x_axis = _unit(sensor_x_axis_in_segment)
    z_axis = _unit(sensor_z_axis_in_segment)

    if abs(float(np.dot(x_axis, z_axis))) > 1e-9:
        raise ValueError("sensor +X and +Z axes must be orthogonal")

    y_axis = np.cross(z_axis, x_axis)
    matrix = np.column_stack((x_axis, y_axis, z_axis))
    return Rotation.from_matrix(matrix)


def lateral_thigh_or_shank_mount(side: str) -> Rotation:
    """Mount for thigh/shank IMUs placed on the outside face of the leg."""

    side_normal = _outside_normal(side)
    return sensor_to_segment_from_axes(
        sensor_x_axis_in_segment=Z_UP,
        sensor_z_axis_in_segment=side_normal,
    )


def top_foot_mount() -> Rotation:
    """Mount for a foot IMU placed on top of the foot while flat."""

    return sensor_to_segment_from_axes(
        sensor_x_axis_in_segment=X_FORWARD,
        sensor_z_axis_in_segment=Z_UP,
    )


def front_pelvis_mount() -> Rotation:
    """Mount for a pelvis IMU placed on the front of the pelvis."""

    return sensor_to_segment_from_axes(
        sensor_x_axis_in_segment=Z_UP,
        sensor_z_axis_in_segment=X_FORWARD,
    )


def default_lower_limb_mounts(side: str) -> dict[str, Rotation]:
    """Default mounts for the described thigh, shank, and foot wearables."""

    lateral_mount = lateral_thigh_or_shank_mount(side)
    return {
        "thigh": lateral_mount,
        "shank": lateral_mount,
        "foot": top_foot_mount(),
    }


def segment_orientation_from_imu(
    imu_world_sensor: Rotation,
    sensor_to_segment: Rotation,
) -> Rotation:
    """Convert world-from-sensor IMU orientation into world-from-segment."""

    return imu_world_sensor * sensor_to_segment.inv()


def imu_orientation_from_segment(
    segment_world: Rotation,
    sensor_to_segment: Rotation,
) -> Rotation:
    """Convert a known segment orientation into the IMU orientation it implies."""

    return segment_world * sensor_to_segment


def solve_lower_limb_joints_from_imus(
    imu_world_sensor: OrientationMap,
    *,
    side: str,
    mounts: MountMap | None = None,
) -> LowerLimbOrientationSolution:
    """Recover lower-limb joint rotations from segment-mounted IMUs.

    Required IMU: ``thigh``.
    Optional IMUs: ``shank`` and ``foot`` (foot is silently ignored without shank).
    When a distal IMU is absent its joint rotation is ``None`` and the segment
    orientation is inherited from its proximal neighbour (neutral/straight assumption).
    Optional IMU: ``pelvis``. If present, hip rotation is recovered as pelvis-to-thigh.
    """

    sensor_to_segment = dict(default_lower_limb_mounts(side))
    if mounts is not None:
        sensor_to_segment.update(mounts)

    if "thigh" not in imu_world_sensor or "thigh" not in sensor_to_segment:
        raise ValueError("missing required IMU or mount: thigh")

    thigh_seg = segment_orientation_from_imu(
        imu_world_sensor["thigh"], sensor_to_segment["thigh"]
    )
    segment_orientations: dict[str, Rotation] = {"thigh": thigh_seg}

    hip: Rotation | None = None
    if "pelvis" in imu_world_sensor:
        if "pelvis" not in sensor_to_segment:
            raise ValueError("pelvis IMU requires a pelvis mount calibration")
        pelvis = segment_orientation_from_imu(
            imu_world_sensor["pelvis"], sensor_to_segment["pelvis"]
        )
        segment_orientations["pelvis"] = pelvis
        hip = relative_rotation(pelvis, thigh_seg)

    knee: Rotation | None = None
    shank_seg = thigh_seg
    if "shank" in imu_world_sensor:
        shank_seg = segment_orientation_from_imu(
            imu_world_sensor["shank"], sensor_to_segment["shank"]
        )
        segment_orientations["shank"] = shank_seg
        knee = relative_rotation(thigh_seg, shank_seg)

    ankle: Rotation | None = None
    if "foot" in imu_world_sensor and "shank" in imu_world_sensor:
        foot_seg = segment_orientation_from_imu(
            imu_world_sensor["foot"], sensor_to_segment["foot"]
        )
        segment_orientations["foot"] = foot_seg
        ankle = relative_rotation(shank_seg, foot_seg)

    joints = LowerLimbJointRotations(hip=hip, knee=knee, ankle=ankle)
    return LowerLimbOrientationSolution(
        segment_orientations=segment_orientations,
        joints=joints,
    )


def relative_rotation(parent_world: Rotation, child_world: Rotation) -> Rotation:
    """Return the child orientation expressed relative to the parent segment."""

    return parent_world.inv() * child_world


def _outside_normal(side: str) -> Vector3:
    if side == "left":
        return Y_LEFT
    if side == "right":
        return -Y_LEFT
    raise ValueError("side must be 'left' or 'right'")


def _unit(vector: Vector3) -> Vector3:
    norm = float(np.linalg.norm(vector))
    if norm == 0.0:
        raise ValueError("axis vector must be nonzero")
    return vector / norm
