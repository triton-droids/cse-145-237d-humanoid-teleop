"""Minimal visual lower-body model with IMU mount geometry.

The lower-limb dimensions are meant to be a simple rigid-body proxy, not a
subject-specific fit. The defaults use average adult male dimensions where we
have them directly:

- femur/thigh link: 0.50 m
- tibia/shank link: 0.39 m
- foot heel-to-toe length: 0.271 m

The femur and tibia values are taken from radiographic adult means. The foot
length comes from the male ANSUR II mean. Since the kinematic origin for the
foot body is the ankle rather than the back of the heel, the heel/toe split
around the ankle remains an explicit modeling assumption.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from ik.imu_orientation import default_lower_limb_mounts, front_pelvis_mount


Vector3 = NDArray[np.float64]
Side = Literal["left", "right"]


@dataclass(frozen=True)
class LowerBodyDimensions:
    hip_spacing: float = 0.22
    thigh_length: float = 0.5000
    shank_length: float = 0.3900
    foot_total_length: float = 0.2710
    foot_heel_length: float = 0.0950
    pelvis_front_offset: float = 0.06
    pelvis_imu_height_offset: float = 0.01
    thigh_lateral_offset: float = 0.06
    shank_lateral_offset: float = 0.05
    foot_height: float = 0.04

    @property
    def foot_toe_length(self) -> float:
        return self.foot_total_length - self.foot_heel_length


@dataclass(frozen=True)
class LegPose:
    hip: Rotation
    knee: Rotation
    ankle: Rotation


@dataclass(frozen=True)
class SegmentFrame:
    name: str
    origin: Vector3
    rotation: Rotation
    distal_point: Vector3
    visual_points: NDArray[np.float64]


@dataclass(frozen=True)
class IMUMountPose:
    segment_name: str
    position: Vector3
    sensor_rotation_world: Rotation


@dataclass(frozen=True)
class LowerBodyModel:
    pelvis_center: Vector3
    hip_centers: dict[Side, Vector3]
    segments: dict[str, SegmentFrame]
    imu_mounts: dict[str, IMUMountPose]


def default_pose() -> dict[Side, LegPose]:
    return {
        "left": LegPose(
            hip=Rotation.from_euler("xyz", [5.0, 18.0, 3.0], degrees=True),
            knee=Rotation.from_euler("xyz", [0.0, 42.0, 0.0], degrees=True),
            ankle=Rotation.from_euler("xyz", [0.0, -10.0, 2.0], degrees=True),
        ),
        "right": LegPose(
            hip=Rotation.from_euler("xyz", [3.0, 10.0, -2.0], degrees=True),
            knee=Rotation.from_euler("xyz", [0.0, 28.0, 0.0], degrees=True),
            ankle=Rotation.from_euler("xyz", [0.0, -6.0, -1.0], degrees=True),
        ),
    }


def build_lower_body_model(
    poses: dict[Side, LegPose] | None = None,
    dimensions: LowerBodyDimensions = LowerBodyDimensions(),
) -> LowerBodyModel:
    if poses is None:
        poses = default_pose()

    pelvis_center = np.array([0.0, 0.0, 1.0], dtype=float)
    hip_centers = {
        "left": pelvis_center + np.array([0.0, dimensions.hip_spacing / 2.0, 0.0]),
        "right": pelvis_center + np.array([0.0, -dimensions.hip_spacing / 2.0, 0.0]),
    }

    segments: dict[str, SegmentFrame] = {}
    imu_mounts: dict[str, IMUMountPose] = {}

    imu_mounts["pelvis"] = IMUMountPose(
        segment_name="pelvis",
        position=pelvis_center
        + np.array(
            [dimensions.pelvis_front_offset, 0.0, dimensions.pelvis_imu_height_offset],
            dtype=float,
        ),
        sensor_rotation_world=front_pelvis_mount(),
    )

    for side in ("left", "right"):
        pose = poses[side]
        sign = 1.0 if side == "left" else -1.0
        hip_center = hip_centers[side]

        thigh_rotation = pose.hip
        knee_center = hip_center + thigh_rotation.apply([0.0, 0.0, -dimensions.thigh_length])

        shank_rotation = thigh_rotation * pose.knee
        ankle_center = knee_center + shank_rotation.apply([0.0, 0.0, -dimensions.shank_length])

        foot_rotation = shank_rotation * pose.ankle
        heel_point = ankle_center + foot_rotation.apply([-dimensions.foot_heel_length, 0.0, 0.0])
        toe_point = ankle_center + foot_rotation.apply([dimensions.foot_toe_length, 0.0, 0.0])

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

        mounts = default_lower_limb_mounts(side)

        thigh_imu_local = np.array([0.0, sign * dimensions.thigh_lateral_offset, -0.5 * dimensions.thigh_length])
        shank_imu_local = np.array([0.0, sign * dimensions.shank_lateral_offset, -0.5 * dimensions.shank_length])
        foot_imu_local = np.array([0.5 * dimensions.foot_toe_length, 0.0, dimensions.foot_height])

        imu_mounts[thigh_name] = IMUMountPose(
            segment_name=thigh_name,
            position=hip_center + thigh_rotation.apply(thigh_imu_local),
            sensor_rotation_world=thigh_rotation * mounts["thigh"],
        )
        imu_mounts[shank_name] = IMUMountPose(
            segment_name=shank_name,
            position=knee_center + shank_rotation.apply(shank_imu_local),
            sensor_rotation_world=shank_rotation * mounts["shank"],
        )
        imu_mounts[foot_name] = IMUMountPose(
            segment_name=foot_name,
            position=ankle_center + foot_rotation.apply(foot_imu_local),
            sensor_rotation_world=foot_rotation * mounts["foot"],
        )

    return LowerBodyModel(
        pelvis_center=pelvis_center,
        hip_centers=hip_centers,
        segments=segments,
        imu_mounts=imu_mounts,
    )
