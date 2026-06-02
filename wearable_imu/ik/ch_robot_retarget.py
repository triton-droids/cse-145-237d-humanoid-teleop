"""Fast live IMU retargeting for the 10-DOF ``ch_robot`` lower body.

This module converts calibrated human relative joint rotations into the compact
MuJoCo qpos layout used by the ch_robot Holosoma contract:

``x, y, z, qw, qx, qy, qz`` followed by the 10 lower-body hinge joints.

It is intentionally a direct geometric projection, not an optimizer.  The
coordinate mapping below follows the current IMU pipeline convention
(``+X`` forward, ``+Y`` left, ``+Z`` up) and the ch_robot convention inferred
from the MJCF (``+Y`` forward, ``+X`` lateral, ``+Z`` up).  If the ch_robot MJCF
is available, validate signs in the viewer with single-axis poses before using
this for hardware control.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Literal, Mapping, Protocol

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation


Vector3 = NDArray[np.float64]
Side = Literal["left", "right"]
YawMode = Literal["keep", "strip"]


class LegPoseLike(Protocol):
    hip: Rotation
    knee: Rotation | None
    ankle: Rotation | None


@dataclass(frozen=True)
class ReconstructedLegPose:
    """LegPose-compatible rotations reconstructed from joint positions."""

    hip: Rotation
    knee: Rotation | None
    ankle: Rotation | None


@dataclass(frozen=True)
class HumanJointClip:
    """Recorded IMU handoff clip saved by ``demo_record_human_joint_clip.py``."""

    joint_positions: np.ndarray
    root_quat_wxyz: np.ndarray | None
    timestamps_s: np.ndarray
    fps: float
    joint_names: tuple[str, ...]
    frame_key: str


CH_ROBOT_JOINT_NAMES: tuple[str, ...] = (
    "left_hip1_joint",
    "left_hip2_joint",
    "left_thigh_joint",
    "left_knee_joint",
    "left_ankle_joint",
    "right_hip1_joint",
    "right_hip2_joint",
    "right_thigh_joint",
    "right_knee_joint",
    "right_ankle_joint",
)

HUMAN_JOINT_NAMES: tuple[str, ...] = (
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

SPINE1_IDX = 0
LEFT_HIP_IDX = 1
LEFT_KNEE_IDX = 2
LEFT_ANKLE_IDX = 3
LEFT_TOE_IDX = 4
RIGHT_HIP_IDX = 5
RIGHT_KNEE_IDX = 6
RIGHT_ANKLE_IDX = 7
RIGHT_TOE_IDX = 8

QPOS_WIDTH = 7 + len(CH_ROBOT_JOINT_NAMES)
QVEL_WIDTH = 6 + len(CH_ROBOT_JOINT_NAMES)
FLOATING_BASE_QPOS_DIMS = 7
FLOATING_BASE_QVEL_DIMS = 6
BASE_HEIGHT_M = 0.765
JOINT_LIMIT_LOW = -1.57
JOINT_LIMIT_HIGH = 1.57

X_AXIS = np.array([1.0, 0.0, 0.0], dtype=np.float64)

# Human: +X forward, +Y left, +Z up.
# ch_robot: +Y forward, +X lateral/right, +Z up.
# Mapping: human +X -> robot +Y, human +Y -> robot -X, human +Z -> robot +Z.
HUMAN_TO_ROBOT_FRAME = Rotation.from_matrix(
    np.array(
        [
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
)

# Kept explicit so right-leg mirror corrections can be applied in one place
# after empirical ch_robot viewer validation.
SIDE_JOINT_SIGNS: dict[Side, np.ndarray] = {
    "left": np.ones(5, dtype=np.float64),
    "right": np.ones(5, dtype=np.float64),
}


@dataclass
class QvelFiniteDifferencer:
    """Finite-difference helper for the ch_robot 16-D qvel layout."""

    previous_qpos: np.ndarray | None = None
    previous_time_s: float | None = None

    def update(self, qpos: np.ndarray, timestamp_s: float) -> np.ndarray:
        """Return qvel for ``qpos`` and store it as the next previous sample."""

        current_qpos = np.asarray(qpos, dtype=np.float64)
        if current_qpos.shape != (QPOS_WIDTH,):
            raise ValueError(f"qpos must have shape ({QPOS_WIDTH},), got {current_qpos.shape}")

        if self.previous_qpos is None or self.previous_time_s is None:
            self.previous_qpos = current_qpos.copy()
            self.previous_time_s = float(timestamp_s)
            return np.zeros(QVEL_WIDTH, dtype=np.float64)

        dt = float(timestamp_s) - self.previous_time_s
        if dt <= 0.0:
            return np.zeros(QVEL_WIDTH, dtype=np.float64)

        qvel = qpos_to_qvel(self.previous_qpos, current_qpos, dt)
        self.previous_qpos = current_qpos.copy()
        self.previous_time_s = float(timestamp_s)
        return qvel

    def reset(self) -> None:
        self.previous_qpos = None
        self.previous_time_s = None


def legposes_to_qpos(
    joint_rotations: Mapping[Side, LegPoseLike],
    pelvis_orientation: Rotation | None = None,
    *,
    base_height: float = BASE_HEIGHT_M,
    yaw_mode: YawMode = "keep",
) -> np.ndarray:
    """Convert left/right ``LegPose`` rotations into a ch_robot qpos vector."""

    _require_sides(joint_rotations)

    qpos = np.zeros(QPOS_WIDTH, dtype=np.float64)
    qpos[:3] = np.array([0.0, 0.0, base_height], dtype=np.float64)
    qpos[3:7] = _wxyz_quat(
        _base_orientation_to_robot_frame(
            pelvis_orientation or Rotation.identity(),
            yaw_mode=yaw_mode,
        )
    )

    joint_values = np.array(
        [
            *_leg_joint_values("left", joint_rotations["left"]),
            *_leg_joint_values("right", joint_rotations["right"]),
        ],
        dtype=np.float64,
    )
    joint_values[np.abs(joint_values) < 1e-15] = 0.0
    qpos[FLOATING_BASE_QPOS_DIMS:] = np.clip(
        joint_values,
        JOINT_LIMIT_LOW,
        JOINT_LIMIT_HIGH,
    )
    assert qpos.shape == (QPOS_WIDTH,)
    return qpos


def joint_positions_to_legposes(
    joint_positions: np.ndarray,
    pelvis_orientation: Rotation | None = None,
) -> dict[Side, ReconstructedLegPose]:
    """Approximate ``LegPose`` rotations from one recorded 9-joint position frame.

    The recorded clip stores positions, not IMU segment quaternions.  This
    reconstructs segment frames from bone directions and the pelvis left-right
    axis.  Axial twist around thigh/shank bones is not observable from points
    alone, so this is the replay/offline approximation path.
    """

    points = np.asarray(joint_positions, dtype=np.float64)
    if points.shape != (len(HUMAN_JOINT_NAMES), 3):
        raise ValueError(
            f"joint_positions must have shape ({len(HUMAN_JOINT_NAMES)}, 3), got {points.shape}"
        )

    pelvis_rot = pelvis_orientation or _pelvis_frame_from_points(points)
    lateral_axis = _unit(points[LEFT_HIP_IDX] - points[RIGHT_HIP_IDX])

    left_thigh = _limb_segment_frame(
        proximal=points[LEFT_HIP_IDX],
        distal=points[LEFT_KNEE_IDX],
        lateral_axis=lateral_axis,
    )
    left_shank = _limb_segment_frame(
        proximal=points[LEFT_KNEE_IDX],
        distal=points[LEFT_ANKLE_IDX],
        lateral_axis=lateral_axis,
    )
    left_foot = _foot_segment_frame(
        ankle=points[LEFT_ANKLE_IDX],
        toe=points[LEFT_TOE_IDX],
        lateral_axis=lateral_axis,
    )

    right_thigh = _limb_segment_frame(
        proximal=points[RIGHT_HIP_IDX],
        distal=points[RIGHT_KNEE_IDX],
        lateral_axis=lateral_axis,
    )
    right_shank = _limb_segment_frame(
        proximal=points[RIGHT_KNEE_IDX],
        distal=points[RIGHT_ANKLE_IDX],
        lateral_axis=lateral_axis,
    )
    right_foot = _foot_segment_frame(
        ankle=points[RIGHT_ANKLE_IDX],
        toe=points[RIGHT_TOE_IDX],
        lateral_axis=lateral_axis,
    )

    return {
        "left": ReconstructedLegPose(
            hip=pelvis_rot.inv() * left_thigh,
            knee=left_thigh.inv() * left_shank,
            ankle=left_shank.inv() * left_foot,
        ),
        "right": ReconstructedLegPose(
            hip=pelvis_rot.inv() * right_thigh,
            knee=right_thigh.inv() * right_shank,
            ankle=right_shank.inv() * right_foot,
        ),
    }


def joint_positions_to_qpos(
    joint_positions: np.ndarray,
    pelvis_orientation: Rotation | None = None,
    *,
    base_height: float = BASE_HEIGHT_M,
    yaw_mode: YawMode = "keep",
) -> np.ndarray:
    """Convert one recorded 9-joint IMU handoff frame to ch_robot qpos."""

    legposes = joint_positions_to_legposes(joint_positions, pelvis_orientation)
    return legposes_to_qpos(
        legposes,
        pelvis_orientation,
        base_height=base_height,
        yaw_mode=yaw_mode,
    )


def load_human_joint_clip(path: str | Path, *, frame_key: str = "joint_pos_origin") -> HumanJointClip:
    """Load a recorded IMU handoff ``.npz`` clip."""

    clip_path = Path(path)
    data = np.load(clip_path, allow_pickle=True)
    if frame_key not in data.files:
        raise KeyError(f"{clip_path} has no {frame_key!r} array; found {data.files}")

    joint_positions = np.asarray(data[frame_key], dtype=np.float64)
    if joint_positions.ndim != 3 or joint_positions.shape[1:] != (len(HUMAN_JOINT_NAMES), 3):
        raise ValueError(
            f"{frame_key} must have shape (frames, {len(HUMAN_JOINT_NAMES)}, 3), got {joint_positions.shape}"
        )

    joint_names = tuple(str(name) for name in data["joint_names"]) if "joint_names" in data.files else HUMAN_JOINT_NAMES
    if joint_names != HUMAN_JOINT_NAMES:
        raise ValueError(f"joint_names do not match expected IMU handoff order: {joint_names}")

    fps = float(np.asarray(data["fps"]).reshape(-1)[0]) if "fps" in data.files else 30.0
    if "timestamps_s" in data.files:
        timestamps = np.asarray(data["timestamps_s"], dtype=np.float64).reshape(-1)
    else:
        step = 1.0 / fps if fps > 0.0 else 1.0 / 30.0
        timestamps = np.arange(joint_positions.shape[0], dtype=np.float64) * step
    if timestamps.shape != (joint_positions.shape[0],):
        raise ValueError(f"timestamps_s must have shape ({joint_positions.shape[0]},), got {timestamps.shape}")

    root_quat_wxyz = None
    if "root_quat_wxyz" in data.files:
        root_quat_wxyz = np.asarray(data["root_quat_wxyz"], dtype=np.float64)
        if root_quat_wxyz.shape != (joint_positions.shape[0], 4):
            raise ValueError(f"root_quat_wxyz must have shape ({joint_positions.shape[0]}, 4), got {root_quat_wxyz.shape}")

    return HumanJointClip(
        joint_positions=joint_positions,
        root_quat_wxyz=root_quat_wxyz,
        timestamps_s=timestamps,
        fps=fps,
        joint_names=joint_names,
        frame_key=frame_key,
    )


def human_joint_clip_to_qpos_qvel(
    clip: HumanJointClip,
    *,
    base_height: float = BASE_HEIGHT_M,
    yaw_mode: YawMode = "keep",
) -> tuple[np.ndarray, np.ndarray]:
    """Convert a recorded IMU handoff clip to ``(qpos, qvel)`` arrays."""

    qpos_frames: list[np.ndarray] = []
    qvel_frames: list[np.ndarray] = []
    differencer = QvelFiniteDifferencer()

    for frame_index, points in enumerate(clip.joint_positions):
        pelvis_orientation = (
            None
            if clip.root_quat_wxyz is None
            else _rotation_from_wxyz(clip.root_quat_wxyz[frame_index])
        )
        qpos = joint_positions_to_qpos(
            points,
            pelvis_orientation,
            base_height=base_height,
            yaw_mode=yaw_mode,
        )
        qvel = differencer.update(qpos, float(clip.timestamps_s[frame_index]))
        qpos_frames.append(qpos)
        qvel_frames.append(qvel)

    return np.stack(qpos_frames), np.stack(qvel_frames)


def qpos_to_qvel(previous_qpos: np.ndarray, current_qpos: np.ndarray, dt_s: float) -> np.ndarray:
    """Finite-difference two qpos frames into ch_robot's 16-D qvel layout."""

    if dt_s <= 0.0:
        raise ValueError("dt_s must be greater than zero")

    prev = np.asarray(previous_qpos, dtype=np.float64)
    curr = np.asarray(current_qpos, dtype=np.float64)
    if prev.shape != (QPOS_WIDTH,) or curr.shape != (QPOS_WIDTH,):
        raise ValueError(
            f"qpos frames must both have shape ({QPOS_WIDTH},), got {prev.shape} and {curr.shape}"
        )

    qvel = np.zeros(QVEL_WIDTH, dtype=np.float64)
    qvel[:3] = (curr[:3] - prev[:3]) / dt_s
    qvel[3:6] = (_rotation_from_wxyz(curr[3:7]) * _rotation_from_wxyz(prev[3:7]).inv()).as_rotvec() / dt_s
    qvel[FLOATING_BASE_QVEL_DIMS:] = (curr[FLOATING_BASE_QPOS_DIMS:] - prev[FLOATING_BASE_QPOS_DIMS:]) / dt_s
    assert qvel.shape == (QVEL_WIDTH,)
    return qvel


def twist_about_axis(rot: Rotation, axis: Vector3) -> float:
    """Return the signed swing-twist angle about ``axis`` for ``rot``."""

    unit_axis = _unit(axis)
    quat_xyzw = rot.as_quat()
    projected_vector = unit_axis * float(np.dot(quat_xyzw[:3], unit_axis))
    twist_quat = np.array(
        [projected_vector[0], projected_vector[1], projected_vector[2], quat_xyzw[3]],
        dtype=np.float64,
    )
    twist_norm = float(np.linalg.norm(twist_quat))
    if twist_norm == 0.0:
        return 0.0

    twist_rot = Rotation.from_quat(twist_quat / twist_norm)
    return float(np.dot(twist_rot.as_rotvec(), unit_axis))


def to_robot_frame(rot: Rotation) -> Rotation:
    """Express a human-pipeline rotation in the ch_robot body frame."""

    return HUMAN_TO_ROBOT_FRAME * rot * HUMAN_TO_ROBOT_FRAME.inv()


def strip_robot_yaw(rot: Rotation) -> Rotation:
    """Return ``rot`` with the intrinsic Z/yaw component set to zero."""

    roll, pitch, _yaw = rot.as_euler("XYZ")
    return Rotation.from_euler("XYZ", [roll, pitch, 0.0])


def _leg_joint_values(side: Side, legpose: LegPoseLike) -> tuple[float, float, float, float, float]:
    hip_robot = to_robot_frame(legpose.hip)
    hip1, hip2, thigh = hip_robot.as_euler("XYZ")
    knee = 0.0 if legpose.knee is None else twist_about_axis(to_robot_frame(legpose.knee), X_AXIS)
    ankle = 0.0 if legpose.ankle is None else twist_about_axis(to_robot_frame(legpose.ankle), X_AXIS)
    values = np.array([hip1, hip2, thigh, knee, ankle], dtype=np.float64)
    values *= SIDE_JOINT_SIGNS[side]
    return tuple(float(value) for value in values)


def _base_orientation_to_robot_frame(rot: Rotation, *, yaw_mode: YawMode) -> Rotation:
    robot_rot = to_robot_frame(rot)
    if yaw_mode == "keep":
        return robot_rot
    if yaw_mode == "strip":
        return strip_robot_yaw(robot_rot)
    raise ValueError(f"unknown yaw_mode: {yaw_mode!r}")


def _require_sides(joint_rotations: Mapping[Side, LegPoseLike]) -> None:
    missing = [side for side in ("left", "right") if side not in joint_rotations]
    if missing:
        raise KeyError(f"joint_rotations missing side(s): {', '.join(missing)}")


def _wxyz_quat(rot: Rotation) -> np.ndarray:
    quat_xyzw = rot.as_quat()
    quat_wxyz = np.array(
        [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]],
        dtype=np.float64,
    )
    quat_wxyz[np.abs(quat_wxyz) < 1e-15] = 0.0
    return quat_wxyz


def _rotation_from_wxyz(quat_wxyz: np.ndarray) -> Rotation:
    qw, qx, qy, qz = np.asarray(quat_wxyz, dtype=np.float64)
    return Rotation.from_quat([qx, qy, qz, qw])


def _pelvis_frame_from_points(points: np.ndarray) -> Rotation:
    y_axis = _unit(points[LEFT_HIP_IDX] - points[RIGHT_HIP_IDX])
    spine_vector = points[SPINE1_IDX] - 0.5 * (points[LEFT_HIP_IDX] + points[RIGHT_HIP_IDX])
    spine_axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    if np.linalg.norm(spine_vector) >= 1e-8:
        spine_axis = _unit(spine_vector)
    z_axis = _orthogonal_component(spine_axis, y_axis)
    if np.linalg.norm(z_axis) < 1e-8:
        z_axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    else:
        z_axis = _unit(z_axis)
    x_axis = _unit(np.cross(y_axis, z_axis))
    y_axis = _unit(np.cross(z_axis, x_axis))
    return Rotation.from_matrix(np.column_stack([x_axis, y_axis, z_axis]))


def _limb_segment_frame(proximal: np.ndarray, distal: np.ndarray, lateral_axis: np.ndarray) -> Rotation:
    z_axis = _unit(np.asarray(proximal, dtype=np.float64) - np.asarray(distal, dtype=np.float64))
    y_axis = _orthogonal_component(lateral_axis, z_axis)
    if np.linalg.norm(y_axis) < 1e-8:
        y_axis = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    y_axis = _unit(y_axis)
    x_axis = _unit(np.cross(y_axis, z_axis))
    y_axis = _unit(np.cross(z_axis, x_axis))
    return Rotation.from_matrix(np.column_stack([x_axis, y_axis, z_axis]))


def _foot_segment_frame(ankle: np.ndarray, toe: np.ndarray, lateral_axis: np.ndarray) -> Rotation:
    x_axis = _unit(np.asarray(toe, dtype=np.float64) - np.asarray(ankle, dtype=np.float64))
    y_axis = _orthogonal_component(lateral_axis, x_axis)
    if np.linalg.norm(y_axis) < 1e-8:
        y_axis = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    y_axis = _unit(y_axis)
    z_axis = _unit(np.cross(x_axis, y_axis))
    y_axis = _unit(np.cross(z_axis, x_axis))
    return Rotation.from_matrix(np.column_stack([x_axis, y_axis, z_axis]))


def _orthogonal_component(vector: np.ndarray, normal: np.ndarray) -> np.ndarray:
    unit_normal = _unit(normal)
    return np.asarray(vector, dtype=np.float64) - unit_normal * float(np.dot(vector, unit_normal))


def _unit(vector: Vector3) -> Vector3:
    axis = np.asarray(vector, dtype=np.float64)
    norm = float(np.linalg.norm(axis))
    if norm == 0.0:
        raise ValueError("axis vector must be nonzero")
    return axis / norm


def _assert_contract_alignment() -> None:
    """Best-effort check against repo-root ``retargeting.ch_robot_contract``."""

    try:
        from retargeting.ch_robot_contract import CH_ROBOT_JOINT_NAMES as CONTRACT_JOINT_NAMES
    except Exception:
        return
    if CH_ROBOT_JOINT_NAMES != tuple(CONTRACT_JOINT_NAMES):
        raise RuntimeError("local ch_robot joint order does not match retargeting contract")


_assert_contract_alignment()
