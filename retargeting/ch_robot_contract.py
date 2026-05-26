"""Retargeting contract for the Triton humanoid / ch_robot model."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any


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

FLOATING_BASE_QPOS_DIMS = 7
FLOATING_BASE_QVEL_DIMS = 6

RETARGETED_REQUIRED_KEYS: tuple[str, ...] = ("qpos", "fps", "human_joints", "cost")
CONVERTED_REQUIRED_KEYS: tuple[str, ...] = (
    "joint_pos",
    "joint_vel",
    "body_pos_w",
    "body_quat_w",
    "body_lin_vel_w",
    "body_ang_vel_w",
    "joint_names",
    "body_names",
    "fps",
)


@dataclass(frozen=True)
class RetargetingContract:
    robot_name: str
    joint_names: tuple[str, ...]
    qpos_width: int
    qvel_width: int
    retargeted_required_keys: tuple[str, ...]
    converted_required_keys: tuple[str, ...]


CH_ROBOT_CONTRACT = RetargetingContract(
    robot_name="ch_robot",
    joint_names=CH_ROBOT_JOINT_NAMES,
    qpos_width=FLOATING_BASE_QPOS_DIMS + len(CH_ROBOT_JOINT_NAMES),
    qvel_width=FLOATING_BASE_QVEL_DIMS + len(CH_ROBOT_JOINT_NAMES),
    retargeted_required_keys=RETARGETED_REQUIRED_KEYS,
    converted_required_keys=CONVERTED_REQUIRED_KEYS,
)


def _shape(value: Any) -> tuple[int, ...]:
    shape = getattr(value, "shape", None)
    if shape is not None:
        return tuple(int(dim) for dim in shape)
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        if not value:
            return (0,)
        first = value[0]
        if isinstance(first, Sequence) and not isinstance(first, (str, bytes)):
            return (len(value), len(first))
        return (len(value),)
    return ()


def _missing_keys(data: Mapping[str, Any], required: Sequence[str]) -> list[str]:
    return [key for key in required if key not in data]


def validate_retargeted_motion(data: Mapping[str, Any]) -> list[str]:
    """Return validation errors for a compact retargeted motion mapping."""

    errors = _missing_keys(data, RETARGETED_REQUIRED_KEYS)
    if errors:
        return [f"missing key: {key}" for key in errors]

    qpos_shape = _shape(data["qpos"])
    if len(qpos_shape) != 2:
        errors.append(f"qpos must be 2D, got shape {qpos_shape}")
    elif qpos_shape[1] != CH_ROBOT_CONTRACT.qpos_width:
        errors.append(
            f"qpos width must be {CH_ROBOT_CONTRACT.qpos_width} for ch_robot, "
            f"got {qpos_shape[1]}"
        )

    human_shape = _shape(data["human_joints"])
    if len(human_shape) != 3:
        errors.append(f"human_joints must be 3D, got shape {human_shape}")
    elif human_shape[2] != 3:
        errors.append(f"human_joints last dimension must be xyz=3, got {human_shape[2]}")

    return errors


def validate_converted_motion(data: Mapping[str, Any]) -> list[str]:
    """Return validation errors for Isaac Lab / RL tracking motion data."""

    errors = _missing_keys(data, CONVERTED_REQUIRED_KEYS)
    if errors:
        return [f"missing key: {key}" for key in errors]

    joint_pos_shape = _shape(data["joint_pos"])
    joint_vel_shape = _shape(data["joint_vel"])

    if len(joint_pos_shape) != 2:
        errors.append(f"joint_pos must be 2D, got shape {joint_pos_shape}")
    elif joint_pos_shape[1] != CH_ROBOT_CONTRACT.qpos_width:
        errors.append(
            f"joint_pos width must be {CH_ROBOT_CONTRACT.qpos_width}, got {joint_pos_shape[1]}"
        )

    if len(joint_vel_shape) != 2:
        errors.append(f"joint_vel must be 2D, got shape {joint_vel_shape}")
    elif joint_vel_shape[1] != CH_ROBOT_CONTRACT.qvel_width:
        errors.append(
            f"joint_vel width must be {CH_ROBOT_CONTRACT.qvel_width}, got {joint_vel_shape[1]}"
        )

    joint_names = tuple(str(name) for name in data["joint_names"])
    if joint_names != CH_ROBOT_JOINT_NAMES:
        errors.append("joint_names do not match the ch_robot joint order")

    return errors


def describe_contract() -> dict[str, Any]:
    """Return a serializable description of the ch_robot retargeting contract."""

    return {
        "robot_name": CH_ROBOT_CONTRACT.robot_name,
        "joint_names": list(CH_ROBOT_CONTRACT.joint_names),
        "retargeted_motion": {
            "required_keys": list(RETARGETED_REQUIRED_KEYS),
            "qpos_width": CH_ROBOT_CONTRACT.qpos_width,
            "qpos_layout": "7 floating-base values + 10 joint positions",
        },
        "converted_motion": {
            "required_keys": list(CONVERTED_REQUIRED_KEYS),
            "joint_pos_width": CH_ROBOT_CONTRACT.qpos_width,
            "joint_vel_width": CH_ROBOT_CONTRACT.qvel_width,
        },
    }
