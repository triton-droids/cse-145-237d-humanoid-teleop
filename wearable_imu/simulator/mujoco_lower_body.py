"""Generate a simple MuJoCo lower-body model for visual inspection."""

from __future__ import annotations

from math import radians
from typing import Iterable

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from ik.imu_orientation import (
    LowerLimbOrientationSolution,
    default_lower_limb_mounts,
    front_pelvis_mount,
    imu_orientation_from_segment,
    solve_lower_limb_joints_from_imus,
)
from model.lower_body import LowerBodyDimensions, default_pose


JOINT_NAMES = (
    "left_hip_x",
    "left_hip_y",
    "left_hip_z",
    "left_knee_y",
    "left_ankle_x",
    "left_ankle_y",
    "left_ankle_z",
    "right_hip_x",
    "right_hip_y",
    "right_hip_z",
    "right_knee_y",
    "right_ankle_x",
    "right_ankle_y",
    "right_ankle_z",
)

ACTUATOR_NAMES = tuple(f"{joint}_ctrl" for joint in JOINT_NAMES)

VIRTUAL_IMU_SITE_NAMES = {
    "pelvis": "pelvis_imu",
    "left_thigh": "left_thigh_imu",
    "left_shank": "left_shank_imu",
    "left_foot": "left_foot_imu",
    "right_thigh": "right_thigh_imu",
    "right_shank": "right_shank_imu",
    "right_foot": "right_foot_imu",
}


def build_lower_body_mjcf(
    dimensions: LowerBodyDimensions = LowerBodyDimensions(),
) -> str:
    """Return a generated MJCF string for the lower-body inspection model."""

    left_mounts = default_lower_limb_mounts("left")
    right_mounts = default_lower_limb_mounts("right")
    pelvis_mount = front_pelvis_mount()

    hip_y = dimensions.hip_spacing / 2.0
    foot_center_x = 0.5 * (dimensions.foot_toe_length - dimensions.foot_heel_length)
    foot_half_length = 0.5 * dimensions.foot_total_length
    foot_half_height = 0.016
    foot_width = 0.045

    left_thigh_quat = _quat_wxyz(left_mounts["thigh"].as_quat())
    left_shank_quat = _quat_wxyz(left_mounts["shank"].as_quat())
    left_foot_quat = _quat_wxyz(left_mounts["foot"].as_quat())
    right_thigh_quat = _quat_wxyz(right_mounts["thigh"].as_quat())
    right_shank_quat = _quat_wxyz(right_mounts["shank"].as_quat())
    right_foot_quat = _quat_wxyz(right_mounts["foot"].as_quat())
    pelvis_quat = _quat_wxyz(pelvis_mount.as_quat())

    return f"""
<mujoco model="lower_body_inspector">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.01" gravity="0 0 0"/>

  <visual>
    <scale framelength="0.10" framewidth="0.005" jointlength="0.04" jointwidth="0.01"/>
    <headlight ambient="0.55 0.55 0.55" diffuse="0.45 0.45 0.45" specular="0.15 0.15 0.15"/>
    <rgba haze="0.15 0.2 0.24 1"/>
  </visual>

  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.18 0.2 0.22" rgb2="0.28 0.30 0.32" width="256" height="256"/>
    <material name="floor" texture="grid" texrepeat="6 6" reflectance="0.15"/>
  </asset>

  <default>
    <joint damping="2.5" armature="0.02"/>
    <default class="left_segment">
      <geom rgba="0.16 0.62 0.56 1"/>
    </default>
    <default class="right_segment">
      <geom rgba="0.91 0.45 0.31 1"/>
    </default>
    <default class="imu_site">
      <site type="box" size="0.028 0.018 0.006" rgba="1 0.82 0.05 1"/>
    </default>
  </default>

  <worldbody>
    <light pos="0 0 2.6" dir="0 0 -1"/>
    <geom name="floor" type="plane" pos="0 0 0" size="2 2 0.1" material="floor"/>

    <body name="pelvis" pos="0 0 1.0">
      <geom name="pelvis_geom" type="box" pos="0 0 0" size="0.06 {hip_y:.6f} 0.05" rgba="0.22 0.22 0.24 1"/>
      <site name="pelvis_imu" class="imu_site" pos="{dimensions.pelvis_front_offset:.6f} 0 {dimensions.pelvis_imu_height_offset:.6f}" quat="{pelvis_quat}"/>

      <body name="left_thigh" pos="0 {hip_y:.6f} 0">
        <joint name="left_hip_x" type="hinge" axis="1 0 0" range="-90 90"/>
        <joint name="left_hip_y" type="hinge" axis="0 1 0" range="-120 120"/>
        <joint name="left_hip_z" type="hinge" axis="0 0 1" range="-60 60"/>
        <geom name="left_thigh_geom" class="left_segment" type="capsule" fromto="0 0 0 0 0 {-dimensions.thigh_length:.6f}" size="0.038"/>
        <site name="left_thigh_imu" class="imu_site" pos="0 {dimensions.thigh_lateral_offset:.6f} {-0.5 * dimensions.thigh_length:.6f}" quat="{left_thigh_quat}"/>
        <body name="left_shank" pos="0 0 {-dimensions.thigh_length:.6f}">
          <joint name="left_knee_y" type="hinge" axis="0 1 0" range="-5 135"/>
          <geom name="left_shank_geom" class="left_segment" type="capsule" fromto="0 0 0 0 0 {-dimensions.shank_length:.6f}" size="0.032"/>
          <site name="left_shank_imu" class="imu_site" pos="0 {dimensions.shank_lateral_offset:.6f} {-0.5 * dimensions.shank_length:.6f}" quat="{left_shank_quat}"/>
          <body name="left_foot" pos="0 0 {-dimensions.shank_length:.6f}">
            <joint name="left_ankle_x" type="hinge" axis="1 0 0" range="-30 30"/>
            <joint name="left_ankle_y" type="hinge" axis="0 1 0" range="-50 50"/>
            <joint name="left_ankle_z" type="hinge" axis="0 0 1" range="-30 30"/>
            <geom name="left_foot_geom" class="left_segment" type="box" pos="{foot_center_x:.6f} 0 {-foot_half_height:.6f}" size="{foot_half_length:.6f} {foot_width:.6f} {foot_half_height:.6f}"/>
            <site name="left_foot_imu" class="imu_site" pos="{0.5 * dimensions.foot_toe_length:.6f} 0 {dimensions.foot_height:.6f}" quat="{left_foot_quat}"/>
          </body>
        </body>
      </body>

      <body name="right_thigh" pos="0 {-hip_y:.6f} 0">
        <joint name="right_hip_x" type="hinge" axis="1 0 0" range="-90 90"/>
        <joint name="right_hip_y" type="hinge" axis="0 1 0" range="-120 120"/>
        <joint name="right_hip_z" type="hinge" axis="0 0 1" range="-60 60"/>
        <geom name="right_thigh_geom" class="right_segment" type="capsule" fromto="0 0 0 0 0 {-dimensions.thigh_length:.6f}" size="0.038"/>
        <site name="right_thigh_imu" class="imu_site" pos="0 {-dimensions.thigh_lateral_offset:.6f} {-0.5 * dimensions.thigh_length:.6f}" quat="{right_thigh_quat}"/>
        <body name="right_shank" pos="0 0 {-dimensions.thigh_length:.6f}">
          <joint name="right_knee_y" type="hinge" axis="0 1 0" range="-5 135"/>
          <geom name="right_shank_geom" class="right_segment" type="capsule" fromto="0 0 0 0 0 {-dimensions.shank_length:.6f}" size="0.032"/>
          <site name="right_shank_imu" class="imu_site" pos="0 {-dimensions.shank_lateral_offset:.6f} {-0.5 * dimensions.shank_length:.6f}" quat="{right_shank_quat}"/>
          <body name="right_foot" pos="0 0 {-dimensions.shank_length:.6f}">
            <joint name="right_ankle_x" type="hinge" axis="1 0 0" range="-30 30"/>
            <joint name="right_ankle_y" type="hinge" axis="0 1 0" range="-50 50"/>
            <joint name="right_ankle_z" type="hinge" axis="0 0 1" range="-30 30"/>
            <geom name="right_foot_geom" class="right_segment" type="box" pos="{foot_center_x:.6f} 0 {-foot_half_height:.6f}" size="{foot_half_length:.6f} {foot_width:.6f} {foot_half_height:.6f}"/>
            <site name="right_foot_imu" class="imu_site" pos="{0.5 * dimensions.foot_toe_length:.6f} 0 {dimensions.foot_height:.6f}" quat="{right_foot_quat}"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <position name="left_hip_x_ctrl" joint="left_hip_x" kp="40" ctrlrange="-1.570796 1.570796" forcerange="-120 120"/>
    <position name="left_hip_y_ctrl" joint="left_hip_y" kp="55" ctrlrange="-2.094395 2.094395" forcerange="-160 160"/>
    <position name="left_hip_z_ctrl" joint="left_hip_z" kp="35" ctrlrange="-1.047198 1.047198" forcerange="-90 90"/>
    <position name="left_knee_y_ctrl" joint="left_knee_y" kp="65" ctrlrange="-0.087266 2.356194" forcerange="-180 180"/>
    <position name="left_ankle_x_ctrl" joint="left_ankle_x" kp="30" ctrlrange="-0.523599 0.523599" forcerange="-70 70"/>
    <position name="left_ankle_y_ctrl" joint="left_ankle_y" kp="40" ctrlrange="-0.872665 0.872665" forcerange="-85 85"/>
    <position name="left_ankle_z_ctrl" joint="left_ankle_z" kp="25" ctrlrange="-0.523599 0.523599" forcerange="-55 55"/>

    <position name="right_hip_x_ctrl" joint="right_hip_x" kp="40" ctrlrange="-1.570796 1.570796" forcerange="-120 120"/>
    <position name="right_hip_y_ctrl" joint="right_hip_y" kp="55" ctrlrange="-2.094395 2.094395" forcerange="-160 160"/>
    <position name="right_hip_z_ctrl" joint="right_hip_z" kp="35" ctrlrange="-1.047198 1.047198" forcerange="-90 90"/>
    <position name="right_knee_y_ctrl" joint="right_knee_y" kp="65" ctrlrange="-0.087266 2.356194" forcerange="-180 180"/>
    <position name="right_ankle_x_ctrl" joint="right_ankle_x" kp="30" ctrlrange="-0.523599 0.523599" forcerange="-70 70"/>
    <position name="right_ankle_y_ctrl" joint="right_ankle_y" kp="40" ctrlrange="-0.872665 0.872665" forcerange="-85 85"/>
    <position name="right_ankle_z_ctrl" joint="right_ankle_z" kp="25" ctrlrange="-0.523599 0.523599" forcerange="-55 55"/>
  </actuator>
</mujoco>
""".strip()


def default_qpos() -> np.ndarray:
    """Return the default articulated pose as a MuJoCo qpos vector."""

    pose = default_pose()
    return np.array(
        [
            *pose["left"].hip.as_euler("xyz"),
            pose["left"].knee.as_euler("xyz")[1],
            *pose["left"].ankle.as_euler("xyz"),
            *pose["right"].hip.as_euler("xyz"),
            pose["right"].knee.as_euler("xyz")[1],
            *pose["right"].ankle.as_euler("xyz"),
        ],
        dtype=float,
    )


def pose_presets() -> dict[str, np.ndarray]:
    """Return named preset poses for viewer inspection."""

    return {
        "standing": np.array(
            [
                radians(0.0),
                radians(2.0),
                radians(0.0),
                radians(2.0),
                radians(0.0),
                radians(-2.0),
                radians(0.0),
                radians(0.0),
                radians(2.0),
                radians(0.0),
                radians(2.0),
                radians(0.0),
                radians(-2.0),
                radians(0.0),
            ],
            dtype=float,
        ),
        "squat": np.array(
            [
                radians(0.0),
                radians(-30.0),
                radians(0.0),
                radians(70.0),
                radians(0.0),
                radians(15.0),
                radians(0.0),
                radians(0.0),
                radians(-30.0),
                radians(0.0),
                radians(70.0),
                radians(0.0),
                radians(15.0),
                radians(0.0),
            ],
            dtype=float,
        ),
        "step": default_qpos(),
    }


def preset_qpos(name: str) -> np.ndarray:
    """Return a copy of a named preset pose."""

    presets = pose_presets()
    if name not in presets:
        raise KeyError(f"unknown preset: {name}")
    return presets[name].copy()


def clamp_qpos_to_joint_ranges(model, qpos: np.ndarray) -> np.ndarray:
    """Clip a qpos vector to the MuJoCo hinge ranges."""

    clipped = qpos.copy()
    for joint_id in range(model.njnt):
        qpos_adr = model.jnt_qposadr[joint_id]
        lower, upper = model.jnt_range[joint_id]
        clipped[qpos_adr] = np.clip(clipped[qpos_adr], lower, upper)
    return clipped


def clamp_ctrl_to_actuator_ranges(model, ctrl: np.ndarray) -> np.ndarray:
    """Clip a control vector to the MuJoCo actuator ranges."""

    clipped = ctrl.copy()
    for actuator_id in range(model.nu):
        lower, upper = model.actuator_ctrlrange[actuator_id]
        clipped[actuator_id] = np.clip(clipped[actuator_id], lower, upper)
    return clipped


def site_names() -> tuple[str, ...]:
    return tuple(VIRTUAL_IMU_SITE_NAMES.values())


def segment_rotations_from_qpos(qpos: np.ndarray) -> dict[str, Rotation]:
    """Return segment world orientations from the MuJoCo qpos vector."""

    pelvis = Rotation.identity()

    left_hip = Rotation.from_euler("xyz", qpos[0:3])
    left_knee = Rotation.from_euler("y", [qpos[3]])
    left_ankle = Rotation.from_euler("xyz", qpos[4:7])

    right_hip = Rotation.from_euler("xyz", qpos[7:10])
    right_knee = Rotation.from_euler("y", [qpos[10]])
    right_ankle = Rotation.from_euler("xyz", qpos[11:14])

    left_thigh = pelvis * left_hip
    left_shank = left_thigh * left_knee
    left_foot = left_shank * left_ankle

    right_thigh = pelvis * right_hip
    right_shank = right_thigh * right_knee
    right_foot = right_shank * right_ankle

    return {
        "pelvis": pelvis,
        "left_thigh": left_thigh,
        "left_shank": left_shank,
        "left_foot": left_foot,
        "right_thigh": right_thigh,
        "right_shank": right_shank,
        "right_foot": right_foot,
    }


def imu_orientations_from_qpos(qpos: np.ndarray) -> dict[str, Rotation]:
    """Return synthetic IMU world orientations for the current articulated pose."""

    segments = segment_rotations_from_qpos(qpos)
    left_mounts = default_lower_limb_mounts("left")
    right_mounts = default_lower_limb_mounts("right")
    pelvis_mount = front_pelvis_mount()

    return {
        "pelvis": imu_orientation_from_segment(segments["pelvis"], pelvis_mount),
        "left_thigh": imu_orientation_from_segment(segments["left_thigh"], left_mounts["thigh"]),
        "left_shank": imu_orientation_from_segment(segments["left_shank"], left_mounts["shank"]),
        "left_foot": imu_orientation_from_segment(segments["left_foot"], left_mounts["foot"]),
        "right_thigh": imu_orientation_from_segment(segments["right_thigh"], right_mounts["thigh"]),
        "right_shank": imu_orientation_from_segment(segments["right_shank"], right_mounts["shank"]),
        "right_foot": imu_orientation_from_segment(segments["right_foot"], right_mounts["foot"]),
    }


def virtual_imu_orientations_from_mujoco(
    model: mujoco.MjModel,
    data: mujoco.MjData,
) -> dict[str, Rotation]:
    """Return world-from-sensor rotations from the MuJoCo virtual IMU sites."""

    imu_orientations: dict[str, Rotation] = {}
    for segment_name, site_name in VIRTUAL_IMU_SITE_NAMES.items():
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if site_id < 0:
            raise ValueError(f"missing MuJoCo IMU site: {site_name}")
        imu_orientations[segment_name] = Rotation.from_matrix(
            np.asarray(data.site_xmat[site_id]).reshape(3, 3)
        )
    return imu_orientations


def lower_body_points_from_qpos(
    qpos: np.ndarray,
    dimensions: LowerBodyDimensions = LowerBodyDimensions(),
) -> dict[str, np.ndarray]:
    """Return key lower-body points for plotting the current pose."""

    segments = segment_rotations_from_qpos(qpos)
    pelvis_center = np.array([0.0, 0.0, 1.0], dtype=float)
    left_hip = pelvis_center + np.array([0.0, dimensions.hip_spacing / 2.0, 0.0], dtype=float)
    right_hip = pelvis_center + np.array([0.0, -dimensions.hip_spacing / 2.0, 0.0], dtype=float)

    left_knee = left_hip + segments["left_thigh"].apply([0.0, 0.0, -dimensions.thigh_length])
    right_knee = right_hip + segments["right_thigh"].apply([0.0, 0.0, -dimensions.thigh_length])

    left_ankle = left_knee + segments["left_shank"].apply([0.0, 0.0, -dimensions.shank_length])
    right_ankle = right_knee + segments["right_shank"].apply([0.0, 0.0, -dimensions.shank_length])

    left_heel = left_ankle + segments["left_foot"].apply([-dimensions.foot_heel_length, 0.0, 0.0])
    left_toe = left_ankle + segments["left_foot"].apply([dimensions.foot_toe_length, 0.0, 0.0])
    right_heel = right_ankle + segments["right_foot"].apply([-dimensions.foot_heel_length, 0.0, 0.0])
    right_toe = right_ankle + segments["right_foot"].apply([dimensions.foot_toe_length, 0.0, 0.0])

    return {
        "pelvis": np.vstack([left_hip, right_hip]),
        "left_leg": np.vstack([left_hip, left_knee, left_ankle]),
        "right_leg": np.vstack([right_hip, right_knee, right_ankle]),
        "left_foot": np.vstack([left_heel, left_ankle, left_toe]),
        "right_foot": np.vstack([right_heel, right_ankle, right_toe]),
    }


def lower_limb_solutions_from_imus(
    imu: dict[str, Rotation],
) -> tuple[LowerLimbOrientationSolution, LowerLimbOrientationSolution]:
    """Solve left and right lower-limb joints from virtual or real IMU rotations."""

    left_solution = solve_lower_limb_joints_from_imus(
        {
            "pelvis": imu["pelvis"],
            "thigh": imu["left_thigh"],
            "shank": imu["left_shank"],
            "foot": imu["left_foot"],
        },
        side="left",
        mounts={"pelvis": front_pelvis_mount()},
    )
    right_solution = solve_lower_limb_joints_from_imus(
        {
            "pelvis": imu["pelvis"],
            "thigh": imu["right_thigh"],
            "shank": imu["right_shank"],
            "foot": imu["right_foot"],
        },
        side="right",
        mounts={"pelvis": front_pelvis_mount()},
    )
    return left_solution, right_solution


def format_pose_readout_from_imus(
    imu: dict[str, Rotation],
    *,
    preset_name: str,
    animate: bool,
) -> str:
    """Return a compact live readout from virtual or real IMU orientations."""

    left_solution, right_solution = lower_limb_solutions_from_imus(imu)

    pelvis_xyzw = imu["pelvis"].as_quat()
    pelvis_wxyz = (pelvis_xyzw[3], pelvis_xyzw[0], pelvis_xyzw[1], pelvis_xyzw[2])
    left_xyz = left_solution.joints.euler_xyz_degrees()
    right_xyz = right_solution.joints.euler_xyz_degrees()

    return "\n".join(
        [
            "MuJoCo lower-body live readout",
            f"preset: {preset_name} | animate: {animate}",
            (
                "pelvis imu quat wxyz: "
                f"{pelvis_wxyz[0]:6.3f} {pelvis_wxyz[1]:6.3f} {pelvis_wxyz[2]:6.3f} {pelvis_wxyz[3]:6.3f}"
            ),
            (
                "left  hip/knee/ankle y deg: "
                f"{left_xyz['hip'][1]:6.1f} {left_xyz['knee'][1]:6.1f} {left_xyz['ankle'][1]:6.1f}"
            ),
            (
                "right hip/knee/ankle y deg: "
                f"{right_xyz['hip'][1]:6.1f} {right_xyz['knee'][1]:6.1f} {right_xyz['ankle'][1]:6.1f}"
            ),
            "keys: 1 standing | 2 squat | 3 step | space animate",
            "      q/a w/s e/d left | u/j i/k o/l right",
        ]
    )


def format_pose_readout(qpos: np.ndarray, *, preset_name: str, animate: bool) -> str:
    """Return a compact live readout for the current viewer pose."""

    return format_pose_readout_from_imus(
        imu_orientations_from_qpos(qpos),
        preset_name=preset_name,
        animate=animate,
    )


def pose_overlay_columns_from_imus(
    imu: dict[str, Rotation],
    *,
    preset_name: str,
    animate: bool,
) -> tuple[str, str]:
    """Return left/right overlay text from virtual or real IMU orientations."""

    left_solution, right_solution = lower_limb_solutions_from_imus(imu)

    pelvis_xyzw = imu["pelvis"].as_quat()
    pelvis_wxyz = (pelvis_xyzw[3], pelvis_xyzw[0], pelvis_xyzw[1], pelvis_xyzw[2])
    left_xyz = left_solution.joints.euler_xyz_degrees()
    right_xyz = right_solution.joints.euler_xyz_degrees()

    left_column = "\n".join(
        [
            "Lower-body IMU inspector",
            f"preset: {preset_name}",
            f"animate: {animate}",
            "",
            "Pelvis IMU quat wxyz",
            f"{pelvis_wxyz[0]:6.3f} {pelvis_wxyz[1]:6.3f} {pelvis_wxyz[2]:6.3f} {pelvis_wxyz[3]:6.3f}",
            "",
            "Estimated sagittal angles (deg)",
            f"L hip   {left_xyz['hip'][1]:6.1f}",
            f"L knee  {left_xyz['knee'][1]:6.1f}",
            f"L ankle {left_xyz['ankle'][1]:6.1f}",
            f"R hip   {right_xyz['hip'][1]:6.1f}",
            f"R knee  {right_xyz['knee'][1]:6.1f}",
            f"R ankle {right_xyz['ankle'][1]:6.1f}",
        ]
    )

    right_column = "\n".join(
        [
            "Controls",
            "1 standing",
            "2 squat",
            "3 step",
            "Space animate",
            "",
            "Left leg",
            "q/a hip",
            "w/s knee",
            "e/d ankle",
            "",
            "Right leg",
            "u/j hip",
            "i/k knee",
            "o/l ankle",
        ]
    )
    return left_column, right_column


def pose_overlay_columns(
    qpos: np.ndarray,
    *,
    preset_name: str,
    animate: bool,
) -> tuple[str, str]:
    """Return left/right overlay text columns for the MuJoCo viewer."""

    return pose_overlay_columns_from_imus(
        imu_orientations_from_qpos(qpos),
        preset_name=preset_name,
        animate=animate,
    )


def _quat_wxyz(xyzw: Iterable[float]) -> str:
    x, y, z, w = xyzw
    return f"{w:.8f} {x:.8f} {y:.8f} {z:.8f}"
