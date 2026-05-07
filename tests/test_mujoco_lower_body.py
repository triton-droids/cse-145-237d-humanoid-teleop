import mujoco
import numpy as np
import pytest

from model.lower_body import build_lower_body_model
from simulator.mujoco_lower_body import (
    build_lower_body_mjcf,
    clamp_ctrl_to_actuator_ranges,
    clamp_qpos_to_joint_ranges,
    default_qpos,
    lower_limb_solutions_from_imus,
    pose_presets,
    pose_overlay_columns,
    site_names,
    virtual_imu_orientations_from_mujoco,
)


def test_generated_mujoco_model_loads_and_matches_imu_positions() -> None:
    model = mujoco.MjModel.from_xml_string(build_lower_body_mjcf())
    data = mujoco.MjData(model)

    data.qpos[:] = default_qpos()
    data.ctrl[:] = default_qpos()
    mujoco.mj_forward(model, data)

    assert model.nq == 14
    assert model.nu == 14
    assert model.nsite == 7

    rigid_model = build_lower_body_model()
    for site_name in site_names():
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        assert site_id >= 0
        expected_name = site_name.replace("_imu", "")
        np.testing.assert_allclose(
            data.site_xpos[site_id],
            rigid_model.imu_mounts[expected_name].position,
            atol=1e-2,
        )


def test_pose_presets_match_model_dimensions_and_limits() -> None:
    model = mujoco.MjModel.from_xml_string(build_lower_body_mjcf())
    presets = pose_presets()

    assert {"standing", "squat", "step"} <= set(presets)
    for qpos in presets.values():
        assert qpos.shape == (model.nq,)
        np.testing.assert_allclose(qpos, clamp_qpos_to_joint_ranges(model, qpos))
        np.testing.assert_allclose(qpos, clamp_ctrl_to_actuator_ranges(model, qpos))


def test_virtual_imu_orientations_drive_lower_limb_solver() -> None:
    model = mujoco.MjModel.from_xml_string(build_lower_body_mjcf())
    data = mujoco.MjData(model)

    data.qpos[:] = pose_presets()["step"]
    data.ctrl[:] = data.qpos
    mujoco.mj_forward(model, data)

    virtual_imus = virtual_imu_orientations_from_mujoco(model, data)
    left_solution, right_solution = lower_limb_solutions_from_imus(virtual_imus)
    left_xyz = left_solution.joints.euler_xyz_degrees()
    right_xyz = right_solution.joints.euler_xyz_degrees()

    assert set(virtual_imus) == {
        "pelvis",
        "left_thigh",
        "left_shank",
        "left_foot",
        "right_thigh",
        "right_shank",
        "right_foot",
    }
    assert left_xyz["hip"] is not None
    assert right_xyz["hip"] is not None
    assert left_xyz["knee"][1] == pytest.approx(np.degrees(data.qpos[3]))
    assert right_xyz["knee"][1] == pytest.approx(np.degrees(data.qpos[10]))


def test_pose_overlay_contains_pelvis_joint_estimates_and_controls() -> None:
    left_text, right_text = pose_overlay_columns(
        default_qpos(),
        preset_name="step",
        animate=False,
    )

    assert "Pelvis IMU quat wxyz" in left_text
    assert "Estimated sagittal angles (deg)" in left_text
    assert "preset: step" in left_text
    assert "Controls" in right_text
    assert "1 standing" in right_text
    assert "q/a hip" in right_text
