import mujoco
import numpy as np

from model.lower_body import build_lower_body_model
from simulator.mujoco_lower_body import (
    build_lower_body_mjcf,
    clamp_ctrl_to_actuator_ranges,
    clamp_qpos_to_joint_ranges,
    default_qpos,
    pose_presets,
    pose_overlay_columns,
    site_names,
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
