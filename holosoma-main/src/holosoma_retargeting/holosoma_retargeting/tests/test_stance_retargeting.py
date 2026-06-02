from __future__ import annotations

from pathlib import Path

import mujoco

from holosoma_retargeting.config_types.retargeting import RetargetingConfig
from holosoma_retargeting.config_types.retargeter import StanceConfig
from holosoma_retargeting.examples.robot_retarget import (
    build_contact_confidence_sequence,
    determine_output_path,
    resolve_frame_window,
)


def test_contact_confidence_ramps_and_thresholds_short_segments() -> None:
    sequence = (
        [{"L_Toe": False, "R_Toe": False}]
        + [{"L_Toe": True, "R_Toe": False} for _ in range(5)]
        + [{"L_Toe": False, "R_Toe": True} for _ in range(2)]
        + [{"L_Toe": False, "R_Toe": False}]
    )
    config = StanceConfig(enable=True, ramp_frames=2, min_contact_frames=3)

    confidence = build_contact_confidence_sequence(sequence, config)

    expected_left = [0.0, 0.5, 1.0, 1.0, 1.0, 0.5, 0.0, 0.0, 0.0]
    assert all(abs(actual["L_Toe"] - expected) < 1e-12 for actual, expected in zip(confidence, expected_left))
    assert all(frame["R_Toe"] == 0.0 for frame in confidence)


def test_ch_robot_knee_qpos_indices_match_plan() -> None:
    model_path = (
        Path(__file__).resolve().parents[1]
        / "models"
        / "ch_robot"
        / "ch_robot_10dof.xml"
    )
    model = mujoco.MjModel.from_xml_path(str(model_path))
    joint_qpos = {model.joint(i).name: int(model.jnt_qposadr[i]) for i in range(model.njnt)}

    assert joint_qpos["left_knee_joint"] == 10
    assert joint_qpos["right_knee_joint"] == 15


def test_middle_frame_window_and_output_suffix() -> None:
    cfg = RetargetingConfig(middle_frames=500)

    frame_window = resolve_frame_window(cfg, num_frames=6771)

    assert frame_window == (3135, 3635)
    assert determine_output_path(
        "robot_only",
        Path("/tmp/results"),
        "dance2_subject1",
        augmentation=False,
        frame_window=frame_window,
    ) == "/tmp/results/dance2_subject1_frames_3135_3634.npz"
