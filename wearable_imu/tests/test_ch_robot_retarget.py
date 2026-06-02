from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from ik.ch_robot_retarget import (
    BASE_HEIGHT_M,
    CH_ROBOT_JOINT_NAMES,
    FLOATING_BASE_QPOS_DIMS,
    HUMAN_TO_ROBOT_FRAME,
    HUMAN_JOINT_NAMES,
    JOINT_LIMIT_HIGH,
    JOINT_LIMIT_LOW,
    LEFT_ANKLE_IDX,
    LEFT_TOE_IDX,
    QPOS_WIDTH,
    QVEL_WIDTH,
    RIGHT_ANKLE_IDX,
    RIGHT_TOE_IDX,
    QvelFiniteDifferencer,
    HumanJointClip,
    base_position_from_joint_points,
    human_joint_clip_to_qpos_qvel,
    joint_positions_to_legposes,
    joint_positions_to_qpos,
    legposes_to_qpos,
    load_human_joint_clip,
    load_human_joint_source,
    load_smplh_camera_jsonl,
    qpos_to_qvel,
    strip_robot_yaw,
    to_robot_frame,
    twist_about_axis,
)
from ik.zmq_human_joint_stream import (
    HUMAN_JOINT_TOPIC,
    decode_human_joint_frame,
    encode_human_joint_frame,
)
from model.lower_body import LegPose
from demos.demo_mujoco_ch_robot_replay import (
    FLOOR_MATERIAL_NAME,
    FLOOR_TEXTURE_NAME,
    ensure_visible_floor,
)


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


def test_identity_legposes_produce_zero_joints_and_default_base() -> None:
    qpos = legposes_to_qpos(_identity_joint_rotations())

    assert qpos.shape == (QPOS_WIDTH,)
    assert qpos.dtype == np.float64
    np.testing.assert_allclose(qpos[:7], [0.0, 0.0, BASE_HEIGHT_M, 1.0, 0.0, 0.0, 0.0])
    np.testing.assert_allclose(qpos[FLOATING_BASE_QPOS_DIMS:], np.zeros(len(CH_ROBOT_JOINT_NAMES)))


def test_known_robot_frame_joint_rotations_map_to_contract_order() -> None:
    left = LegPose(
        hip=_from_robot_frame(Rotation.from_euler("XYZ", [0.20, -0.30, 0.40])),
        knee=_from_robot_frame(Rotation.from_euler("x", 0.55)),
        ankle=_from_robot_frame(Rotation.from_euler("x", -0.12)),
    )
    right = LegPose(
        hip=_from_robot_frame(Rotation.from_euler("XYZ", [-0.10, 0.15, -0.25])),
        knee=_from_robot_frame(Rotation.from_euler("x", -0.35)),
        ankle=_from_robot_frame(Rotation.from_euler("x", 0.22)),
    )

    qpos = legposes_to_qpos({"left": left, "right": right})

    np.testing.assert_allclose(
        qpos[FLOATING_BASE_QPOS_DIMS:],
        [-0.10, 0.15, -0.25, -0.35, 0.22, 0.20, -0.30, 0.40, 0.55, -0.12],
        atol=1e-12,
    )


def test_human_sides_drive_matching_display_sides_not_xml_bank_names() -> None:
    left = LegPose(
        hip=Rotation.identity(),
        knee=_from_robot_frame(Rotation.from_euler("x", 0.25)),
        ankle=Rotation.identity(),
    )
    right = LegPose(
        hip=Rotation.identity(),
        knee=_from_robot_frame(Rotation.from_euler("x", 0.75)),
        ankle=Rotation.identity(),
    )

    qpos = legposes_to_qpos({"left": left, "right": right})

    assert qpos[10] == pytest.approx(0.75)
    assert qpos[15] == pytest.approx(0.25)


def test_missing_knee_and_ankle_are_neutral() -> None:
    qpos = legposes_to_qpos(
        {
            "left": LegPose(hip=Rotation.identity(), knee=None, ankle=None),
            "right": LegPose(hip=Rotation.identity(), knee=None, ankle=None),
        }
    )

    assert qpos[10] == pytest.approx(0.0)
    assert qpos[11] == pytest.approx(0.0)
    assert qpos[15] == pytest.approx(0.0)
    assert qpos[16] == pytest.approx(0.0)


def test_joint_values_are_clipped_to_ch_robot_limits() -> None:
    extreme = LegPose(
        hip=_from_robot_frame(Rotation.from_euler("XYZ", [2.0, 0.0, 2.0])),
        knee=_from_robot_frame(Rotation.from_euler("x", 2.0)),
        ankle=_from_robot_frame(Rotation.from_euler("x", -2.0)),
    )

    qpos = legposes_to_qpos({"left": extreme, "right": extreme})
    joints = qpos[FLOATING_BASE_QPOS_DIMS:]

    assert np.all(joints <= JOINT_LIMIT_HIGH)
    assert np.all(joints >= JOINT_LIMIT_LOW)
    np.testing.assert_allclose(
        joints,
        [1.57, 0.0, 1.57, 1.57, -1.57, 1.57, 0.0, 1.57, 1.57, -1.57],
    )


def test_twist_about_axis_extracts_only_requested_axis() -> None:
    assert twist_about_axis(Rotation.from_euler("x", 0.42), np.array([1.0, 0.0, 0.0])) == pytest.approx(0.42)
    assert twist_about_axis(Rotation.from_euler("y", 0.42), np.array([1.0, 0.0, 0.0])) == pytest.approx(0.0)


def test_strip_yaw_keeps_robot_frame_roll_pitch_only() -> None:
    pelvis_human = _from_robot_frame(Rotation.from_euler("XYZ", [0.12, -0.18, 0.75]))

    keep = legposes_to_qpos(_identity_joint_rotations(), pelvis_human, yaw_mode="keep")
    strip = legposes_to_qpos(_identity_joint_rotations(), pelvis_human, yaw_mode="strip")

    keep_robot = _rotation_from_wxyz(keep[3:7])
    strip_robot = _rotation_from_wxyz(strip[3:7])

    np.testing.assert_allclose(keep_robot.as_euler("XYZ"), [0.12, -0.18, 0.75], atol=1e-12)
    np.testing.assert_allclose(strip_robot.as_euler("XYZ"), [0.12, -0.18, 0.0], atol=1e-12)
    np.testing.assert_allclose(strip_robot.as_matrix(), strip_robot_yaw_reference(keep_robot).as_matrix())


def test_qpos_to_qvel_returns_16d_finite_difference() -> None:
    previous = legposes_to_qpos(_identity_joint_rotations())
    current = previous.copy()
    current[0:3] = [0.01, -0.02, BASE_HEIGHT_M + 0.03]
    current[3:7] = [np.cos(0.10), 0.0, 0.0, np.sin(0.10)]
    current[FLOATING_BASE_QPOS_DIMS:] = np.linspace(-0.2, 0.2, len(CH_ROBOT_JOINT_NAMES))

    qvel = qpos_to_qvel(previous, current, 0.5)

    assert qvel.shape == (QVEL_WIDTH,)
    np.testing.assert_allclose(qvel[:3], [0.02, -0.04, 0.06])
    np.testing.assert_allclose(qvel[3:6], [0.0, 0.0, 0.4], atol=1e-12)
    np.testing.assert_allclose(qvel[6:], current[FLOATING_BASE_QPOS_DIMS:] / 0.5)


def test_qvel_finite_differencer_first_sample_is_zero_then_differences() -> None:
    differencer = QvelFiniteDifferencer()
    previous = legposes_to_qpos(_identity_joint_rotations())
    current = previous.copy()
    current[7] = 0.3

    np.testing.assert_allclose(differencer.update(previous, 10.0), np.zeros(QVEL_WIDTH))
    qvel = differencer.update(current, 10.1)

    assert qvel[6] == pytest.approx(3.0)


def test_neutral_joint_positions_reconstruct_zero_qpos() -> None:
    points = _neutral_joint_positions()

    legposes = joint_positions_to_legposes(points, Rotation.identity())
    qpos = joint_positions_to_qpos(points, Rotation.identity())

    for legpose in legposes.values():
        np.testing.assert_allclose(legpose.hip.as_rotvec(), [0.0, 0.0, 0.0], atol=1e-12)
        np.testing.assert_allclose(legpose.knee.as_rotvec(), [0.0, 0.0, 0.0], atol=1e-12)
        np.testing.assert_allclose(legpose.ankle.as_rotvec(), [0.0, 0.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(qpos, legposes_to_qpos(_identity_joint_rotations()))


def test_backward_generated_toe_points_do_not_flip_foot_frame() -> None:
    points = _neutral_joint_positions()
    points[LEFT_TOE_IDX] = points[LEFT_ANKLE_IDX] + np.array([-0.18, 0.0, 0.0])
    points[RIGHT_TOE_IDX] = points[RIGHT_ANKLE_IDX] + np.array([-0.18, 0.0, 0.0])

    legposes = joint_positions_to_legposes(points, Rotation.identity())
    qpos = joint_positions_to_qpos(points, Rotation.identity())

    for legpose in legposes.values():
        np.testing.assert_allclose(legpose.ankle.as_rotvec(), [0.0, 0.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(qpos, legposes_to_qpos(_identity_joint_rotations()))


def test_human_joint_clip_to_qpos_qvel_converts_recorded_imu_handoff_shape() -> None:
    points = np.stack([_neutral_joint_positions(), _neutral_joint_positions()])
    points[1, 2, 0] = 0.03
    clip = HumanJointClip(
        joint_positions=points,
        root_quat_wxyz=np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (2, 1)),
        timestamps_s=np.array([0.0, 0.02]),
        fps=50.0,
        joint_names=HUMAN_JOINT_NAMES,
        frame_key="joint_pos_origin",
    )

    qpos, qvel = human_joint_clip_to_qpos_qvel(clip)

    assert qpos.shape == (2, QPOS_WIDTH)
    assert qvel.shape == (2, QVEL_WIDTH)
    np.testing.assert_allclose(qvel[0], np.zeros(QVEL_WIDTH))
    assert np.any(np.abs(qpos[1, FLOATING_BASE_QPOS_DIMS:]) > 0.0)


def test_root_xy_base_motion_drives_freejoint_translation_and_velocity() -> None:
    points = np.stack([_neutral_joint_positions(), _neutral_joint_positions()])
    points[1] += np.array([0.12, -0.04, 0.03], dtype=np.float64)
    clip = HumanJointClip(
        joint_positions=points,
        root_quat_wxyz=None,
        timestamps_s=np.array([0.0, 0.02]),
        fps=50.0,
        joint_names=HUMAN_JOINT_NAMES,
        frame_key="joint_pos_origin",
    )

    qpos, qvel = human_joint_clip_to_qpos_qvel(clip, base_motion="root_xy")

    np.testing.assert_allclose(qpos[0, :3], [0.0, 0.0, BASE_HEIGHT_M])
    np.testing.assert_allclose(qpos[1, :3], [0.04, 0.12, BASE_HEIGHT_M])
    np.testing.assert_allclose(qvel[0, :3], [0.0, 0.0, 0.0])
    np.testing.assert_allclose(qvel[1, :3], [2.0, 6.0, 0.0])
    np.testing.assert_allclose(
        base_position_from_joint_points(
            points[1],
            root_origin=points[0, 0],
            base_motion="root_xyz",
        ),
        [0.04, 0.12, BASE_HEIGHT_M + 0.03],
    )


def test_root_xy_forward_base_motion_keeps_forward_displacement_positive() -> None:
    points = np.stack([_neutral_joint_positions(), _neutral_joint_positions()])
    points[1] += np.array([-0.12, -0.04, 0.03], dtype=np.float64)
    clip = HumanJointClip(
        joint_positions=points,
        root_quat_wxyz=None,
        timestamps_s=np.array([0.0, 0.02]),
        fps=50.0,
        joint_names=HUMAN_JOINT_NAMES,
        frame_key="joint_pos_origin",
    )

    qpos, qvel = human_joint_clip_to_qpos_qvel(clip, base_motion="root_xy_forward")

    np.testing.assert_allclose(qpos[1, :3], [0.04, 0.12, BASE_HEIGHT_M])
    np.testing.assert_allclose(qvel[1, :3], [2.0, 6.0, 0.0])
    np.testing.assert_allclose(
        base_position_from_joint_points(
            points[1],
            root_origin=points[0, 0],
            base_motion="root_xy_forward",
        ),
        [0.04, 0.12, BASE_HEIGHT_M],
    )


def test_load_actual_recorded_clip_if_available() -> None:
    path = REPO_ROOT / "data" / "human_joint_clip_20260601_231345.npz"
    if not path.exists():
        pytest.skip(f"recorded IMU handoff clip not available: {path}")

    clip = load_human_joint_clip(path)
    qpos, qvel = human_joint_clip_to_qpos_qvel(clip)

    assert clip.joint_names == HUMAN_JOINT_NAMES
    assert qpos.shape == (clip.joint_positions.shape[0], QPOS_WIDTH)
    assert qvel.shape == (clip.joint_positions.shape[0], QVEL_WIDTH)
    assert np.all(np.isfinite(qpos))
    assert np.all(np.isfinite(qvel))


def test_load_actual_smplh_camera_jsonl_if_available() -> None:
    path = REPO_ROOT / "data" / "smplh_capture_3.jsonl"
    if not path.exists():
        pytest.skip(f"camera JSONL clip not available: {path}")

    clip = load_smplh_camera_jsonl(path)
    qpos, qvel = human_joint_clip_to_qpos_qvel(clip)
    generic = load_human_joint_source(path)

    assert clip.joint_names == HUMAN_JOINT_NAMES
    assert clip.joint_positions.shape[0] == 149
    assert clip.joint_positions.shape[1:] == (len(HUMAN_JOINT_NAMES), 3)
    assert clip.fps == pytest.approx(8.08, rel=0.05)
    assert generic.frame_key == "smplh_camera_jsonl"
    assert qpos.shape == (clip.joint_positions.shape[0], QPOS_WIDTH)
    assert qvel.shape == (clip.joint_positions.shape[0], QVEL_WIDTH)
    assert np.all(np.isfinite(qpos))
    assert np.all(np.isfinite(qvel))


def test_zmq_human_joint_frame_roundtrip() -> None:
    points = _neutral_joint_positions()

    parts = encode_human_joint_frame(
        points,
        frame_index=3,
        clip_time_s=0.06,
        timestamp_s=123.0,
        root_quat_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
        fps=50.0,
        frame_key="joint_pos_origin",
    )
    header, decoded = decode_human_joint_frame(parts)

    assert parts[0] == HUMAN_JOINT_TOPIC
    assert header["frame_index"] == 3
    assert header["timestamp_s"] == pytest.approx(123.0)
    assert header["fps"] == pytest.approx(50.0)
    assert header["root_quat_wxyz"] == [1.0, 0.0, 0.0, 0.0]
    np.testing.assert_allclose(decoded, points)


def test_ensure_visible_floor_adds_checker_plane(tmp_path: Path) -> None:
    xml_path = tmp_path / "model.xml"
    xml_path.write_text(
        """
        <mujoco model="tiny">
          <asset>
            <mesh name="left_foot_sole_visual" file="visual/left_foot_sole.stl"/>
            <mesh name="right_foot_sole_visual" file="visual/right_foot_sole.stl"/>
          </asset>
          <worldbody>
            <body name="left_foot">
              <geom class="visual" mesh="left_foot_sole_visual" pos="0.30958 0 0"/>
            </body>
            <body name="right_foot">
              <geom class="visual" mesh="right_foot_sole_visual" pos="-0.310851 0 0"/>
            </body>
          </worldbody>
        </mujoco>
        """,
        encoding="utf-8",
    )

    ensure_visible_floor(xml_path)
    root = ET.parse(xml_path).getroot()
    texture = root.find(f".//texture[@name='{FLOOR_TEXTURE_NAME}']")
    material = root.find(f".//material[@name='{FLOOR_MATERIAL_NAME}']")
    ground = root.find(".//geom[@name='ground']")

    assert texture is not None
    assert texture.attrib["builtin"] == "checker"
    assert material is not None
    assert material.attrib["texture"] == FLOOR_TEXTURE_NAME
    assert ground is not None
    assert ground.attrib["type"] == "plane"
    assert ground.attrib["material"] == FLOOR_MATERIAL_NAME

    left_sole = root.find(".//body[@name='left_foot']/geom[@mesh='right_foot_sole_visual']")
    right_sole = root.find(".//body[@name='right_foot']/geom[@mesh='left_foot_sole_visual']")
    assert left_sole is not None
    assert "pos" not in left_sole.attrib
    assert right_sole is not None
    assert "pos" not in right_sole.attrib


def test_retargeted_qpos_matches_repo_root_contract_validator() -> None:
    from retargeting.ch_robot_contract import (
        CH_ROBOT_JOINT_NAMES as CONTRACT_JOINT_NAMES,
        validate_retargeted_motion,
    )

    qpos = legposes_to_qpos(_identity_joint_rotations())
    errors = validate_retargeted_motion(
        {
            "qpos": qpos.reshape(1, -1),
            "fps": np.array([50.0]),
            "human_joints": np.zeros((1, 9, 3), dtype=np.float64),
            "cost": np.zeros(1, dtype=np.float64),
        }
    )

    assert CH_ROBOT_JOINT_NAMES == CONTRACT_JOINT_NAMES
    assert errors == []


def test_holosoma_branch_mjcf_contract_matches_live_retargeter() -> None:
    xml = _git_show(
        "origin/retargeting_holosoma:"
        "holosoma-main/src/holosoma_retargeting/holosoma_retargeting/models/ch_robot/ch_robot_10dof.xml"
    )
    root = ET.fromstring(xml)
    joints = [
        (
            joint.attrib["name"],
            tuple(float(value) for value in joint.attrib["axis"].split()),
            tuple(float(value) for value in joint.attrib["range"].split()),
        )
        for joint in root.findall(".//joint")
        if joint.attrib.get("type") == "hinge"
    ]
    torso = root.find('.//body[@name="torso"]')

    assert torso is not None
    assert tuple(float(value) for value in torso.attrib["pos"].split()) == pytest.approx((0.0, 0.0, BASE_HEIGHT_M))
    assert tuple(name for name, _axis, _range in joints) == CH_ROBOT_JOINT_NAMES
    assert tuple(axis for _name, axis, _range in joints) == (
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
        (1.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
        (1.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
    )
    assert all(joint_range == pytest.approx((JOINT_LIMIT_LOW, JOINT_LIMIT_HIGH)) for _name, _axis, joint_range in joints)


def test_to_robot_frame_uses_expected_basis_change() -> None:
    assert np.linalg.det(HUMAN_TO_ROBOT_FRAME) == pytest.approx(1.0)
    np.testing.assert_allclose(HUMAN_TO_ROBOT_FRAME @ [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0])
    np.testing.assert_allclose(HUMAN_TO_ROBOT_FRAME @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])

    human_forward_yaw = Rotation.from_euler("z", np.pi / 2.0)
    robot_rot = to_robot_frame(human_forward_yaw)

    np.testing.assert_allclose(robot_rot.as_rotvec(), [0.0, 0.0, np.pi / 2.0], atol=1e-12)


def strip_robot_yaw_reference(rot: Rotation) -> Rotation:
    return strip_robot_yaw(rot)


def _identity_joint_rotations() -> dict[str, LegPose]:
    identity = Rotation.identity()
    return {
        "left": LegPose(hip=identity, knee=identity, ankle=identity),
        "right": LegPose(hip=identity, knee=identity, ankle=identity),
    }


def _from_robot_frame(rot: Rotation) -> Rotation:
    return Rotation.from_matrix(HUMAN_TO_ROBOT_FRAME.T @ rot.as_matrix() @ HUMAN_TO_ROBOT_FRAME)


def _rotation_from_wxyz(quat_wxyz: np.ndarray) -> Rotation:
    qw, qx, qy, qz = quat_wxyz
    return Rotation.from_quat([qx, qy, qz, qw])


def _neutral_joint_positions() -> np.ndarray:
    return np.array(
        [
            [0.00, 0.00, 0.89],
            [0.00, 0.11, 0.89],
            [0.00, 0.11, 0.39],
            [0.00, 0.11, 0.00],
            [0.18, 0.11, 0.00],
            [0.00, -0.11, 0.89],
            [0.00, -0.11, 0.39],
            [0.00, -0.11, 0.00],
            [0.18, -0.11, 0.00],
        ],
        dtype=np.float64,
    )


def _git_show(revision_path: str) -> str:
    try:
        return subprocess.check_output(
            ["git", "show", revision_path],
            cwd=REPO_ROOT,
            text=True,
            stderr=subprocess.DEVNULL,
        )
    except subprocess.CalledProcessError:
        pytest.skip(f"git revision not available: {revision_path}")
