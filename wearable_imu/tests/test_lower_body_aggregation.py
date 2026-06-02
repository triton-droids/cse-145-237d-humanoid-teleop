import numpy as np
from scipy.spatial.transform import Rotation

from ik.lower_body_aggregation import aggregate_lower_body_skeleton
from model.lower_body import LowerBodyDimensions, build_lower_body_model
from sensor.packet import SegmentId


def test_missing_distal_segment_orientation_uses_neutral_estimate() -> None:
    orientations = {
        SegmentId.PELVIS: Rotation.identity(),
        SegmentId.LEFT_THIGH: Rotation.identity(),
        SegmentId.LEFT_SHANK: Rotation.identity(),
        SegmentId.LEFT_FOOT: Rotation.identity(),
        SegmentId.RIGHT_THIGH: Rotation.identity(),
        SegmentId.RIGHT_SHANK: Rotation.identity(),
    }

    skeleton = aggregate_lower_body_skeleton(orientations)

    assert SegmentId.RIGHT_FOOT not in skeleton.available_segments
    np.testing.assert_allclose(
        skeleton.segment_orientations[SegmentId.RIGHT_FOOT].as_matrix(),
        orientations[SegmentId.RIGHT_SHANK].as_matrix(),
    )
    assert skeleton.joint_rotations["right"].ankle is None


def test_identity_orientations_produce_symmetric_standing_skeleton() -> None:
    dims = LowerBodyDimensions()
    orientations = _identity_segment_orientations()

    skeleton = aggregate_lower_body_skeleton(orientations, dimensions=dims)

    np.testing.assert_allclose(skeleton.joints["pelvis"], [0.0, 0.0, 1.0])
    np.testing.assert_allclose(
        skeleton.joints["left_hip"] - skeleton.joints["right_hip"],
        [0.0, dims.hip_spacing, 0.0],
    )
    np.testing.assert_allclose(
        skeleton.joints["left_knee"] - skeleton.joints["left_hip"],
        [0.0, 0.0, -dims.thigh_length],
    )
    np.testing.assert_allclose(
        skeleton.joints["right_ankle"] - skeleton.joints["right_knee"],
        [0.0, 0.0, -dims.shank_length],
    )
    np.testing.assert_allclose(
        skeleton.joints["left_toe"] - skeleton.joints["left_ankle"],
        [dims.foot_toe_length, 0.0, 0.0],
    )
    np.testing.assert_allclose(
        skeleton.joints["right_heel"] - skeleton.joints["right_ankle"],
        [-dims.foot_heel_length, 0.0, 0.0],
    )


def test_aggregated_joint_rotations_match_segment_orientation_chain() -> None:
    pelvis = Rotation.from_euler("xyz", [1.0, -2.0, 4.0], degrees=True)
    left_hip = Rotation.from_euler("xyz", [5.0, 16.0, -3.0], degrees=True)
    left_knee = Rotation.from_euler("xyz", [0.0, 47.0, 1.0], degrees=True)
    left_ankle = Rotation.from_euler("xyz", [1.0, -9.0, 5.0], degrees=True)
    right_hip = Rotation.from_euler("xyz", [2.0, 11.0, 2.0], degrees=True)
    right_knee = Rotation.from_euler("xyz", [0.0, 31.0, -1.0], degrees=True)
    right_ankle = Rotation.from_euler("xyz", [-1.0, -7.0, -3.0], degrees=True)

    orientations = {
        SegmentId.PELVIS: pelvis,
        SegmentId.LEFT_THIGH: pelvis * left_hip,
        SegmentId.LEFT_SHANK: pelvis * left_hip * left_knee,
        SegmentId.LEFT_FOOT: pelvis * left_hip * left_knee * left_ankle,
        SegmentId.RIGHT_THIGH: pelvis * right_hip,
        SegmentId.RIGHT_SHANK: pelvis * right_hip * right_knee,
        SegmentId.RIGHT_FOOT: pelvis * right_hip * right_knee * right_ankle,
    }

    skeleton = aggregate_lower_body_skeleton(orientations)

    np.testing.assert_allclose(
        skeleton.joint_rotations["left"].hip.as_matrix(),
        left_hip.as_matrix(),
        atol=1e-12,
    )
    np.testing.assert_allclose(
        skeleton.joint_rotations["left"].knee.as_matrix(),
        left_knee.as_matrix(),
        atol=1e-12,
    )
    np.testing.assert_allclose(
        skeleton.joint_rotations["left"].ankle.as_matrix(),
        left_ankle.as_matrix(),
        atol=1e-12,
    )
    np.testing.assert_allclose(
        skeleton.joint_rotations["right"].hip.as_matrix(),
        right_hip.as_matrix(),
        atol=1e-12,
    )
    np.testing.assert_allclose(
        skeleton.joint_rotations["right"].knee.as_matrix(),
        right_knee.as_matrix(),
        atol=1e-12,
    )
    np.testing.assert_allclose(
        skeleton.joint_rotations["right"].ankle.as_matrix(),
        right_ankle.as_matrix(),
        atol=1e-12,
    )


def test_aggregation_reconstructs_existing_lower_body_model_segments() -> None:
    model = build_lower_body_model()
    orientations = {
        SegmentId.PELVIS: Rotation.identity(),
        SegmentId.LEFT_THIGH: model.segments["left_thigh"].rotation,
        SegmentId.LEFT_SHANK: model.segments["left_shank"].rotation,
        SegmentId.LEFT_FOOT: model.segments["left_foot"].rotation,
        SegmentId.RIGHT_THIGH: model.segments["right_thigh"].rotation,
        SegmentId.RIGHT_SHANK: model.segments["right_shank"].rotation,
        SegmentId.RIGHT_FOOT: model.segments["right_foot"].rotation,
    }

    skeleton = aggregate_lower_body_skeleton(orientations)

    for side in ("left", "right"):
        np.testing.assert_allclose(
            skeleton.hip_centers[side],
            model.hip_centers[side],
            atol=1e-12,
        )
        for segment_name in (f"{side}_thigh", f"{side}_shank", f"{side}_foot"):
            np.testing.assert_allclose(
                skeleton.segments[segment_name].origin,
                model.segments[segment_name].origin,
                atol=1e-12,
            )
            np.testing.assert_allclose(
                skeleton.segments[segment_name].distal_point,
                model.segments[segment_name].distal_point,
                atol=1e-12,
            )


def _identity_segment_orientations() -> dict[SegmentId, Rotation]:
    return {
        SegmentId.PELVIS: Rotation.identity(),
        SegmentId.LEFT_THIGH: Rotation.identity(),
        SegmentId.LEFT_SHANK: Rotation.identity(),
        SegmentId.LEFT_FOOT: Rotation.identity(),
        SegmentId.RIGHT_THIGH: Rotation.identity(),
        SegmentId.RIGHT_SHANK: Rotation.identity(),
        SegmentId.RIGHT_FOOT: Rotation.identity(),
    }
