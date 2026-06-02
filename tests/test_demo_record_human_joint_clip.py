import numpy as np
from scipy.spatial.transform import Rotation

from demos.demo_record_human_joint_clip import (
    JOINT_NAMES,
    frame_as_dict,
    ml_joint_positions_w,
    origin_relative_points,
    pelvis_ground_origin_w,
)
from ik.lower_body_aggregation import aggregate_lower_body_skeleton
from model.lower_body import LowerBodyDimensions
from sensor.packet import SegmentId


def test_ml_joint_positions_use_requested_names_and_order() -> None:
    dims = LowerBodyDimensions()
    skeleton = aggregate_lower_body_skeleton(
        {
            SegmentId.PELVIS: Rotation.identity(),
            SegmentId.LEFT_THIGH: Rotation.identity(),
            SegmentId.LEFT_SHANK: Rotation.identity(),
            SegmentId.LEFT_FOOT: Rotation.identity(),
            SegmentId.RIGHT_THIGH: Rotation.identity(),
            SegmentId.RIGHT_SHANK: Rotation.identity(),
            SegmentId.RIGHT_FOOT: Rotation.identity(),
        },
        dimensions=dims,
    )

    points_w = ml_joint_positions_w(skeleton)

    assert JOINT_NAMES == (
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
    np.testing.assert_allclose(points_w[0], skeleton.joints["pelvis"])
    np.testing.assert_allclose(points_w[1], skeleton.joints["left_hip"])
    np.testing.assert_allclose(points_w[2], skeleton.joints["left_knee"])
    np.testing.assert_allclose(points_w[3], skeleton.joints["left_ankle"])
    np.testing.assert_allclose(points_w[4], skeleton.joints["left_toe"])
    np.testing.assert_allclose(points_w[5], skeleton.joints["right_hip"])
    np.testing.assert_allclose(points_w[6], skeleton.joints["right_knee"])
    np.testing.assert_allclose(points_w[7], skeleton.joints["right_ankle"])
    np.testing.assert_allclose(points_w[8], skeleton.joints["right_toe"])


def test_origin_relative_points_use_frame0_pelvis_floor_projection() -> None:
    frame0 = np.array(
        [
            [2.0, 3.0, 1.0],
            [2.0, 3.1, 1.0],
        ],
        dtype=float,
    )
    frame1 = np.array(
        [
            [2.2, 3.3, 1.05],
            [2.2, 3.4, 0.95],
        ],
        dtype=float,
    )

    origin_w = pelvis_ground_origin_w(frame0)

    np.testing.assert_allclose(origin_w, [2.0, 3.0, 0.0])
    np.testing.assert_allclose(
        origin_relative_points(frame1, origin_w),
        [
            [0.2, 0.3, 1.05],
            [0.2, 0.4, 0.95],
        ],
    )


def test_frame_as_dict_matches_ml_frame_shape() -> None:
    points = np.arange(27, dtype=float).reshape(9, 3)

    frame = frame_as_dict(points)

    assert list(frame) == list(JOINT_NAMES)
    assert frame["Spine1"] == [0.0, 1.0, 2.0]
    assert frame["RightToeBase"] == [24.0, 25.0, 26.0]
