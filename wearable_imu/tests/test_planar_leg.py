from math import isclose

import numpy as np

from ik.planar_leg import (
    FootTarget,
    JointAngles,
    LegSegmentLengths,
    forward_kinematics,
    solve_leg_ik,
)


def test_forward_inverse_round_trip_positive_knee_branch() -> None:
    lengths = LegSegmentLengths()
    original = JointAngles(hip=0.25, knee=0.7, ankle=-0.35)
    pose = forward_kinematics(original, lengths)
    foot = pose["foot"]

    target = FootTarget(x=float(foot[0]), z=float(foot[1]), angle=original.foot_angle)
    solved = solve_leg_ik(target, lengths, knee_direction=1)

    assert solved.reachable
    assert solved.position_error < 1e-12
    assert solved.orientation_error < 1e-12
    np.testing.assert_allclose(solved.angles.as_array(), original.as_array(), atol=1e-12)


def test_negative_knee_branch_is_available() -> None:
    lengths = LegSegmentLengths()
    original = JointAngles(hip=0.4, knee=-0.65, ankle=0.2)
    pose = forward_kinematics(original, lengths)
    foot = pose["foot"]

    target = FootTarget(x=float(foot[0]), z=float(foot[1]), angle=original.foot_angle)
    solved = solve_leg_ik(target, lengths, knee_direction=-1)

    assert solved.reachable
    np.testing.assert_allclose(solved.angles.as_array(), original.as_array(), atol=1e-12)


def test_unreachable_target_reports_error() -> None:
    lengths = LegSegmentLengths(thigh=0.45, shank=0.43, foot=0.25)
    target = FootTarget(x=0.0, z=-2.0, angle=0.0)

    solved = solve_leg_ik(target, lengths)

    assert not solved.reachable
    assert solved.position_error > 0.0
    assert isclose(solved.orientation_error, 0.0, abs_tol=1e-12)


def test_invalid_segment_lengths_are_rejected() -> None:
    try:
        LegSegmentLengths(thigh=0.0)
    except ValueError as exc:
        assert "thigh length must be positive" in str(exc)
    else:
        raise AssertionError("zero segment length should fail")
