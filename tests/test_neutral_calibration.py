import pytest
from scipy.spatial.transform import Rotation

from calibration.neutral import (
    REQUIRED_LOWER_BODY_SEGMENTS,
    NeutralCalibrationAccumulator,
    apply_neutral_calibration,
)
from sensor.filtering import rotation_distance_degrees, xyzw_to_wxyz
from sensor.packet import QuaternionPacket, SegmentId


def packet_for(segment: SegmentId, sequence: int, rotation: Rotation) -> QuaternionPacket:
    return QuaternionPacket(
        sensor_id=int(segment) + 10,
        segment_id=segment,
        sequence=sequence,
        sensor_time_us=sequence * 10_000,
        quat_wxyz=xyzw_to_wxyz(rotation),
        accuracy=1.0,
        status=0,
        report_type=1,
        receive_time_s=sequence * 0.01,
    )


def test_neutral_accumulator_requires_all_segments() -> None:
    accumulator = NeutralCalibrationAccumulator(min_samples_per_segment=2)

    accumulator.add_packet(packet_for(SegmentId.PELVIS, 1, Rotation.identity()))
    accumulator.add_packet(packet_for(SegmentId.PELVIS, 2, Rotation.identity()))

    assert not accumulator.ready()
    with pytest.raises(ValueError, match="not enough neutral samples"):
        accumulator.build_profile()


def test_neutral_accumulator_builds_profile_from_synthetic_samples() -> None:
    accumulator = NeutralCalibrationAccumulator(min_samples_per_segment=3)
    neutral = {
        segment: Rotation.from_euler("z", 5.0 * index, degrees=True)
        for index, segment in enumerate(REQUIRED_LOWER_BODY_SEGMENTS)
    }

    sequence = 1
    for _ in range(3):
        for segment, rotation in neutral.items():
            accumulator.add_packet(packet_for(segment, sequence, rotation))
            sequence += 1

    assert accumulator.ready()
    profile = accumulator.build_profile()

    assert set(profile.neutral_orientations) == set(REQUIRED_LOWER_BODY_SEGMENTS)
    assert profile.sample_counts[SegmentId.LEFT_THIGH] == 3
    assert profile.sensor_ids[SegmentId.LEFT_THIGH] == int(SegmentId.LEFT_THIGH) + 10
    for segment, rotation in neutral.items():
        assert rotation_distance_degrees(profile.neutral_orientations[segment], rotation) == pytest.approx(0.0)


def test_apply_neutral_calibration_returns_relative_motion() -> None:
    accumulator = NeutralCalibrationAccumulator(
        required_segments=(SegmentId.LEFT_THIGH,),
        min_samples_per_segment=2,
    )
    neutral = Rotation.from_euler("z", 30.0, degrees=True)
    live = Rotation.from_euler("z", 45.0, degrees=True)

    accumulator.add_packet(packet_for(SegmentId.LEFT_THIGH, 1, neutral))
    accumulator.add_packet(packet_for(SegmentId.LEFT_THIGH, 2, neutral))
    profile = accumulator.build_profile()

    relative = apply_neutral_calibration(profile, {SegmentId.LEFT_THIGH: live})

    expected = Rotation.from_euler("z", 15.0, degrees=True)
    assert rotation_distance_degrees(relative[SegmentId.LEFT_THIGH], expected) == pytest.approx(0.0, abs=1e-6)
