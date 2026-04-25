import pytest
from scipy.spatial.transform import Rotation

from sensor.filtering import (
    PacketRejectReason,
    QuaternionFilterConfig,
    QuaternionPacketFilter,
    rotation_distance_degrees,
    xyzw_to_wxyz,
)
from sensor.packet import QuaternionPacket, SegmentId


def packet(
    *,
    segment_id: SegmentId = SegmentId.LEFT_THIGH,
    sequence: int,
    rotation: Rotation,
    receive_time_s: float,
    sensor_id: int = 1,
) -> QuaternionPacket:
    return QuaternionPacket(
        sensor_id=sensor_id,
        segment_id=segment_id,
        sequence=sequence,
        sensor_time_us=sequence * 10_000,
        quat_wxyz=xyzw_to_wxyz(rotation),
        accuracy=1.0,
        status=0,
        report_type=1,
        receive_time_s=receive_time_s,
    )


def test_filter_accepts_first_valid_packet() -> None:
    filt = QuaternionPacketFilter()
    rotation = Rotation.identity()

    result = filt.update(packet(sequence=1, rotation=rotation, receive_time_s=0.0))

    assert result.accepted
    assert result.reason is None
    assert result.rotation is not None
    assert rotation_distance_degrees(result.rotation, rotation) == pytest.approx(0.0)


def test_filter_rejects_bad_quaternion_norm() -> None:
    filt = QuaternionPacketFilter()
    bad_packet = QuaternionPacket(
        sensor_id=1,
        segment_id=SegmentId.LEFT_THIGH,
        sequence=1,
        sensor_time_us=0,
        quat_wxyz=(2.0, 0.0, 0.0, 0.0),
        accuracy=1.0,
        status=0,
        report_type=1,
        receive_time_s=0.0,
    )

    result = filt.update(bad_packet)

    assert not result.accepted
    assert result.reason == PacketRejectReason.BAD_NORM


def test_filter_rejects_large_angular_spike_and_keeps_previous_state() -> None:
    filt = QuaternionPacketFilter(
        QuaternionFilterConfig(
            smoothing_alpha=1.0,
            max_angular_speed_deg_s=360.0,
            max_single_jump_deg=30.0,
        )
    )
    first = Rotation.identity()
    spike = Rotation.from_euler("z", 120.0, degrees=True)

    first_result = filt.update(packet(sequence=1, rotation=first, receive_time_s=0.0))
    spike_result = filt.update(packet(sequence=2, rotation=spike, receive_time_s=0.01))

    assert first_result.accepted
    assert not spike_result.accepted
    assert spike_result.reason == PacketRejectReason.ANGULAR_SPIKE
    assert spike_result.rotation is not None
    assert rotation_distance_degrees(spike_result.rotation, first) == pytest.approx(0.0)


def test_filter_smooths_accepted_orientation_with_slerp() -> None:
    filt = QuaternionPacketFilter(
        QuaternionFilterConfig(
            smoothing_alpha=0.5,
            max_angular_speed_deg_s=10_000.0,
            max_single_jump_deg=180.0,
        )
    )

    filt.update(packet(sequence=1, rotation=Rotation.identity(), receive_time_s=0.0))
    result = filt.update(
        packet(
            sequence=2,
            rotation=Rotation.from_euler("z", 20.0, degrees=True),
            receive_time_s=0.02,
        )
    )

    assert result.accepted
    assert result.rotation is not None
    expected = Rotation.from_euler("z", 10.0, degrees=True)
    assert rotation_distance_degrees(result.rotation, expected) == pytest.approx(0.0, abs=1e-6)


def test_filter_rejects_stale_sequence() -> None:
    filt = QuaternionPacketFilter()

    filt.update(packet(sequence=5, rotation=Rotation.identity(), receive_time_s=0.0))
    result = filt.update(packet(sequence=4, rotation=Rotation.identity(), receive_time_s=0.01))

    assert not result.accepted
    assert result.reason == PacketRejectReason.STALE_SEQUENCE
