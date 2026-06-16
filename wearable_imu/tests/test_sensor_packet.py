import pytest

from sensor.packet import (
    PACKET_MAGIC,
    QUATERNION_PACKET_SIZE,
    QuaternionOrder,
    SegmentId,
    pack_quaternion_packet,
    parse_quaternion_packet,
)


def test_quaternion_packet_round_trips_binary_payload() -> None:
    payload = pack_quaternion_packet(
        sensor_id=2,
        segment_id=SegmentId.LEFT_SHANK,
        sequence=42,
        sensor_time_us=123456,
        quat_wxyz=(1.0, 0.1, 0.2, 0.3),
        accuracy=2.5,
        status=3,
        report_type=1,
    )

    assert len(payload) == QUATERNION_PACKET_SIZE

    packet = parse_quaternion_packet(payload, receive_time_s=10.0)

    assert packet.sensor_id == 2
    assert packet.segment_id == SegmentId.LEFT_SHANK
    assert packet.sequence == 42
    assert packet.sensor_time_us == 123456
    assert packet.quat_wxyz == pytest.approx((1.0, 0.1, 0.2, 0.3))
    assert packet.accuracy == pytest.approx(2.5)
    assert packet.status == 3
    assert packet.report_type == 1
    assert packet.receive_time_s == 10.0


def test_quaternion_packet_rejects_bad_magic() -> None:
    payload = bytearray(
        pack_quaternion_packet(
            sensor_id=1,
            segment_id=SegmentId.PELVIS,
            sequence=1,
            sensor_time_us=1,
            quat_wxyz=(1.0, 0.0, 0.0, 0.0),
            accuracy=0.0,
        )
    )
    payload[0:4] = b"NOPE"

    with pytest.raises(ValueError, match="bad packet magic"):
        parse_quaternion_packet(bytes(payload))


def test_quaternion_packet_rejects_unknown_quaternion_order() -> None:
    payload = bytearray(
        pack_quaternion_packet(
            sensor_id=1,
            segment_id=SegmentId.PELVIS,
            sequence=1,
            sensor_time_us=1,
            quat_wxyz=(1.0, 0.0, 0.0, 0.0),
            accuracy=0.0,
        )
    )
    payload[7] = int(QuaternionOrder.WXYZ) + 1

    with pytest.raises(ValueError, match="unsupported quaternion order"):
        parse_quaternion_packet(bytes(payload))


def test_packet_magic_and_size_match_firmware_contract() -> None:
    assert PACKET_MAGIC == b"IMUQ"
    assert QUATERNION_PACKET_SIZE == 40
