"""Binary quaternion packet format shared by ESP32 firmware and Jetson code."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import struct
import time


PACKET_MAGIC = b"IMUQ"
PACKET_VERSION = 1


class SegmentId(IntEnum):
    PELVIS = 0
    LEFT_THIGH = 1
    LEFT_SHANK = 2
    LEFT_FOOT = 3
    RIGHT_THIGH = 4
    RIGHT_SHANK = 5
    RIGHT_FOOT = 6
    UNKNOWN = 255


class QuaternionOrder(IntEnum):
    WXYZ = 1


_PACKET_STRUCT = struct.Struct("<4sBBBBIIfffffBBH")
QUATERNION_PACKET_SIZE = _PACKET_STRUCT.size


@dataclass(frozen=True)
class QuaternionPacket:
    """One BNO085-like orientation packet received by the compute node."""

    sensor_id: int
    segment_id: SegmentId
    sequence: int
    sensor_time_us: int
    quat_wxyz: tuple[float, float, float, float]
    accuracy: float
    status: int
    report_type: int
    receive_time_s: float


def parse_quaternion_packet(payload: bytes, *, receive_time_s: float | None = None) -> QuaternionPacket:
    """Parse one UDP payload from an ESP32 IMU node."""

    if len(payload) != QUATERNION_PACKET_SIZE:
        raise ValueError(f"expected {QUATERNION_PACKET_SIZE} bytes, got {len(payload)}")

    (
        magic,
        version,
        sensor_id,
        segment_id_raw,
        quat_order,
        sequence,
        sensor_time_us,
        qw,
        qx,
        qy,
        qz,
        accuracy,
        status,
        report_type,
        _reserved,
    ) = _PACKET_STRUCT.unpack(payload)

    if magic != PACKET_MAGIC:
        raise ValueError(f"bad packet magic: {magic!r}")
    if version != PACKET_VERSION:
        raise ValueError(f"unsupported packet version: {version}")
    if quat_order != QuaternionOrder.WXYZ:
        raise ValueError(f"unsupported quaternion order: {quat_order}")

    try:
        segment_id = SegmentId(segment_id_raw)
    except ValueError:
        segment_id = SegmentId.UNKNOWN

    return QuaternionPacket(
        sensor_id=sensor_id,
        segment_id=segment_id,
        sequence=sequence,
        sensor_time_us=sensor_time_us,
        quat_wxyz=(qw, qx, qy, qz),
        accuracy=accuracy,
        status=status,
        report_type=report_type,
        receive_time_s=time.monotonic() if receive_time_s is None else receive_time_s,
    )


def pack_quaternion_packet(
    *,
    sensor_id: int,
    segment_id: SegmentId,
    sequence: int,
    sensor_time_us: int,
    quat_wxyz: tuple[float, float, float, float],
    accuracy: float,
    status: int = 0,
    report_type: int = 0,
) -> bytes:
    """Pack a quaternion packet for tests, tools, and simulator bridges."""

    return _PACKET_STRUCT.pack(
        PACKET_MAGIC,
        PACKET_VERSION,
        sensor_id,
        int(segment_id),
        int(QuaternionOrder.WXYZ),
        sequence,
        sensor_time_us,
        quat_wxyz[0],
        quat_wxyz[1],
        quat_wxyz[2],
        quat_wxyz[3],
        accuracy,
        status,
        report_type,
        0,
    )
