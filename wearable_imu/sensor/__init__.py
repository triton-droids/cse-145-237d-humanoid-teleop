"""Sensor packet contracts for the wearable IMU pipeline."""

from .packet import (
    PACKET_MAGIC,
    PACKET_VERSION,
    QUATERNION_PACKET_SIZE,
    QuaternionOrder,
    QuaternionPacket,
    SegmentId,
    pack_quaternion_packet,
    parse_quaternion_packet,
)
from .filtering import (
    PacketRejectReason,
    QuaternionFilterConfig,
    QuaternionPacketFilter,
)

__all__ = [
    "PACKET_MAGIC",
    "PACKET_VERSION",
    "QUATERNION_PACKET_SIZE",
    "QuaternionOrder",
    "QuaternionPacket",
    "SegmentId",
    "pack_quaternion_packet",
    "parse_quaternion_packet",
    "PacketRejectReason",
    "QuaternionFilterConfig",
    "QuaternionPacketFilter",
]
