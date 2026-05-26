"""UDP receiver helpers for ESP32 quaternion packets."""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass, field
import socket
import time
from typing import Callable

from .packet import QuaternionPacket, SegmentId, parse_quaternion_packet


@dataclass
class LatestPacketBuffer:
    """Keep the newest valid packet per body segment."""

    packets: dict[SegmentId, QuaternionPacket] = field(default_factory=dict)

    def update(self, packet: QuaternionPacket) -> None:
        current = self.packets.get(packet.segment_id)
        if current is None or _sequence_is_newer(packet.sequence, current.sequence):
            self.packets[packet.segment_id] = packet

    def complete(self, *, max_age_s: float = 0.1, now_s: float | None = None) -> bool:
        now = time.monotonic() if now_s is None else now_s
        required = (
            SegmentId.PELVIS,
            SegmentId.LEFT_THIGH,
            SegmentId.LEFT_SHANK,
            SegmentId.LEFT_FOOT,
            SegmentId.RIGHT_THIGH,
            SegmentId.RIGHT_SHANK,
            SegmentId.RIGHT_FOOT,
        )
        return all(
            segment in self.packets and now - self.packets[segment].receive_time_s <= max_age_s
            for segment in required
        )


def receive_quaternion_packets(
    *,
    host: str = "0.0.0.0",
    port: int = 5005,
    timeout_s: float | None = None,
    on_timeout: Callable[[], None] | None = None,
) -> Iterator[QuaternionPacket]:
    """Yield parsed quaternion packets from a UDP socket."""

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((host, port))
        sock.settimeout(timeout_s)
        while True:
            try:
                payload, _addr = sock.recvfrom(256)
            except socket.timeout:
                if on_timeout is not None:
                    on_timeout()
                continue
            yield parse_quaternion_packet(payload, receive_time_s=time.monotonic())


def _sequence_is_newer(candidate: int, current: int) -> bool:
    # uint32 wrap-aware comparison: candidate is newer if it is within the
    # forward half of the sequence-number space.
    return 0 < ((candidate - current) & 0xFFFFFFFF) < 0x80000000
