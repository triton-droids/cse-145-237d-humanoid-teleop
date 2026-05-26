"""Quaternion filtering for incoming wearable IMU packets."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Iterable

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from .packet import QuaternionPacket, SegmentId


class PacketRejectReason(str, Enum):
    BAD_NORM = "bad_norm"
    STALE_SEQUENCE = "stale_sequence"
    ANGULAR_SPIKE = "angular_spike"
    UNKNOWN_SEGMENT = "unknown_segment"


@dataclass(frozen=True)
class FilterResult:
    packet: QuaternionPacket
    rotation: Rotation | None
    accepted: bool
    reason: PacketRejectReason | None = None


@dataclass
class SegmentFilterState:
    sequence: int
    rotation: Rotation
    receive_time_s: float


@dataclass(frozen=True)
class QuaternionFilterConfig:
    smoothing_alpha: float = 0.35
    max_angular_speed_deg_s: float = 720.0
    max_single_jump_deg: float = 45.0
    norm_tolerance: float = 0.15


class QuaternionPacketFilter:
    """Reject obvious packet spikes and smooth accepted orientations."""

    def __init__(self, config: QuaternionFilterConfig | None = None) -> None:
        self.config = config or QuaternionFilterConfig()
        self._state: dict[SegmentId, SegmentFilterState] = {}

    def update(self, packet: QuaternionPacket) -> FilterResult:
        if packet.segment_id == SegmentId.UNKNOWN:
            return FilterResult(packet=packet, rotation=None, accepted=False, reason=PacketRejectReason.UNKNOWN_SEGMENT)

        quat_xyzw = wxyz_to_xyzw(packet.quat_wxyz)
        norm = float(np.linalg.norm(quat_xyzw))
        if not np.isfinite(norm) or abs(norm - 1.0) > self.config.norm_tolerance:
            return FilterResult(packet=packet, rotation=None, accepted=False, reason=PacketRejectReason.BAD_NORM)

        rotation = Rotation.from_quat(quat_xyzw / norm)
        previous = self._state.get(packet.segment_id)
        if previous is None:
            self._state[packet.segment_id] = SegmentFilterState(
                sequence=packet.sequence,
                rotation=rotation,
                receive_time_s=packet.receive_time_s,
            )
            return FilterResult(packet=packet, rotation=rotation, accepted=True)

        if not sequence_is_newer(packet.sequence, previous.sequence):
            return FilterResult(packet=packet, rotation=previous.rotation, accepted=False, reason=PacketRejectReason.STALE_SEQUENCE)

        angle_deg = rotation_distance_degrees(previous.rotation, rotation)
        dt = max(packet.receive_time_s - previous.receive_time_s, 1e-6)
        max_allowed = max(self.config.max_single_jump_deg, self.config.max_angular_speed_deg_s * dt)
        if angle_deg > max_allowed:
            return FilterResult(packet=packet, rotation=previous.rotation, accepted=False, reason=PacketRejectReason.ANGULAR_SPIKE)

        smoothed = slerp_pair(previous.rotation, rotation, self.config.smoothing_alpha)
        self._state[packet.segment_id] = SegmentFilterState(
            sequence=packet.sequence,
            rotation=smoothed,
            receive_time_s=packet.receive_time_s,
        )
        return FilterResult(packet=packet, rotation=smoothed, accepted=True)

    def latest(self, segment_id: SegmentId) -> Rotation | None:
        state = self._state.get(segment_id)
        return None if state is None else state.rotation


def average_rotations(rotations: Iterable[Rotation]) -> Rotation:
    """Average rotations while keeping quaternion signs in the same hemisphere."""

    quats = np.array([rotation.as_quat() for rotation in rotations], dtype=float)
    if quats.size == 0:
        raise ValueError("cannot average an empty rotation collection")

    reference = quats[0]
    for index in range(1, len(quats)):
        if float(np.dot(reference, quats[index])) < 0.0:
            quats[index] *= -1.0

    mean_quat = np.mean(quats, axis=0)
    norm = float(np.linalg.norm(mean_quat))
    if norm == 0.0:
        raise ValueError("rotation average produced a zero quaternion")
    return Rotation.from_quat(mean_quat / norm)


def rotation_distance_degrees(a: Rotation, b: Rotation) -> float:
    return float((a.inv() * b).magnitude() * 180.0 / np.pi)


def slerp_pair(a: Rotation, b: Rotation, alpha: float) -> Rotation:
    alpha = float(np.clip(alpha, 0.0, 1.0))
    return Slerp([0.0, 1.0], Rotation.concatenate([a, b]))([alpha])[0]


def sequence_is_newer(candidate: int, current: int) -> bool:
    return 0 < ((candidate - current) & 0xFFFFFFFF) < 0x80000000


def wxyz_to_xyzw(quat_wxyz: tuple[float, float, float, float]) -> np.ndarray:
    qw, qx, qy, qz = quat_wxyz
    return np.array([qx, qy, qz, qw], dtype=float)


def xyzw_to_wxyz(rotation: Rotation) -> tuple[float, float, float, float]:
    qx, qy, qz, qw = rotation.as_quat()
    return (float(qw), float(qx), float(qy), float(qz))
