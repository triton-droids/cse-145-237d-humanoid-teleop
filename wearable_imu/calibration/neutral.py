"""Neutral-pose calibration for quaternion IMU streams."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

from scipy.spatial.transform import Rotation

from sensor.filtering import average_rotations, wxyz_to_xyzw
from sensor.packet import QuaternionPacket, SegmentId


REQUIRED_LOWER_BODY_SEGMENTS = (
    SegmentId.PELVIS,
    SegmentId.LEFT_THIGH,
    SegmentId.LEFT_SHANK,
    SegmentId.LEFT_FOOT,
    SegmentId.RIGHT_THIGH,
    SegmentId.RIGHT_SHANK,
    SegmentId.RIGHT_FOOT,
)


@dataclass(frozen=True)
class CalibrationProfile:
    """Session calibration captured from a neutral standing pose."""

    neutral_orientations: dict[SegmentId, Rotation]
    sensor_ids: dict[SegmentId, int]
    sample_counts: dict[SegmentId, int]

    def relative_orientation(self, segment_id: SegmentId, orientation: Rotation) -> Rotation:
        """Express a live sensor orientation relative to the neutral pose."""

        return self.neutral_orientations[segment_id].inv() * orientation


class NeutralCalibrationAccumulator:
    """Collect packets during still standing and build a neutral profile."""

    def __init__(
        self,
        *,
        required_segments: tuple[SegmentId, ...] = REQUIRED_LOWER_BODY_SEGMENTS,
        min_samples_per_segment: int = 20,
    ) -> None:
        self.required_segments = required_segments
        self.min_samples_per_segment = min_samples_per_segment
        self._samples: dict[SegmentId, list[Rotation]] = {segment: [] for segment in required_segments}
        self._sensor_ids: dict[SegmentId, int] = {}

    def add_packet(self, packet: QuaternionPacket) -> None:
        if packet.segment_id not in self._samples:
            return

        rotation = Rotation.from_quat(_normalized_xyzw(packet.quat_wxyz))
        self._samples[packet.segment_id].append(rotation)
        self._sensor_ids.setdefault(packet.segment_id, packet.sensor_id)

    def ready(self) -> bool:
        return all(
            len(self._samples[segment]) >= self.min_samples_per_segment
            for segment in self.required_segments
        )

    def sample_counts(self) -> dict[SegmentId, int]:
        return {segment: len(samples) for segment, samples in self._samples.items()}

    def build_profile(self) -> CalibrationProfile:
        missing = [
            segment.name
            for segment in self.required_segments
            if len(self._samples[segment]) < self.min_samples_per_segment
        ]
        if missing:
            raise ValueError(f"not enough neutral samples for: {', '.join(missing)}")

        return CalibrationProfile(
            neutral_orientations={
                segment: average_rotations(self._samples[segment])
                for segment in self.required_segments
            },
            sensor_ids={segment: self._sensor_ids[segment] for segment in self.required_segments},
            sample_counts=self.sample_counts(),
        )


def apply_neutral_calibration(
    profile: CalibrationProfile,
    orientations: Mapping[SegmentId, Rotation],
) -> dict[SegmentId, Rotation]:
    """Return live orientations expressed relative to the neutral calibration."""

    return {
        segment: profile.relative_orientation(segment, orientation)
        for segment, orientation in orientations.items()
        if segment in profile.neutral_orientations
    }


def _normalized_xyzw(quat_wxyz: tuple[float, float, float, float]):
    quat = wxyz_to_xyzw(quat_wxyz)
    norm = float((quat @ quat) ** 0.5)
    if norm == 0.0:
        raise ValueError("cannot normalize zero quaternion")
    return quat / norm
