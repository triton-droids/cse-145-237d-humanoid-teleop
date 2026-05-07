"""Print live ESP32/BNO085 quaternion packet status on the compute node."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from collections import defaultdict, deque
from pathlib import Path
import sys
import time

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from sensor.packet import SegmentId  # noqa: E402
from sensor.udp_receiver import LatestPacketBuffer, receive_quaternion_packets  # noqa: E402


@dataclass
class SegmentTimingStats:
    receive_times: deque[float] = field(default_factory=lambda: deque(maxlen=120))
    sensor_times_us: deque[int] = field(default_factory=lambda: deque(maxlen=120))
    last_sequence: int | None = None
    dropped_packets: int = 0

    def update(self, *, receive_time_s: float, sensor_time_us: int, sequence: int) -> None:
        if self.last_sequence is not None:
            gap = (sequence - self.last_sequence) & 0xFFFFFFFF
            if gap > 1 and gap < 0x80000000:
                self.dropped_packets += gap - 1
        self.last_sequence = sequence
        self.receive_times.append(receive_time_s)
        self.sensor_times_us.append(sensor_time_us)

    def hz(self) -> float:
        if len(self.receive_times) < 2:
            return 0.0
        elapsed = self.receive_times[-1] - self.receive_times[0]
        return 0.0 if elapsed <= 0 else (len(self.receive_times) - 1) / elapsed

    def recv_jitter_ms(self) -> float:
        return _interval_jitter_ms(list(self.receive_times))

    def sensor_jitter_ms(self) -> float:
        sensor_seconds = [value / 1_000_000.0 for value in self.sensor_times_us]
        return _interval_jitter_ms(sensor_seconds)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--max-age-ms", type=float, default=100.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    buffer = LatestPacketBuffer()
    timing_stats: dict[SegmentId, SegmentTimingStats] = defaultdict(SegmentTimingStats)
    last_print = 0.0
    last_wait_print = 0.0

    print(f"Listening for IMU quaternion UDP packets on {args.host}:{args.port}")

    def on_timeout() -> None:
        nonlocal last_wait_print
        now = time.monotonic()
        if now - last_wait_print >= 1.0:
            print(f"listening on {args.host}:{args.port}... no UDP packets yet")
            last_wait_print = now

    for packet in receive_quaternion_packets(host=args.host, port=args.port, timeout_s=0.25, on_timeout=on_timeout):
        buffer.update(packet)
        timing_stats[packet.segment_id].update(
            receive_time_s=packet.receive_time_s,
            sensor_time_us=packet.sensor_time_us,
            sequence=packet.sequence,
        )

        now = time.monotonic()
        if now - last_print < 0.25:
            continue

        print("\033[2J\033[H", end="")
        print("segment        sensor  hz    age_ms  rj_ms sj_ms drops  seq       quat_wxyz")
        for segment in SegmentId:
            if segment == SegmentId.UNKNOWN:
                continue
            latest = buffer.packets.get(segment)
            if latest is None:
                print(f"{segment.name.lower():<14} --      --    --      --    --    --     --        --")
                continue

            stats = timing_stats[segment]
            age_ms = 1000.0 * (now - latest.receive_time_s)
            qw, qx, qy, qz = latest.quat_wxyz
            print(
                f"{segment.name.lower():<14} "
                f"{latest.sensor_id:<7} "
                f"{stats.hz():5.1f} "
                f"{age_ms:7.1f} "
                f"{stats.recv_jitter_ms():5.1f} "
                f"{stats.sensor_jitter_ms():5.1f} "
                f"{stats.dropped_packets:<6} "
                f"{latest.sequence:<9} "
                f"{qw: .3f} {qx: .3f} {qy: .3f} {qz: .3f}"
            )

        complete = buffer.complete(max_age_s=args.max_age_ms / 1000.0, now_s=now)
        print(f"\ncomplete fresh frame: {complete}")
        last_print = now


def _interval_jitter_ms(times_s: list[float]) -> float:
    if len(times_s) < 3:
        return 0.0
    intervals_ms = [
        1000.0 * (times_s[index] - times_s[index - 1])
        for index in range(1, len(times_s))
    ]
    mean_interval = sum(intervals_ms) / len(intervals_ms)
    variance = sum((interval - mean_interval) ** 2 for interval in intervals_ms) / len(intervals_ms)
    return variance**0.5


if __name__ == "__main__":
    main()
