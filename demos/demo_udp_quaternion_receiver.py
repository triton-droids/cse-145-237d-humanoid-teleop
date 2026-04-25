"""Print live ESP32/BNO085 quaternion packet status on the compute node."""

from __future__ import annotations

import argparse
from collections import defaultdict, deque
from pathlib import Path
import sys
import time

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from sensor.packet import SegmentId  # noqa: E402
from sensor.udp_receiver import LatestPacketBuffer, receive_quaternion_packets  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--max-age-ms", type=float, default=100.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    buffer = LatestPacketBuffer()
    receive_history: dict[SegmentId, deque[float]] = defaultdict(lambda: deque(maxlen=120))
    last_print = 0.0

    print(f"Listening for IMU quaternion UDP packets on {args.host}:{args.port}")
    for packet in receive_quaternion_packets(host=args.host, port=args.port):
        buffer.update(packet)
        receive_history[packet.segment_id].append(packet.receive_time_s)

        now = time.monotonic()
        if now - last_print < 0.25:
            continue

        print("\033[2J\033[H", end="")
        print("segment        sensor  hz    age_ms  seq       quat_wxyz")
        for segment in SegmentId:
            if segment == SegmentId.UNKNOWN:
                continue
            latest = buffer.packets.get(segment)
            if latest is None:
                print(f"{segment.name.lower():<14} --      --    --      --        --")
                continue

            history = receive_history[segment]
            hz = 0.0
            if len(history) >= 2:
                elapsed = history[-1] - history[0]
                if elapsed > 0:
                    hz = (len(history) - 1) / elapsed
            age_ms = 1000.0 * (now - latest.receive_time_s)
            qw, qx, qy, qz = latest.quat_wxyz
            print(
                f"{segment.name.lower():<14} "
                f"{latest.sensor_id:<7} "
                f"{hz:5.1f} "
                f"{age_ms:7.1f} "
                f"{latest.sequence:<9} "
                f"{qw: .3f} {qx: .3f} {qy: .3f} {qz: .3f}"
            )

        complete = buffer.complete(max_age_s=args.max_age_ms / 1000.0, now_s=now)
        print(f"\ncomplete fresh frame: {complete}")
        last_print = now


if __name__ == "__main__":
    main()
