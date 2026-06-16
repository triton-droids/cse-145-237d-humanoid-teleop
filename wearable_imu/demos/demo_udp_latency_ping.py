"""Measure UDP round-trip latency to one ESP32 node."""

# Usage (from project root):
#   conda run --no-capture-output -p .\.conda python demos\demo_udp_latency_ping.py <ESP32_IP> --port 5006

from __future__ import annotations

import argparse
import socket
import statistics
import time


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("esp32_ip", help="ESP32 IP address printed by Serial Monitor")
    parser.add_argument("--port", type=int, default=5006, help="ESP32 UDP local ping port")
    parser.add_argument("--count", type=int, default=100)
    parser.add_argument("--interval-ms", type=float, default=50.0)
    parser.add_argument("--timeout-ms", type=float, default=500.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rtts_ms: list[float] = []
    lost = 0

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.settimeout(args.timeout_ms / 1000.0)
        target = (args.esp32_ip, args.port)
        for sequence in range(args.count):
            send_ns = time.monotonic_ns()
            payload = f"PING,{sequence},{send_ns}".encode("ascii")
            sock.sendto(payload, target)

            try:
                reply, addr = sock.recvfrom(128)
            except socket.timeout:
                lost += 1
                print(f"{sequence:04d} timeout")
                time.sleep(args.interval_ms / 1000.0)
                continue

            receive_ns = time.monotonic_ns()
            if reply != payload:
                lost += 1
                print(f"{sequence:04d} bad reply from {addr}: {reply!r}")
                time.sleep(args.interval_ms / 1000.0)
                continue

            rtt_ms = (receive_ns - send_ns) / 1_000_000.0
            rtts_ms.append(rtt_ms)
            print(f"{sequence:04d} rtt_ms={rtt_ms:7.2f} from={addr[0]}:{addr[1]}")
            time.sleep(args.interval_ms / 1000.0)

    if not rtts_ms:
        print("\nNo successful ping replies.")
        return

    print("\nUDP round-trip latency summary")
    print(f"sent: {args.count}")
    print(f"received: {len(rtts_ms)}")
    print(f"lost: {lost}")
    print(f"min_ms: {min(rtts_ms):.2f}")
    print(f"mean_ms: {statistics.mean(rtts_ms):.2f}")
    print(f"median_ms: {statistics.median(rtts_ms):.2f}")
    print(f"max_ms: {max(rtts_ms):.2f}")


if __name__ == "__main__":
    main()
