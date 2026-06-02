"""Publish a recorded IMU handoff clip as mock-live human joints over ZeroMQ.

Default stream:
  endpoint: tcp://127.0.0.1:5556
  topic   : human_joint_frame
  rate    : recorded fps, or --fps if provided

Run this in one terminal, then run a ZMQ subscriber such as
``demo_mujoco_ch_robot_zmq.py`` in another.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

PROJECT_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PROJECT_ROOT.parent
for path in (PROJECT_ROOT, REPO_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from ik.ch_robot_retarget import load_human_joint_clip  # noqa: E402
from ik.zmq_human_joint_stream import (  # noqa: E402
    DEFAULT_HUMAN_JOINT_ENDPOINT,
    HUMAN_JOINT_TOPIC,
    encode_human_joint_frame,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("clip", type=Path, help="Recorded human_joint_clip_*.npz path.")
    parser.add_argument("--endpoint", default=DEFAULT_HUMAN_JOINT_ENDPOINT)
    parser.add_argument("--frame-key", choices=("joint_pos_origin", "joint_pos_w"), default="joint_pos_origin")
    parser.add_argument("--fps", type=float, default=0.0, help="Publish rate. 0 uses clip fps.")
    parser.add_argument("--loop", action="store_true", default=True)
    parser.add_argument("--no-loop", dest="loop", action="store_false")
    parser.add_argument("--start-delay-s", type=float, default=0.5, help="Delay after bind so subscribers can connect.")
    parser.add_argument("--max-frames", type=int, default=0, help="Stop after N sent frames; 0 sends full clip/loops forever.")
    parser.add_argument("--status-every", type=int, default=50)
    return parser.parse_args()


def main() -> None:
    try:
        import zmq
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError("pyzmq is required. Install with: python -m pip install pyzmq") from exc

    args = parse_args()
    clip = load_human_joint_clip(args.clip, frame_key=args.frame_key)
    fps = clip.fps if args.fps <= 0.0 else args.fps
    if fps <= 0.0:
        raise ValueError("publish fps must be greater than zero")
    frame_period_s = 1.0 / fps

    context = zmq.Context.instance()
    socket = context.socket(zmq.PUB)
    socket.bind(args.endpoint)

    print("Mock-live human joint publisher")
    print(f"  clip      : {args.clip}")
    print(f"  endpoint  : {args.endpoint}")
    print(f"  topic     : {HUMAN_JOINT_TOPIC.decode()}")
    print(f"  frame key : {args.frame_key}")
    print(f"  frames    : {clip.joint_positions.shape[0]}")
    print(f"  fps       : {fps:.1f}")
    print(f"  loop      : {args.loop}")
    time.sleep(max(0.0, args.start_delay_s))

    sent = 0
    frame_index = 0
    next_send_s = time.monotonic()
    try:
        while True:
            now = time.monotonic()
            if now < next_send_s:
                time.sleep(min(next_send_s - now, 0.002))
                continue

            local_index = frame_index % clip.joint_positions.shape[0]
            root_quat = None if clip.root_quat_wxyz is None else clip.root_quat_wxyz[local_index]
            msg = encode_human_joint_frame(
                clip.joint_positions[local_index],
                frame_index=frame_index,
                clip_time_s=float(clip.timestamps_s[local_index]),
                timestamp_s=time.time(),
                root_quat_wxyz=root_quat,
                fps=fps,
                frame_key=args.frame_key,
                joint_names=clip.joint_names,
            )
            socket.send_multipart(msg)

            sent += 1
            frame_index += 1
            next_send_s += frame_period_s
            if args.status_every > 0 and sent % args.status_every == 0:
                print(f"  sent {sent} frames")
            if args.max_frames > 0 and sent >= args.max_frames:
                break
            if not args.loop and frame_index >= clip.joint_positions.shape[0]:
                break
    except KeyboardInterrupt:
        print("Stopping publisher.")
    finally:
        socket.close(linger=0)


if __name__ == "__main__":
    main()
