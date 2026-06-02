"""Publish replay live-leg frames over ZeroMQ for live retargeting demos."""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import tyro

src_root = Path(__file__).resolve().parents[2]
if str(src_root) not in sys.path:
    sys.path.insert(0, str(src_root))

from holosoma_retargeting.examples.live_leg_retarget import (  # noqa: E402
    LIVE_LEG_JOINTS,
    canonicalize_live_leg_sequence,
    encode_zmq_frame,
    load_replay_sequence,
    slice_replay_sequence,
)


@dataclass
class LiveLegPublisherConfig:
    replay_path: Path
    """Path to replay .npy/.npz data."""

    zmq_endpoint: str = "tcp://127.0.0.1:5556"
    """ZeroMQ endpoint. The publisher binds a PUB socket."""

    fps: float = 30.0
    """Publish rate."""

    replay_start: int = 0
    """First replay frame to publish."""

    replay_count: int | None = None
    """Number of replay frames to publish."""

    loop: bool = False
    """Loop replay forever."""

    canonicalize_input: bool = True
    """Canonicalize replay frames before publishing."""

    input_scale: float = 1.0
    """Additional scale applied after canonicalization."""

    lafan_replay_scale: float = 1.27 / 1.7
    """Scale applied when replay input is detected as raw LAFAN data."""


def main(cfg: LiveLegPublisherConfig) -> None:
    try:
        import zmq
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError("pyzmq is required for live_leg_publisher.py. Install package dependency pyzmq.") from exc

    sequence, detected_scale = load_replay_sequence(cfg.replay_path, lafan_scale=cfg.lafan_replay_scale)
    sequence = slice_replay_sequence(sequence, cfg.replay_start, cfg.replay_count)
    sequence = canonicalize_live_leg_sequence(
        sequence,
        enabled=cfg.canonicalize_input,
        scale=cfg.input_scale * detected_scale,
    ).astype(np.float32)

    context = zmq.Context.instance()
    socket = context.socket(zmq.PUB)
    socket.bind(cfg.zmq_endpoint)
    frame_period = 1.0 / cfg.fps if cfg.fps > 0 else 0.0
    print(f"Publishing {len(sequence)} live-leg frames on {cfg.zmq_endpoint}")
    time.sleep(0.25)

    frame_idx = 0
    try:
        while True:
            for frame in sequence:
                socket.send_multipart(
                    encode_zmq_frame(frame, frame_idx=frame_idx, timestamp=time.time(), joint_names=LIVE_LEG_JOINTS)
                )
                frame_idx += 1
                if frame_period > 0:
                    time.sleep(frame_period)
            if not cfg.loop:
                break
    except KeyboardInterrupt:
        print("Stopping publisher.")


if __name__ == "__main__":
    main(tyro.cli(LiveLegPublisherConfig))
