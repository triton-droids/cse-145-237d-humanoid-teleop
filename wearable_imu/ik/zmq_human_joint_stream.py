"""ZeroMQ wire format for mock-live human joint frames."""

from __future__ import annotations

import json
import time
from typing import Sequence

import numpy as np

from ik.ch_robot_retarget import HUMAN_JOINT_NAMES


DEFAULT_HUMAN_JOINT_ENDPOINT = "tcp://127.0.0.1:5556"
HUMAN_JOINT_TOPIC = b"human_joint_frame"


def encode_human_joint_frame(
    points: np.ndarray,
    *,
    frame_index: int,
    clip_time_s: float,
    timestamp_s: float | None = None,
    root_quat_wxyz: np.ndarray | None = None,
    fps: float,
    frame_key: str,
    joint_names: Sequence[str] = HUMAN_JOINT_NAMES,
) -> list[bytes]:
    """Encode one ``(9, 3)`` human joint frame as a ZMQ multipart message."""

    arr = np.ascontiguousarray(points, dtype=np.float64)
    if arr.shape != (len(HUMAN_JOINT_NAMES), 3):
        raise ValueError(f"points must have shape ({len(HUMAN_JOINT_NAMES)}, 3), got {arr.shape}")

    header = {
        "frame_index": int(frame_index),
        "timestamp_s": float(time.time() if timestamp_s is None else timestamp_s),
        "clip_time_s": float(clip_time_s),
        "shape": list(arr.shape),
        "dtype": str(arr.dtype),
        "joint_names": list(joint_names),
        "fps": float(fps),
        "frame_key": frame_key,
    }
    if root_quat_wxyz is not None:
        quat = np.asarray(root_quat_wxyz, dtype=np.float64).reshape(4)
        header["root_quat_wxyz"] = [float(value) for value in quat]

    return [
        HUMAN_JOINT_TOPIC,
        json.dumps(header, separators=(",", ":")).encode("utf-8"),
        arr.tobytes(),
    ]


def decode_human_joint_frame(parts: Sequence[bytes]) -> tuple[dict, np.ndarray]:
    """Decode a ZMQ multipart message from ``encode_human_joint_frame``."""

    if len(parts) == 3:
        topic, header_raw, body = parts
        if topic != HUMAN_JOINT_TOPIC:
            raise ValueError(f"unexpected topic: {topic!r}")
    elif len(parts) == 2:
        header_raw, body = parts
    else:
        raise ValueError(f"expected 2 or 3 message parts, got {len(parts)}")

    header = json.loads(header_raw.decode("utf-8"))
    dtype = np.dtype(header["dtype"])
    shape = tuple(int(value) for value in header["shape"])
    points = np.frombuffer(body, dtype=dtype).reshape(shape).astype(np.float64)
    if points.shape != (len(HUMAN_JOINT_NAMES), 3):
        raise ValueError(f"decoded points must have shape ({len(HUMAN_JOINT_NAMES)}, 3), got {points.shape}")

    joint_names = tuple(str(name) for name in header.get("joint_names", ()))
    if joint_names and joint_names != HUMAN_JOINT_NAMES:
        raise ValueError(f"unexpected joint order: {joint_names}")

    return header, points
