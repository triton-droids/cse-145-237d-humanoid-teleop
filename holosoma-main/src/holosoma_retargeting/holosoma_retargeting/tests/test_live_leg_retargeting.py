from __future__ import annotations

import numpy as np

from holosoma_retargeting.config_types.data_type import LAFAN_DEMO_JOINTS
from holosoma_retargeting.examples.live_leg_retarget import (
    LIVE_LEG_JOINTS,
    LiveInputCanonicalizer,
    decode_zmq_frame,
    encode_zmq_frame,
    extract_live_leg_sequence_from_lafan,
    reorder_frame_by_joint_names,
)


def test_zmq_frame_round_trip() -> None:
    frame = np.arange(27, dtype=np.float32).reshape(9, 3)

    frame_idx, timestamp, decoded = decode_zmq_frame(encode_zmq_frame(frame, frame_idx=12, timestamp=34.5))

    assert frame_idx == 12
    assert timestamp == 34.5
    assert decoded.shape == (9, 3)
    assert np.allclose(decoded, frame)


def test_named_joint_reordering() -> None:
    shuffled_names = list(reversed(LIVE_LEG_JOINTS))
    shuffled_frame = np.asarray([[i, i + 0.1, i + 0.2] for i in range(9)], dtype=np.float32)

    reordered = reorder_frame_by_joint_names(shuffled_frame, shuffled_names)

    expected = np.asarray([shuffled_frame[shuffled_names.index(name)] for name in LIVE_LEG_JOINTS])
    assert np.allclose(reordered, expected)


def test_decode_reorders_optional_joint_names() -> None:
    shuffled_names = list(reversed(LIVE_LEG_JOINTS))
    shuffled_frame = np.asarray([[i, i + 1, i + 2] for i in range(9)], dtype=np.float32)

    _, _, decoded = decode_zmq_frame(
        encode_zmq_frame(shuffled_frame, frame_idx=0, timestamp=0.0, joint_names=shuffled_names)
    )

    assert np.allclose(decoded[0], shuffled_frame[shuffled_names.index("Spine1")])


def test_live_input_canonicalizer_keeps_grounded_height() -> None:
    frame = np.zeros((9, 3), dtype=float)
    frame[0] = [10.0, -2.0, 1.0]  # Spine1
    frame[4] = [9.5, -2.1, 0.2]  # LeftToeBase
    frame[8] = [10.5, -1.9, 0.3]  # RightToeBase

    canonical = LiveInputCanonicalizer(enabled=True, scale=1.0).apply(frame)

    assert np.allclose(canonical[0, :2], [0.0, 0.0])
    assert np.isclose(canonical[4, 2], 0.0)
    assert np.isclose(canonical[8, 2], 0.1)
    assert np.isclose(canonical[0, 2], 0.8)


def test_lafan_extraction_to_live_leg_order() -> None:
    lafan = np.zeros((1, len(LAFAN_DEMO_JOINTS), 3), dtype=float)
    for i, name in enumerate(LAFAN_DEMO_JOINTS):
        lafan[0, i] = [i, i + 100.0, i + 200.0]

    live = extract_live_leg_sequence_from_lafan(lafan)

    assert live.shape == (1, len(LIVE_LEG_JOINTS), 3)
    for out_idx, name in enumerate(LIVE_LEG_JOINTS):
        src_idx = LAFAN_DEMO_JOINTS.index(name)
        expected = np.array([src_idx, src_idx + 200.0, src_idx + 100.0])
        if name == "Spine1":
            expected[2] -= 0.06
        assert np.allclose(live[0, out_idx], expected)
