"""Tests for free-root (translating pelvis) tracking."""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from ik.free_root import FreeRootTracker
from ik.lower_body_aggregation import aggregate_lower_body_skeleton
from sensor.packet import SegmentId


def _neutral_orientations() -> dict[SegmentId, Rotation]:
    """All segments upright/identity: a symmetric standing pose."""
    return {
        SegmentId.PELVIS: Rotation.identity(),
        SegmentId.LEFT_THIGH: Rotation.identity(),
        SegmentId.LEFT_SHANK: Rotation.identity(),
        SegmentId.LEFT_FOOT: Rotation.identity(),
        SegmentId.RIGHT_THIGH: Rotation.identity(),
        SegmentId.RIGHT_SHANK: Rotation.identity(),
        SegmentId.RIGHT_FOOT: Rotation.identity(),
    }


def _ankle_world(orientations, pelvis_center, side: str) -> np.ndarray:
    sk = aggregate_lower_body_skeleton(orientations, pelvis_center=pelvis_center)
    return np.asarray(sk.joints[f"{side}_ankle"], dtype=float)


def test_standing_still_pelvis_does_not_drift():
    tracker = FreeRootTracker()
    orientations = _neutral_orientations()
    first = tracker.update(orientations)
    for _ in range(50):
        latest = tracker.update(orientations)
    # A perfectly still pose must not accumulate translation.
    assert np.allclose(first, latest, atol=1e-9)


def test_initialises_lower_foot_onto_ground():
    tracker = FreeRootTracker()
    orientations = _neutral_orientations()
    pelvis = tracker.update(orientations)
    # The stance ankle should sit on the floor plane z=0 after initialisation.
    stance = tracker.state.stance
    ankle_z = _ankle_world(orientations, pelvis, stance)[2]
    assert abs(ankle_z) < 1e-9


def test_stance_foot_stays_anchored_within_a_step():
    tracker = FreeRootTracker()
    orientations = _neutral_orientations()
    tracker.update(orientations)
    stance = tracker.state.stance
    anchor = tracker.state.anchor_world.copy()

    # Swing the NON-stance thigh forward (hip flexion about +Y) a little. The
    # stance ankle must remain pinned to its anchor while the pelvis adjusts.
    swing = "right" if stance == "left" else "left"
    seg = {"left": SegmentId.LEFT_THIGH, "right": SegmentId.RIGHT_THIGH}[swing]
    for deg in (5.0, 10.0, 15.0):
        moved = dict(orientations)
        moved[seg] = Rotation.from_euler("y", deg, degrees=True)
        pelvis = tracker.update(moved)
        anchored = _ankle_world(moved, pelvis, stance)
        assert np.allclose(anchored, anchor, atol=1e-9)


def test_pelvis_translates_when_body_leans_forward():
    tracker = FreeRootTracker()
    base = _neutral_orientations()
    tracker.update(base)
    start = tracker.state.pelvis_center.copy()

    # Lean the whole body forward about +Y (pitch). With the stance foot pinned,
    # leaning forward must move the pelvis in +X (forward), not keep it fixed.
    moved = {seg: Rotation.from_euler("y", 20.0, degrees=True) for seg in base}
    pelvis = tracker.update(moved)
    assert pelvis[0] > start[0] + 1e-3


def test_reset_clears_state():
    tracker = FreeRootTracker()
    orientations = _neutral_orientations()
    tracker.update(orientations)
    tracker.reset()
    assert tracker._initialised is False
    # After reset, the first update re-initialises cleanly.
    pelvis = tracker.update(orientations)
    assert pelvis.shape == (3,)
