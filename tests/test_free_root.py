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

    # Lift the NON-stance foot off the ground (bend its knee so the ankle rises).
    # A lifted foot is clearly the swing foot, so the stance foot must stay the
    # anchor and remain pinned to its world position while the pelvis adjusts.
    swing = "right" if stance == "left" else "left"
    thigh = {"left": SegmentId.LEFT_THIGH, "right": SegmentId.RIGHT_THIGH}[swing]
    shank = {"left": SegmentId.LEFT_SHANK, "right": SegmentId.RIGHT_SHANK}[swing]
    foot = {"left": SegmentId.LEFT_FOOT, "right": SegmentId.RIGHT_FOOT}[swing]
    for deg in (10.0, 20.0, 30.0):
        moved = dict(orientations)
        # Hip flexes forward (-y) and knee bends to lift the foot.
        moved[thigh] = Rotation.from_euler("y", -deg, degrees=True)
        moved[shank] = Rotation.from_euler("y", 0.0, degrees=True)
        moved[foot] = Rotation.from_euler("y", 0.0, degrees=True)
        pelvis = tracker.update(moved)
        assert tracker.state.stance == stance  # anchor must not switch
        anchored = _ankle_world(moved, pelvis, stance)
        assert np.allclose(anchored, anchor, atol=1e-9)


def test_pelvis_translates_when_body_leans_forward():
    tracker = FreeRootTracker()
    base = _neutral_orientations()
    tracker.update(base)
    start = tracker.state.pelvis_center.copy()

    # Rotate the whole body about +Y (pitch). With the stance foot pinned, this
    # must move the pelvis in x (the body cannot stay put while the legs tilt).
    # In this model +y pitch carries the ankle toward -x, so the pinned-ankle
    # solve pushes the pelvis the opposite way; we only assert it moved.
    moved = {seg: Rotation.from_euler("y", 20.0, degrees=True) for seg in base}
    pelvis = tracker.update(moved)
    assert abs(pelvis[0] - start[0]) > 1e-3


def test_stance_anchor_stays_on_floor_across_steps():
    """Regression: the body must not sink into (or rise out of) the ground.

    Every anchor handoff must pin the stance foot to z=0, otherwise small
    per-step height errors accumulate and the pelvis drifts vertically (ZUPT
    vertical drift). Drive an alternating gait and assert the stance ankle stays
    on the floor and the pelvis height stays in a sane band.
    """
    tracker = FreeRootTracker()
    base = _neutral_orientations()
    tracker.update(base)
    init_height = tracker.state.pelvis_center[2]

    dt = 1.0 / 30.0
    t = 0.0
    for _ in range(120):
        # Alternating hip swing -> forces repeated stance switches.
        ls = 20.0 * np.sin(2 * np.pi * t / 1.1)
        rs = 20.0 * np.sin(2 * np.pi * t / 1.1 + np.pi)
        o = dict(base)
        o[SegmentId.LEFT_THIGH] = Rotation.from_euler("y", ls, degrees=True)
        o[SegmentId.RIGHT_THIGH] = Rotation.from_euler("y", rs, degrees=True)
        pelvis = tracker.update(o, dt=dt)
        # Whichever foot is stance must be planted on the floor (z ~= 0).
        stance_ankle = _ankle_world(o, pelvis, tracker.state.stance)
        assert abs(stance_ankle[2]) < 1e-9
        t += dt

    # Pelvis height must not have drifted far from its starting value.
    assert abs(tracker.state.pelvis_center[2] - init_height) < 0.25


def test_reset_clears_state():
    tracker = FreeRootTracker()
    orientations = _neutral_orientations()
    tracker.update(orientations)
    tracker.reset()
    assert tracker._initialised is False
    # After reset, the first update re-initialises cleanly.
    pelvis = tracker.update(orientations)
    assert pelvis.shape == (3,)
