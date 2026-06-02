"""Free-root (translating pelvis) tracking from lower-body IMU orientations.

The base skeleton in :mod:`ik.lower_body_aggregation` pins the pelvis at a fixed
point, so it captures *posture* but not *travel* across the floor. This module
recovers pelvis translation using foot-contact anchoring (a.k.a. zero-velocity
update / ZUPT): at each frame the planted foot is held fixed in the world, and
the pelvis position is solved *up* the leg chain from that anchor. When the
stance foot switches, the anchor is handed to the new foot at its current world
position, so translation accumulates as the person walks.

This is pure kinematics, not physics simulation. It needs only the orientation
data already streamed, plus bone lengths. The dominant error is heading/yaw
drift (a magnetometer problem), and small per-step error from approximate
orientation-only contact detection.

The solve reuses :func:`aggregate_lower_body_skeleton` to turn orientations into
joint offsets, then chooses a moving ``pelvis_center`` to feed back into it.
``aggregate_lower_body_skeleton`` itself is unchanged; the fixed-pelvis path
remains the default everywhere.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping

import numpy as np
from scipy.spatial.transform import Rotation

from ik.lower_body_aggregation import aggregate_lower_body_skeleton
from model.lower_body import LowerBodyDimensions
from sensor.packet import SegmentId


Vector3 = np.ndarray
Side = str


@dataclass(frozen=True)
class FreeRootConfig:
    """Tuning for stance detection and anchoring."""

    # Stance score = ankle_z + speed_weight * ankle_speed. Lower score = more
    # planted. The foot with the lower score is the stance candidate.
    speed_weight: float = 0.15
    # Hysteresis: only switch stance feet when the other foot's score beats the
    # current stance foot's score by at least this margin (metres-equivalent).
    switch_margin: float = 0.03
    # Number of consecutive frames the challenger must win by the margin before
    # the anchor is handed over (debounces double-support flutter).
    switch_frames: int = 2


@dataclass
class FreeRootState:
    """Mutable per-session tracking state (exposed for debugging/visualisation)."""

    pelvis_center: Vector3 = field(default_factory=lambda: np.array([0.0, 0.0, 1.0]))
    stance: Side = "left"
    anchor_world: Vector3 | None = None  # world position the stance ankle is pinned to
    prev_ankles_world: dict[Side, Vector3] | None = None
    challenge_side: Side | None = None
    challenge_count: int = 0


_ANKLE_JOINT = {"left": "left_ankle", "right": "right_ankle"}


class FreeRootTracker:
    """Stateful pelvis-translation estimator from lower-body orientations.

    Feed it calibrated segment orientations each frame via :meth:`update`; it
    returns the moving ``pelvis_center`` to pass into
    :func:`aggregate_lower_body_skeleton` so the whole skeleton travels in a
    consistent world frame.
    """

    def __init__(
        self,
        *,
        dimensions: LowerBodyDimensions = LowerBodyDimensions(),
        config: FreeRootConfig = FreeRootConfig(),
    ) -> None:
        self.dimensions = dimensions
        self.config = config
        self.state = FreeRootState()
        self._initialised = False

    def reset(self) -> None:
        self.state = FreeRootState()
        self._initialised = False

    def _ankle_offsets(self, orientations: Mapping[SegmentId, Rotation]) -> dict[Side, Vector3]:
        """Vector from pelvis origin to each ankle, in world axes (orientation only)."""
        sk0 = aggregate_lower_body_skeleton(
            orientations, dimensions=self.dimensions, pelvis_center=np.zeros(3)
        )
        return {
            "left": np.asarray(sk0.joints["left_ankle"], dtype=float),
            "right": np.asarray(sk0.joints["right_ankle"], dtype=float),
        }

    def update(
        self,
        orientations: Mapping[SegmentId, Rotation],
        *,
        dt: float = 1.0 / 50.0,
    ) -> Vector3:
        """Advance one frame and return the world ``pelvis_center``.

        ``dt`` is the time since the previous frame, used only to turn ankle
        displacement into a speed for stance scoring.
        """
        offsets = self._ankle_offsets(orientations)

        if not self._initialised:
            return self._initialise(offsets)

        cfg = self.config
        st = self.state
        dt = max(dt, 1e-6)

        # Current ankle world positions implied by the *previous* pelvis (the
        # pelvis has not moved yet this frame). Used for stance scoring and to
        # keep the handoff continuous.
        ankles_world = {side: st.pelvis_center + offsets[side] for side in ("left", "right")}

        # Stance score: lower ankle + slower ankle is "more planted".
        scores: dict[Side, float] = {}
        for side in ("left", "right"):
            if st.prev_ankles_world is not None:
                speed = float(np.linalg.norm(ankles_world[side] - st.prev_ankles_world[side]) / dt)
            else:
                speed = 0.0
            scores[side] = float(ankles_world[side][2]) + cfg.speed_weight * speed

        other = "right" if st.stance == "left" else "left"
        # Hysteresis + debounce before switching the anchor to the other foot.
        if scores[other] < scores[st.stance] - cfg.switch_margin:
            if st.challenge_side == other:
                st.challenge_count += 1
            else:
                st.challenge_side = other
                st.challenge_count = 1
            if st.challenge_count >= cfg.switch_frames:
                # Hand the anchor to the new stance foot at its current world
                # position so the pelvis trajectory stays continuous.
                st.stance = other
                st.anchor_world = ankles_world[other].copy()
                st.challenge_side = None
                st.challenge_count = 0
        else:
            st.challenge_side = None
            st.challenge_count = 0

        # Solve the pelvis so the stance ankle stays pinned to its anchor.
        assert st.anchor_world is not None
        st.pelvis_center = st.anchor_world - offsets[st.stance]
        st.prev_ankles_world = {
            side: st.pelvis_center + offsets[side] for side in ("left", "right")
        }
        return st.pelvis_center.copy()

    def _initialise(self, offsets: dict[Side, Vector3]) -> Vector3:
        """First frame: drop the lower foot onto the floor (z=0) at the origin."""
        st = self.state
        # Place the pelvis so the lower ankle sits on the ground plane (z=0) and
        # the body starts centred over the origin in x/y.
        lower = "left" if offsets["left"][2] <= offsets["right"][2] else "right"
        st.stance = lower
        ground_z = -float(offsets[lower][2])
        st.pelvis_center = np.array([0.0, 0.0, ground_z], dtype=float)
        st.anchor_world = (st.pelvis_center + offsets[lower]).copy()
        st.prev_ankles_world = {
            side: st.pelvis_center + offsets[side] for side in ("left", "right")
        }
        self._initialised = True
        return st.pelvis_center.copy()
