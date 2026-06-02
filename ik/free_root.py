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

    # Stance detection uses the physical truth that the stance foot is the anchor
    # and cannot swing: each frame we only test whether the SWING foot has
    # planted. A plant requires the swing foot to be low to the ground, no longer
    # advancing forward in the pelvis frame (weight transferring onto it), and
    # clearly ahead of the stance foot. Because only a positive plant event hands
    # off the anchor, two near-equal feet can never cause a flip-flop.
    #
    # Swing-foot ankle height (m) below which it counts as "on the floor".
    plant_height: float = 0.06
    # Swing-foot forward velocity (m/s, pelvis frame) below which it counts as
    # "settled" (stopped advancing -> weight coming onto it).
    plant_speed: float = 0.1
    # The swing foot must be at least this far ahead of the stance foot (forward,
    # pelvis frame) before it can plant. Stops re-anchoring before a real step.
    min_swing_advance: float = 0.04
    # Number of consecutive frames the plant condition must hold before the
    # anchor is handed over (debounces double-support flutter).
    switch_frames: int = 2


@dataclass
class FreeRootState:
    """Mutable per-session tracking state (exposed for debugging/visualisation)."""

    pelvis_center: Vector3 = field(default_factory=lambda: np.array([0.0, 0.0, 1.0]))
    stance: Side = "left"
    anchor_world: Vector3 | None = None  # world position the stance ankle is pinned to
    # Each foot's offset from the pelvis (pelvis frame), previous frame, used to
    # measure pelvis-relative foot motion.
    prev_offsets: dict[Side, Vector3] | None = None
    # Consecutive frames the swing-foot plant condition has held.
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
            return self._initialise(offsets, orientations)

        cfg = self.config
        st = self.state
        dt = max(dt, 1e-6)

        # Pelvis-frame foot offsets: rotate the world-axis offsets into the
        # pelvis's own frame so "forward" means the body's forward, regardless of
        # which way the person is facing. The pelvis +X axis is forward.
        pelvis_rot = orientations[SegmentId.PELVIS]
        pelvis_offsets = {side: pelvis_rot.inv().apply(offsets[side]) for side in ("left", "right")}

        ankles_world = {side: st.pelvis_center + offsets[side] for side in ("left", "right")}

        # Asymmetric model (matches the physical truth): the stance foot is the
        # anchor by definition and *cannot* swing, so we never re-evaluate it.
        # We only ask one question each frame — has the SWING foot planted? A
        # plant means the swing foot is (a) low to the ground, (b) no longer
        # advancing forward in the pelvis frame (its forward velocity has dropped
        # to ~0 or reversed, i.e. weight is transferring onto it), and (c) it has
        # swung clearly ahead of the current stance foot. Only a positive plant
        # event hands off the anchor, so two near-equal feet can never cause a
        # flip-flop.
        swing = "right" if st.stance == "left" else "left"
        if st.prev_offsets is not None:
            swing_fwd_vel = float((pelvis_offsets[swing][0] - st.prev_offsets[swing][0]) / dt)
        else:
            swing_fwd_vel = 0.0

        swing_low = float(ankles_world[swing][2]) < cfg.plant_height
        swing_settled = swing_fwd_vel <= cfg.plant_speed
        swing_ahead = (pelvis_offsets[swing][0] - pelvis_offsets[st.stance][0]) >= cfg.min_swing_advance

        if swing_low and swing_settled and swing_ahead:
            st.challenge_count += 1
            if st.challenge_count >= cfg.switch_frames:
                # Plant confirmed: hand the anchor to the swing foot. Pin its x/y
                # where it is, but force z=0 (it's on the floor by definition);
                # anchoring at its measured z would let small per-step height
                # errors accumulate into vertical drift (sinking/rising).
                st.stance = swing
                anchor = ankles_world[swing].copy()
                anchor[2] = 0.0
                st.anchor_world = anchor
                st.challenge_count = 0
        else:
            st.challenge_count = 0

        # Solve the pelvis so the stance ankle stays pinned to its anchor.
        assert st.anchor_world is not None
        st.pelvis_center = st.anchor_world - offsets[st.stance]
        st.prev_offsets = pelvis_offsets
        return st.pelvis_center.copy()

    def _initialise(
        self,
        offsets: dict[Side, Vector3],
        orientations: Mapping[SegmentId, Rotation],
    ) -> Vector3:
        """First frame: drop the lower foot onto the floor (z=0) at the origin."""
        st = self.state
        # Place the pelvis so the lower ankle sits on the ground plane (z=0) and
        # the body starts centred over the origin in x/y.
        lower = "left" if offsets["left"][2] <= offsets["right"][2] else "right"
        st.stance = lower
        ground_z = -float(offsets[lower][2])
        st.pelvis_center = np.array([0.0, 0.0, ground_z], dtype=float)
        # Stance ankle sits exactly on the floor plane (z=0); pin x/y, force z=0.
        anchor = st.pelvis_center + offsets[lower]
        anchor[2] = 0.0
        st.anchor_world = anchor.copy()
        pelvis_rot = orientations[SegmentId.PELVIS]
        st.prev_offsets = {side: pelvis_rot.inv().apply(offsets[side]) for side in ("left", "right")}
        self._initialised = True
        return st.pelvis_center.copy()
