"""Synthetic free-root walk: a translating pelvis over a floor, no hardware.

Drives a scripted walking gait through the same FreeRootTracker the live viewer
and recorder use, so you can see the foot-contact-anchored pelvis travel across
a floor grid with a breadcrumb trail. This is the no-hardware way to sanity-check
the free-root visualization before wearing the IMUs.

Usage (from project root):
  conda run --no-capture-output -n humanoid-sim python demos/demo_free_root_walk.py
  conda run --no-capture-output -n humanoid-sim python demos/demo_free_root_walk.py --no-show --frames 5
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from ik.free_root import FreeRootTracker  # noqa: E402
from ik.lower_body_aggregation import aggregate_lower_body_skeleton  # noqa: E402
from sensor.packet import SegmentId  # noqa: E402
from simulator.mujoco_lower_body import lower_body_points_from_skeleton  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fps", type=float, default=30.0, help="Playback/step rate.")
    parser.add_argument("--cadence-s", type=float, default=1.1, help="Seconds per stride.")
    parser.add_argument("--frames", type=int, default=0, help="Stop after N frames (0 = until closed).")
    parser.add_argument("--no-show", action="store_true", help="Run headless and save a PNG of the last frame.")
    parser.add_argument(
        "--output", type=Path,
        default=Path("data/visualizations/free_root_walk_last_frame.png"),
        help="PNG path used with --no-show.",
    )
    return parser.parse_args()


def walking_orientations(t_s: float, cadence_s: float) -> dict[SegmentId, Rotation]:
    """A simple alternating-stance gait, parameterised by time.

    One leg is in stance (extended, foot down/back) while the other swings
    through (hip flexes, knee bends, then plants ahead). The asymmetry is what
    lets the contact-anchored pelvis ratchet forward instead of marching in
    place.
    """
    phase = (t_s / cadence_s) * 2.0 * np.pi
    # Per-leg swing signals, half a cycle apart.
    left_s = np.sin(phase)
    right_s = np.sin(phase + np.pi)

    def leg(swing: float):
        # Hip pitch about the medio-lateral axis (+Y). In this body model a -y
        # hip rotation carries the ankle toward +x (forward), so a forward swing
        # uses negative pitch.
        hip_pitch = -25.0 * swing
        # Knee bends most during swing (when the foot is off the ground), which
        # also lifts the foot so it reads as swing (high) not stance (low).
        knee_pitch = 35.0 * max(0.0, swing)
        # Ankle keeps the foot roughly flat.
        ankle_pitch = -0.5 * hip_pitch
        hip = Rotation.from_euler("y", hip_pitch, degrees=True)
        knee = Rotation.from_euler("y", knee_pitch, degrees=True)
        ankle = Rotation.from_euler("y", ankle_pitch, degrees=True)
        return hip, knee, ankle

    pelvis = Rotation.identity()
    lh, lk, la = leg(left_s)
    rh, rk, ra = leg(right_s)
    return {
        SegmentId.PELVIS: pelvis,
        SegmentId.LEFT_THIGH: pelvis * lh,
        SegmentId.LEFT_SHANK: pelvis * lh * lk,
        SegmentId.LEFT_FOOT: pelvis * lh * lk * la,
        SegmentId.RIGHT_THIGH: pelvis * rh,
        SegmentId.RIGHT_SHANK: pelvis * rh * rk,
        SegmentId.RIGHT_FOOT: pelvis * rh * rk * ra,
    }


class FreeRootWalkPlot:
    """3D skeleton over a scrolling floor grid with a pelvis trail."""

    GRID_SPACING = 0.5
    GRID_HALF_LINES = 8

    def __init__(self) -> None:
        self.fig = plt.figure(figsize=(8, 8))
        self.ax = ax = self.fig.add_subplot(111, projection="3d")
        ax.set_xlabel("x forward (m)")
        ax.set_ylabel("y left (m)")
        ax.set_zlabel("z up (m)")
        ax.set_zlim(-0.05, 1.2)
        ax.set_box_aspect((1.0, 1.0, 0.9))
        ax.set_proj_type("ortho")
        ax.view_init(elev=18, azim=-62)

        # Mute matplotlib's own panes + back-wall gridlines so they don't
        # compete with our floor grid. Only our z=0 grid should read as "ground".
        for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
            axis._axinfo["grid"]["color"] = (1, 1, 1, 0)  # hide pane gridlines
            axis.set_pane_color((1.0, 1.0, 1.0, 0.0))      # transparent panes

        _L, _R = "#2a9d8f", "#e76f51"
        self.lines = {
            "pelvis": ax.plot([], [], [], color="#555555", lw=3, marker="o", ms=5, label="pelvis")[0],
            "left_thigh": ax.plot([], [], [], color=_L, lw=3, marker="o", ms=4, label="left")[0],
            "left_shank": ax.plot([], [], [], color=_L, lw=3, marker="o", ms=4)[0],
            "left_foot": ax.plot([], [], [], color=_L, lw=3, marker="o", ms=4)[0],
            "right_thigh": ax.plot([], [], [], color=_R, lw=3, marker="o", ms=4, label="right")[0],
            "right_shank": ax.plot([], [], [], color=_R, lw=3, marker="o", ms=4)[0],
            "right_foot": ax.plot([], [], [], color=_R, lw=3, marker="o", ms=4)[0],
        }
        ax.legend(loc="upper left", fontsize=8)

        # A faint filled floor plane gives the grid something to sit on so it
        # reads as ground rather than free-floating lines.
        self.floor_plane = ax.plot_surface(
            np.zeros((2, 2)), np.zeros((2, 2)), np.zeros((2, 2)),
            color="#eef1f5", alpha=0.55, zorder=0, shade=False,
        )
        self.floor = [
            ax.plot([], [], [], color="#5b6b7a", lw=1.0, alpha=0.9, zorder=1)[0]
            for _ in range(2 * (2 * self.GRID_HALF_LINES + 1))
        ]
        (self.origin,) = ax.plot([0, 0], [0, 0], [0, 0.15], color="#b5651d", lw=3, marker="o", ms=5, zorder=4)
        (self.trail,) = ax.plot([], [], [], color="#c44", lw=1.6, linestyle="dotted", zorder=3)
        self.trail_xyz: list[np.ndarray] = []
        self.title = ax.set_title("Free-root walk (synthetic)")

    def _grid(self, px: float, py: float) -> None:
        cx = round(px / self.GRID_SPACING) * self.GRID_SPACING
        cy = round(py / self.GRID_SPACING) * self.GRID_SPACING
        span = self.GRID_HALF_LINES * self.GRID_SPACING

        # Re-lay the filled floor plane under the camera (plot_surface can't be
        # updated in place, so swap it).
        self.floor_plane.remove()
        gx, gy = np.meshgrid([cx - span, cx + span], [cy - span, cy + span])
        self.floor_plane = self.ax.plot_surface(
            gx, gy, np.zeros_like(gx), color="#eef1f5", alpha=0.55, zorder=0, shade=False,
        )

        idx = 0
        for i in range(-self.GRID_HALF_LINES, self.GRID_HALF_LINES + 1):
            y = cy + i * self.GRID_SPACING
            gl = self.floor[idx]; idx += 1
            gl.set_data([cx - span, cx + span], [y, y]); gl.set_3d_properties([0, 0])
            x = cx + i * self.GRID_SPACING
            gl = self.floor[idx]; idx += 1
            gl.set_data([x, x], [cy - span, cy + span]); gl.set_3d_properties([0, 0])

    def update(self, skeleton, frame_index: int) -> None:
        measured, _ = lower_body_points_from_skeleton(skeleton)
        if "pelvis" in measured:
            c = measured["pelvis"]
            self.lines["pelvis"].set_data(c[:, 0], c[:, 1]); self.lines["pelvis"].set_3d_properties(c[:, 2])
        for key in ("left_thigh", "left_shank", "left_foot", "right_thigh", "right_shank", "right_foot"):
            if key in measured:
                c = measured[key]
                self.lines[key].set_data(c[:, 0], c[:, 1]); self.lines[key].set_3d_properties(c[:, 2])

        px, py, _pz = skeleton.pelvis_center
        self.ax.set_xlim(px - 0.8, px + 0.8)
        self.ax.set_ylim(py - 0.8, py + 0.8)
        pt = np.asarray(skeleton.pelvis_center, dtype=float)
        if not self.trail_xyz or np.linalg.norm(pt - self.trail_xyz[-1]) > 0.01:
            self.trail_xyz.append(pt.copy())
        arr = np.asarray(self.trail_xyz)
        self.trail.set_data(arr[:, 0], arr[:, 1]); self.trail.set_3d_properties(arr[:, 2])
        self._grid(px, py)
        travel = float(np.linalg.norm(pt[:2] - self.trail_xyz[0][:2]))
        self.title.set_text(f"Free-root walk (synthetic) — frame {frame_index}  travel {travel:.2f} m")


def main() -> None:
    args = parse_args()
    tracker = FreeRootTracker()
    plot = FreeRootWalkPlot()
    dt = 1.0 / args.fps

    if not args.no_show:
        plt.ion()
        plt.show(block=False)

    frame_limit = args.frames if args.frames > 0 else 10_000_000
    t_s = 0.0
    for frame_index in range(frame_limit):
        orientations = walking_orientations(t_s, args.cadence_s)
        pelvis_center = tracker.update(orientations, dt=dt)
        skeleton = aggregate_lower_body_skeleton(orientations, pelvis_center=pelvis_center)
        plot.update(skeleton, frame_index)
        t_s += dt

        if args.no_show:
            continue
        plt.pause(dt)
        if not plt.fignum_exists(plot.fig.number):
            break

    if args.no_show:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        plot.fig.savefig(args.output, dpi=160)
        print(f"saved: {args.output}")
    else:
        plt.ioff()
        plt.show()


if __name__ == "__main__":
    main()
