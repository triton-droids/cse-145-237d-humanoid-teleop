"""Play back a recorded human-joint .npz clip as a 3D skeleton.

Loads a clip saved by demo_record_human_joint_clip.py (or the live viewer's
Record button) and animates the joint positions. Includes a Play/Pause button
and a frame scrub slider.

Usage (from project root):
  conda run --no-capture-output -n humanoid-sim python demos\demo_play_human_joint_clip.py data\recordings\live_human_joint_clip.npz
  conda run --no-capture-output -n humanoid-sim python demos\demo_play_human_joint_clip.py data\recordings\human_joint_clip.npz --origin
  conda run --no-capture-output -n humanoid-sim python demos\demo_play_human_joint_clip.py CLIP.npz --no-show   # print a summary and exit
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


# Joint order saved by the recorder (see JOINT_NAMES in
# demo_record_human_joint_clip.py).
_SPINE1 = 0
_L_HIP, _L_KNEE, _L_ANKLE, _L_TOE = 1, 2, 3, 4
_R_HIP, _R_KNEE, _R_ANKLE, _R_TOE = 5, 6, 7, 8

# Bone connections drawn each frame, grouped so left/right can be colored.
_PELVIS_CHAIN = (_L_HIP, _SPINE1, _R_HIP)
_LEFT_CHAIN = (_L_HIP, _L_KNEE, _L_ANKLE, _L_TOE)
_RIGHT_CHAIN = (_R_HIP, _R_KNEE, _R_ANKLE, _R_TOE)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("clip", type=Path, help="Path to the .npz clip to play")
    parser.add_argument(
        "--origin",
        action="store_true",
        help="Show pelvis-origin-relative joints (joint_pos_origin) instead of world.",
    )
    parser.add_argument(
        "--speed", type=float, default=1.0,
        help="Playback speed multiplier (default: 1.0 = recorded fps).",
    )
    parser.add_argument(
        "--no-show", action="store_true",
        help="Print a summary and exit without opening a window.",
    )
    return parser.parse_args()


def _load_clip(path: Path, use_origin: bool) -> dict:
    if not path.exists():
        raise SystemExit(f"clip not found: {path}")
    data = np.load(path, allow_pickle=True)
    key = "joint_pos_origin" if use_origin else "joint_pos_w"
    if key not in data.files:
        raise SystemExit(f"{path} has no '{key}' array (found: {', '.join(data.files)})")
    points = np.asarray(data[key], dtype=float)  # (frames, joints, 3)
    fps = float(np.asarray(data["fps"]).reshape(-1)[0]) if "fps" in data.files else 30.0
    config = str(np.asarray(data["config"]).reshape(-1)[0]) if "config" in data.files else "?"
    names = [str(n) for n in data["joint_names"]] if "joint_names" in data.files else []
    return {"points": points, "fps": fps, "config": config, "names": names, "frame_key": key}


def _print_summary(clip: dict, path: Path) -> None:
    pts = clip["points"]
    print(f"Clip: {path}")
    print(f"  frames : {pts.shape[0]}")
    print(f"  joints : {pts.shape[1]} ({', '.join(clip['names'])})")
    print(f"  fps    : {clip['fps']:.1f}")
    print(f"  config : {clip['config']}")
    print(f"  source : {clip['frame_key']}")
    duration = pts.shape[0] / clip["fps"] if clip["fps"] > 0 else 0.0
    print(f"  length : {duration:.2f} s")


def main() -> None:
    args = parse_args()
    clip = _load_clip(args.clip, args.origin)
    _print_summary(clip, args.clip)

    points = clip["points"]
    n_frames = points.shape[0]
    if n_frames == 0:
        raise SystemExit("clip has no frames")
    if args.no_show:
        return

    # Fixed axis bounds from the whole clip so the skeleton does not jump.
    mins = points.reshape(-1, 3).min(axis=0)
    maxs = points.reshape(-1, 3).max(axis=0)
    center = 0.5 * (mins + maxs)
    radius = max(0.5 * float(np.max(maxs - mins)), 0.4)

    plt.ion()
    fig = plt.figure(figsize=(7, 9))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlabel("x forward (m)")
    ax.set_ylabel("y left (m)")
    ax.set_zlabel("z up (m)")
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)
    ax.set_box_aspect((1.0, 1.0, 1.0))
    ax.set_proj_type("ortho")
    ax.view_init(elev=18, azim=-62)
    fig.subplots_adjust(bottom=0.18)

    _PELVIS_C, _L_C, _R_C = "#555555", "#2a9d8f", "#e76f51"

    def _mk(color: str, label: str = ""):
        (ln,) = ax.plot([], [], [], color=color, linewidth=3, marker="o", markersize=5, label=label)
        return ln

    lines = {
        "pelvis": _mk(_PELVIS_C, "pelvis"),
        "left": _mk(_L_C, "left"),
        "right": _mk(_R_C, "right"),
    }
    ax.legend(loc="upper left", fontsize=8)

    def _draw_frame(index: int) -> None:
        frame = points[index]
        for key, chain in (
            ("pelvis", _PELVIS_CHAIN),
            ("left", _LEFT_CHAIN),
            ("right", _RIGHT_CHAIN),
        ):
            coords = frame[list(chain)]
            ln = lines[key]
            ln.set_data(coords[:, 0], coords[:, 1])
            ln.set_3d_properties(coords[:, 2])
        ax.set_title(f"{args.clip.name}  —  frame {index + 1}/{n_frames}  ({clip['config']})")

    # --- transport controls ---------------------------------------------
    play_ax = fig.add_axes([0.13, 0.05, 0.20, 0.05])
    play_button = Button(play_ax, "Pause")
    slider_ax = fig.add_axes([0.40, 0.06, 0.48, 0.03])
    frame_slider = Slider(slider_ax, "frame", 0, n_frames - 1, valinit=0, valstep=1)

    state = {"frame": 0, "playing": True, "interacting_until": 0.0}

    def _on_slider(val: float) -> None:
        state["frame"] = int(val)
        _draw_frame(state["frame"])
        fig.canvas.draw_idle()

    frame_slider.on_changed(_on_slider)

    def _toggle_play(_evt) -> None:
        state["playing"] = not state["playing"]
        play_button.label.set_text("Pause" if state["playing"] else "Play")
        fig.canvas.draw_idle()

    play_button.on_clicked(_toggle_play)

    # Pause auto-advance briefly while the user drags to rotate (anti-flicker).
    import time

    def _pause_interaction(_evt) -> None:
        state["interacting_until"] = time.monotonic() + 0.6

    fig.canvas.mpl_connect("button_press_event", _pause_interaction)

    _draw_frame(0)
    fig.canvas.draw_idle()

    fps = clip["fps"] * max(args.speed, 1e-3)
    period = 1.0 / fps if fps > 0 else 1.0 / 30.0
    print("\nPlayback running. Play/Pause button, drag the slider to scrub.\n")

    last_advance = time.monotonic()
    while plt.fignum_exists(fig.number):
        now = time.monotonic()
        if state["playing"] and (now - last_advance) >= period:
            state["frame"] = (state["frame"] + 1) % n_frames
            last_advance = now
            _draw_frame(state["frame"])
            # Keep the slider in sync without re-triggering playback jumps.
            frame_slider.eventson = False
            frame_slider.set_val(state["frame"])
            frame_slider.eventson = True

        if now >= state["interacting_until"]:
            fig.canvas.draw_idle()
        fig.canvas.flush_events()
        time.sleep(0.01)

    print("Window closed.")


if __name__ == "__main__":
    main()
