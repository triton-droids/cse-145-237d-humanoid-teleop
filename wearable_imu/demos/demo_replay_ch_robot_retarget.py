"""Replay a recorded IMU handoff clip through the ch_robot Plan A retargeter.

This is the offline counterpart to ``demo_live_retarget.py``.  It accepts the
``human_joint_clip_*.npz`` files saved by the IMU recorder, reconstructs
approximate segment rotations from the 9 recorded joint positions, converts
them to ch_robot ``qpos[17]``/``qvel[16]``, and optionally visualizes playback.

Usage:
  python demos/demo_replay_ch_robot_retarget.py ../data/human_joint_clip_20260601_231345.npz
  python demos/demo_replay_ch_robot_retarget.py ../data/human_joint_clip_20260601_231345.npz --save-output ../data/ch_robot_replay_qpos.npz
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PROJECT_ROOT.parent
for path in (PROJECT_ROOT, REPO_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from ik.ch_robot_retarget import (  # noqa: E402
    CH_ROBOT_JOINT_NAMES,
    human_joint_clip_to_qpos_qvel,
    load_human_joint_clip,
)


_SPINE1 = 0
_L_HIP, _L_KNEE, _L_ANKLE, _L_TOE = 1, 2, 3, 4
_R_HIP, _R_KNEE, _R_ANKLE, _R_TOE = 5, 6, 7, 8
_PELVIS_CHAIN = (_L_HIP, _SPINE1, _R_HIP)
_LEFT_CHAIN = (_L_HIP, _L_KNEE, _L_ANKLE, _L_TOE)
_RIGHT_CHAIN = (_R_HIP, _R_KNEE, _R_ANKLE, _R_TOE)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("clip", type=Path, help="Recorded human_joint_clip_*.npz path.")
    parser.add_argument(
        "--frame-key",
        choices=("joint_pos_origin", "joint_pos_w"),
        default="joint_pos_origin",
        help="Position array to replay.",
    )
    parser.add_argument("--yaw-mode", choices=("keep", "strip"), default="keep")
    parser.add_argument("--base-height", type=float, default=0.765)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--no-show", action="store_true", help="Convert and print a summary without opening a window.")
    parser.add_argument("--save-output", type=Path, default=None, help="Optional .npz path for qpos/qvel output.")
    return parser.parse_args()


def _save_output(path: Path, *, qpos: np.ndarray, qvel: np.ndarray, timestamps_s: np.ndarray, fps: float) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        path,
        qpos=qpos,
        qvel=qvel,
        timestamps_s=timestamps_s,
        fps=np.array([fps], dtype=np.float64),
        joint_names=np.array(CH_ROBOT_JOINT_NAMES),
    )
    print(f"Saved ch_robot replay: {path}")


def _print_summary(args: argparse.Namespace, points: np.ndarray, qpos: np.ndarray, qvel: np.ndarray, fps: float) -> None:
    print(f"Clip       : {args.clip}")
    print(f"frame key  : {args.frame_key}")
    print(f"frames     : {points.shape[0]}")
    print(f"fps        : {fps:.1f}")
    print(f"qpos       : {qpos.shape}")
    print(f"qvel       : {qvel.shape}")
    print(f"yaw mode   : {args.yaw_mode}")
    print("joint order: " + ", ".join(CH_ROBOT_JOINT_NAMES))


def main() -> None:
    args = parse_args()
    clip = load_human_joint_clip(args.clip, frame_key=args.frame_key)
    qpos, qvel = human_joint_clip_to_qpos_qvel(
        clip,
        base_height=args.base_height,
        yaw_mode=args.yaw_mode,
    )
    points = clip.joint_positions

    _print_summary(args, points, qpos, qvel, clip.fps)
    if args.save_output is not None:
        _save_output(
            args.save_output,
            qpos=qpos,
            qvel=qvel,
            timestamps_s=clip.timestamps_s,
            fps=clip.fps,
        )
    if args.no_show:
        return

    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button, Slider

    mins = points.reshape(-1, 3).min(axis=0)
    maxs = points.reshape(-1, 3).max(axis=0)
    center = 0.5 * (mins + maxs)
    radius = max(0.5 * float(np.max(maxs - mins)), 0.4)

    plt.ion()
    fig = plt.figure(figsize=(9, 8))
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
    fig.subplots_adjust(bottom=0.20, right=0.72)

    def _mk(color: str, label: str):
        return ax.plot([], [], [], color=color, linewidth=3, marker="o", markersize=5, label=label)[0]

    lines = {
        "pelvis": _mk("#555555", "pelvis"),
        "left": _mk("#2a9d8f", "left"),
        "right": _mk("#e76f51", "right"),
    }
    ax.legend(loc="upper left", fontsize=8)
    readout = ax.text2D(1.03, 0.96, "", transform=ax.transAxes, va="top", family="monospace", fontsize=8)

    n_frames = points.shape[0]
    timestamps = clip.timestamps_s - clip.timestamps_s[0]

    def _draw_frame(index: int) -> None:
        frame = points[index]
        for key, chain in (
            ("pelvis", _PELVIS_CHAIN),
            ("left", _LEFT_CHAIN),
            ("right", _RIGHT_CHAIN),
        ):
            coords = frame[list(chain)]
            line = lines[key]
            line.set_data(coords[:, 0], coords[:, 1])
            line.set_3d_properties(coords[:, 2])

        joints_deg = np.rad2deg(qpos[index, 7:])
        joint_lines = "\n".join(
            f"{name.replace('_joint', ''):>15}: {value:7.2f}"
            for name, value in zip(CH_ROBOT_JOINT_NAMES, joints_deg)
        )
        readout.set_text(f"frame {index + 1}/{n_frames}\nqpos joints deg\n{joint_lines}")
        ax.set_title(f"{args.clip.name} -> ch_robot qpos  frame {index + 1}/{n_frames}")

    play_ax = fig.add_axes([0.12, 0.06, 0.18, 0.05])
    play_button = Button(play_ax, "Pause")
    slider_ax = fig.add_axes([0.38, 0.07, 0.48, 0.03])
    frame_slider = Slider(slider_ax, "frame", 0, n_frames - 1, valinit=0, valstep=1)

    state = {
        "frame": 0,
        "playing": True,
        "clock0": time.monotonic(),
        "clock_t0": float(timestamps[0]),
    }

    def _on_slider(value: float) -> None:
        index = int(value)
        state["frame"] = index
        state["clock0"] = time.monotonic()
        state["clock_t0"] = float(timestamps[index])
        _draw_frame(index)
        fig.canvas.draw_idle()

    frame_slider.on_changed(_on_slider)

    def _toggle_play(_event) -> None:
        state["playing"] = not state["playing"]
        play_button.label.set_text("Pause" if state["playing"] else "Play")
        state["clock0"] = time.monotonic()
        state["clock_t0"] = float(timestamps[state["frame"]])
        fig.canvas.draw_idle()

    play_button.on_clicked(_toggle_play)

    _draw_frame(0)
    fig.canvas.draw_idle()
    speed = max(args.speed, 1e-3)
    print("\nPlayback running. Close the window to exit.\n")

    while plt.fignum_exists(fig.number):
        now = time.monotonic()
        if state["playing"]:
            target_t = state["clock_t0"] + (now - state["clock0"]) * speed
            index = state["frame"]
            while index + 1 < n_frames and timestamps[index + 1] <= target_t:
                index += 1
            if target_t > timestamps[-1]:
                index = 0
                state["clock0"] = now
                state["clock_t0"] = float(timestamps[0])
            if index != state["frame"]:
                state["frame"] = index
                _draw_frame(index)
                frame_slider.eventson = False
                frame_slider.set_val(index)
                frame_slider.eventson = True
        fig.canvas.draw_idle()
        fig.canvas.flush_events()
        time.sleep(0.01)


if __name__ == "__main__":
    main()
