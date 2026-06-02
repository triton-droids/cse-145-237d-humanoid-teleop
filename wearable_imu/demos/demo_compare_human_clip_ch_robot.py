"""Open raw human clip playback and ch_robot MuJoCo replay together.

This is a convenience wrapper for side-by-side before/after inspection:

- before retargeting: ``demo_play_human_joint_clip.py`` raw human skeleton
- after retargeting: ``demo_mujoco_ch_robot_replay.py`` ch_robot MuJoCo robot

The input must be a recorded human-joint ``.npz`` clip. Camera ``.jsonl`` files
can be retargeted by the MuJoCo replay script, but they are not accepted by the
raw human clip player.

Usage:
  python demos/demo_compare_human_clip_ch_robot.py ../data/human_joint_clip_20260601_231345.npz
  python demos/demo_compare_human_clip_ch_robot.py ../data/human_joint_clip_20260601_231345.npz --human-origin --base-motion root_xy
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shlex
import subprocess
import sys
import time

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("clip", type=Path, help="Recorded human_joint_clip_*.npz path.")
    parser.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier for both windows.")
    parser.add_argument(
        "--human-origin",
        action="store_true",
        help="Show pelvis-origin-relative human joints in the raw skeleton player.",
    )
    parser.add_argument(
        "--frame-key",
        choices=("joint_pos_origin", "joint_pos_w"),
        default="joint_pos_origin",
        help="Position array used by the ch_robot replay retargeter.",
    )
    parser.add_argument("--yaw-mode", choices=("keep", "strip"), default="keep")
    parser.add_argument(
        "--base-motion",
        choices=("root_xy", "fixed", "root_xyz"),
        default="root_xy",
        help="How human root translation drives the MuJoCo freejoint base.",
    )
    parser.add_argument("--base-height", type=float, default=0.765)
    parser.add_argument("--refresh-model", action="store_true", help="Re-extract ch_robot assets before replay.")
    parser.add_argument("--no-loop", action="store_true", help="Do not loop the MuJoCo robot replay.")
    parser.add_argument("--no-show", action="store_true", help="Run both child demos in summary-only mode.")
    parser.add_argument(
        "--stagger-s",
        type=float,
        default=0.5,
        help="Seconds to wait between opening the human and MuJoCo windows.",
    )
    return parser.parse_args()


def validate_human_clip(path: Path) -> None:
    if path.suffix != ".npz":
        raise SystemExit(f"input must be a recorded human .npz clip, got: {path}")
    if not path.exists():
        raise SystemExit(f"clip not found: {path}")

    with np.load(path, allow_pickle=True) as data:
        required_any = {"joint_pos_w", "joint_pos_origin"}
        if not any(key in data.files for key in required_any):
            raise SystemExit(
                f"{path} is not a human joint clip; expected one of {sorted(required_any)}, "
                f"found: {', '.join(data.files)}"
            )


def build_human_command(args: argparse.Namespace) -> list[str]:
    command = [
        sys.executable,
        "-u",
        str(PROJECT_ROOT / "demos" / "demo_play_human_joint_clip.py"),
        str(args.clip),
        "--speed",
        str(args.speed),
    ]
    if args.human_origin:
        command.append("--origin")
    if args.no_show:
        command.append("--no-show")
    return command


def build_robot_command(args: argparse.Namespace) -> list[str]:
    command = [
        sys.executable,
        "-u",
        str(PROJECT_ROOT / "demos" / "demo_mujoco_ch_robot_replay.py"),
        str(args.clip),
        "--speed",
        str(args.speed),
        "--frame-key",
        args.frame_key,
        "--yaw-mode",
        args.yaw_mode,
        "--base-motion",
        args.base_motion,
        "--base-height",
        str(args.base_height),
    ]
    if args.refresh_model:
        command.append("--refresh-model")
    if args.no_loop:
        command.append("--no-loop")
    if args.no_show:
        command.append("--no-show")
    return command


def launch(name: str, command: list[str], env: dict[str, str]) -> subprocess.Popen:
    print(f"\nStarting {name}:")
    print("$ " + shlex.join(command))
    sys.stdout.flush()
    return subprocess.Popen(command, cwd=PROJECT_ROOT, env=env)


def child_environment() -> dict[str, str]:
    env = os.environ.copy()
    cache_root = PROJECT_ROOT / ".cache"
    matplotlib_cache = cache_root / "matplotlib"
    matplotlib_cache.mkdir(parents=True, exist_ok=True)
    env.setdefault("XDG_CACHE_HOME", str(cache_root))
    env.setdefault("MPLCONFIGDIR", str(matplotlib_cache))
    return env


def stop_processes(processes: dict[str, subprocess.Popen]) -> None:
    for process in processes.values():
        if process.poll() is None:
            process.terminate()
    deadline = time.monotonic() + 5.0
    for process in processes.values():
        remaining = max(deadline - time.monotonic(), 0.0)
        if process.poll() is None:
            try:
                process.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                process.kill()
    for process in processes.values():
        if process.poll() is None:
            process.wait()


def main() -> None:
    args = parse_args()
    validate_human_clip(args.clip)

    env = child_environment()
    processes: dict[str, subprocess.Popen] = {}
    try:
        processes["human clip player"] = launch("human clip player", build_human_command(args), env)
        if not args.no_show:
            time.sleep(max(args.stagger_s, 0.0))
        processes["ch_robot MuJoCo replay"] = launch("ch_robot MuJoCo replay", build_robot_command(args), env)

        print("\nComparison running. Close either window, or press Ctrl-C here, to stop both.\n")
        exit_code = 0
        stop_on_first_exit = not args.no_show
        while processes:
            for name, process in list(processes.items()):
                return_code = process.poll()
                if return_code is None:
                    continue
                print(f"{name} exited with code {return_code}.")
                processes.pop(name)
                if return_code != 0:
                    exit_code = return_code
                    stop_processes(processes)
                    raise SystemExit(exit_code)
                if stop_on_first_exit:
                    stop_processes(processes)
                    raise SystemExit(exit_code)
            time.sleep(0.1)
        raise SystemExit(exit_code)
    except KeyboardInterrupt:
        print("\nStopping comparison.")
        stop_processes(processes)
        raise SystemExit(130)


if __name__ == "__main__":
    main()
