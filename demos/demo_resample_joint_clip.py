"""Resample a recorded joint clip onto a uniform frame rate.

The live recorder captures frames as fast as fresh packets arrive, so the real
rate is uneven (e.g. ~45 fps with occasional Wi-Fi stalls). For ML training you
usually want a clean, uniformly-spaced timeline. This post-processing step reads
a recorded .npz, interpolates every per-frame array onto an even grid at the
target fps, and writes a new .npz with the same schema.

Joint/root positions are interpolated linearly; the root orientation quaternion
is interpolated with SLERP. Because resampling uses each output time's
neighbours, it both regularises the spacing and smooths over short stalls. It
does NOT extrapolate past the clip, so output times are clamped to the captured
span.

Usage (from project root):
  conda run --no-capture-output -n humanoid-sim python demos/demo_resample_joint_clip.py IN.npz
  conda run --no-capture-output -n humanoid-sim python demos/demo_resample_joint_clip.py IN.npz --fps 50 --output OUT.npz
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


# Per-frame arrays interpolated linearly (first axis is time).
_LINEAR_KEYS = ("joint_pos_origin", "joint_pos_w", "root_pos_w")
# Per-frame array interpolated with SLERP (wxyz quaternion).
_QUAT_KEY = "root_quat_wxyz"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("clip", type=Path, help="Input .npz recorded clip")
    parser.add_argument(
        "--fps", type=float, default=50.0,
        help="Target uniform frame rate (default: 50).",
    )
    parser.add_argument(
        "--output", type=Path, default=None,
        help="Output path (default: <input>_<fps>fps.npz next to the input).",
    )
    return parser.parse_args()


def _uniform_times(timestamps: np.ndarray, fps: float) -> np.ndarray:
    """Even time grid spanning the clip at the target fps (clamped, no extrap)."""
    t0 = float(timestamps[0])
    t1 = float(timestamps[-1])
    span = t1 - t0
    n_out = int(np.floor(span * fps)) + 1
    return t0 + np.arange(n_out) / fps


def resample_clip(data: dict, fps: float) -> dict:
    ts = np.asarray(data["timestamps_s"], dtype=float).reshape(-1)
    n = ts.shape[0]
    if n < 2:
        raise SystemExit(f"clip has only {n} frame(s); nothing to resample")
    if not np.all(np.diff(ts) > 0):
        # Interpolation needs strictly increasing time. Drop any non-increasing
        # samples (can happen if two frames share a timestamp).
        keep = np.concatenate([[True], np.diff(ts) > 0])
        ts = ts[keep]
        data = {k: (v[keep] if _is_per_frame(k, v, n) else v) for k, v in data.items()}
        n = ts.shape[0]

    new_ts = _uniform_times(ts, fps)
    out: dict = {}

    for key, value in data.items():
        if key == "timestamps_s":
            out[key] = new_ts
            continue
        if key == "fps":
            out[key] = np.array([fps], dtype=float)
            continue
        if key in _LINEAR_KEYS:
            out[key] = _interp_linear(ts, np.asarray(value, dtype=float), new_ts)
        elif key == _QUAT_KEY:
            out[key] = _interp_quat_wxyz(ts, np.asarray(value, dtype=float), new_ts)
        else:
            # Metadata / scalars / per-frame arrays we don't touch: pass through.
            out[key] = value
    return out


def _is_per_frame(key: str, value, n: int) -> bool:
    return key in _LINEAR_KEYS or key == _QUAT_KEY or (
        hasattr(value, "shape") and value.shape and value.shape[0] == n and key != "fps"
    )


def _interp_linear(ts: np.ndarray, arr: np.ndarray, new_ts: np.ndarray) -> np.ndarray:
    """Linear interpolation of a (frames, ...) array onto new_ts."""
    flat = arr.reshape(arr.shape[0], -1)
    out = np.empty((new_ts.shape[0], flat.shape[1]), dtype=float)
    for c in range(flat.shape[1]):
        out[:, c] = np.interp(new_ts, ts, flat[:, c])
    return out.reshape((new_ts.shape[0],) + arr.shape[1:])


def _interp_quat_wxyz(ts: np.ndarray, quat_wxyz: np.ndarray, new_ts: np.ndarray) -> np.ndarray:
    """SLERP interpolation of a (frames, 4) wxyz quaternion array onto new_ts."""
    # scipy uses xyzw order.
    xyzw = quat_wxyz[:, [1, 2, 3, 0]]
    slerp = Slerp(ts, Rotation.from_quat(xyzw))
    out_xyzw = slerp(new_ts).as_quat()
    return out_xyzw[:, [3, 0, 1, 2]]  # back to wxyz


def main() -> None:
    args = parse_args()
    if not args.clip.exists():
        raise SystemExit(f"clip not found: {args.clip}")

    data = dict(np.load(args.clip, allow_pickle=True))
    ts = np.asarray(data["timestamps_s"], dtype=float).reshape(-1)
    n_in = ts.shape[0]
    span = float(ts[-1] - ts[0]) if n_in > 1 else 0.0
    actual_in = (n_in - 1) / span if span > 0 else 0.0

    out = resample_clip(data, args.fps)
    n_out = out["timestamps_s"].shape[0]

    output = args.output or args.clip.with_name(
        f"{args.clip.stem}_{int(round(args.fps))}fps{args.clip.suffix}"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez(output, **out)

    print(f"Resampled clip")
    print(f"  in : {args.clip}")
    print(f"       {n_in} frames, {actual_in:.1f} fps actual, {span:.2f} s")
    print(f"  out: {output}")
    print(f"       {n_out} frames, {args.fps:.0f} fps uniform, {span:.2f} s")


if __name__ == "__main__":
    main()
