from __future__ import annotations

import argparse
import warnings
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


ROBOT_QPOS_DIMS = {
    "ch_robot": 17,
}

ROBOT_DOF = {
    "ch_robot": 10,
}


@dataclass(frozen=True)
class VelocitySummary:
    max_joint_vel: float
    max_root_lin_vel_w: float
    max_root_ang_vel_w: float
    max_yaw_rate_ref: float
    nonfinite_count: int


def normalize_quaternions(quat: np.ndarray, *, warn_threshold: float = 1e-3) -> np.ndarray:
    """Normalize quaternions and warn when input norms are suspicious."""
    quat = np.asarray(quat, dtype=np.float64)
    norms = np.linalg.norm(quat, axis=-1, keepdims=True)
    if np.any(norms <= 1e-12):
        raise ValueError("Root quaternion contains near-zero norm entries.")

    max_deviation = float(np.max(np.abs(norms - 1.0)))
    if max_deviation > warn_threshold:
        warnings.warn(
            f"Root quaternion norms deviate from 1.0 by up to {max_deviation:.6g}; normalizing before use.",
            RuntimeWarning,
            stacklevel=2,
        )
    return quat / norms


def enforce_quaternion_sign_continuity(quat: np.ndarray) -> np.ndarray:
    """Flip q/-q signs so adjacent quaternions remain on the same hemisphere."""
    continuous = np.array(quat, dtype=np.float64, copy=True)
    for i in range(1, continuous.shape[0]):
        if float(np.dot(continuous[i], continuous[i - 1])) < 0.0:
            continuous[i] = -continuous[i]
    return continuous


def quat_conjugate(quat: np.ndarray) -> np.ndarray:
    result = np.array(quat, copy=True)
    result[..., 1:] *= -1.0
    return result


def quat_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = np.moveaxis(a, -1, 0)
    bw, bx, by, bz = np.moveaxis(b, -1, 0)
    return np.stack(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        axis=-1,
    )


def quat_to_rotvec(quat: np.ndarray, *, eps: float = 1e-8) -> np.ndarray:
    quat = normalize_quaternions(quat)
    quat = np.where(quat[..., :1] < 0.0, -quat, quat)
    w = np.clip(quat[..., 0], -1.0, 1.0)
    angle = 2.0 * np.arccos(w)
    sin_half_angle = np.sqrt(np.maximum(1.0 - w * w, 0.0))
    axis = np.zeros_like(quat[..., 1:])
    valid = sin_half_angle > eps
    axis[valid] = quat[..., 1:][valid] / sin_half_angle[valid, None]
    return axis * angle[..., None]


def rotate_inverse(quat: np.ndarray, vec: np.ndarray) -> np.ndarray:
    """Rotate world-frame vectors into the local/body frame using wxyz quaternions."""
    quat = normalize_quaternions(quat)
    w = quat[..., :1]
    q_xyz = -quat[..., 1:]
    t = 2.0 * np.cross(q_xyz, vec)
    return vec + w * t + np.cross(q_xyz, t)


def yaw_from_quat_wxyz(quat: np.ndarray) -> np.ndarray:
    quat = normalize_quaternions(quat)
    w, x, y, z = np.moveaxis(quat, -1, 0)
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def finite_difference(values: np.ndarray, dt: float) -> np.ndarray:
    values = np.asarray(values, dtype=np.float64)
    if values.shape[0] == 1:
        return np.zeros_like(values)
    if values.shape[0] == 2:
        diff = (values[1] - values[0]) / dt
        return np.stack([diff, diff], axis=0)
    return np.gradient(values, dt, axis=0)


def angular_velocity_world_from_quat(quat: np.ndarray, dt: float) -> np.ndarray:
    """Compute world-frame angular velocity from body-to-world quaternions."""
    quat = enforce_quaternion_sign_continuity(normalize_quaternions(quat))
    num_frames = quat.shape[0]

    if num_frames == 1:
        return np.zeros((1, 3), dtype=np.float64)

    if num_frames == 2:
        q_delta = quat_multiply(quat[1:2], quat_conjugate(quat[0:1]))
        omega = quat_to_rotvec(q_delta) / dt
        return np.repeat(omega, 2, axis=0)

    ang_vel = np.zeros((num_frames, 3), dtype=np.float64)
    q_delta = quat_multiply(quat[2:], quat_conjugate(quat[:-2]))
    ang_vel[1:-1] = quat_to_rotvec(q_delta) / (2.0 * dt)
    ang_vel[0] = ang_vel[1]
    ang_vel[-1] = ang_vel[-2]
    return ang_vel


def compute_retargeted_velocities(
    qpos: np.ndarray,
    fps: float,
    *,
    robot: str = "ch_robot",
    generated_dtype: np.dtype[Any] | type[np.floating[Any]] = np.float32,
) -> dict[str, np.ndarray]:
    if robot not in ROBOT_QPOS_DIMS:
        raise ValueError(f"Unsupported robot {robot!r}. Supported robots: {sorted(ROBOT_QPOS_DIMS)}")
    if qpos.ndim != 2:
        raise ValueError(f"Expected qpos to be 2D, got shape {qpos.shape}.")
    expected_qpos_dim = ROBOT_QPOS_DIMS[robot]
    if qpos.shape[1] != expected_qpos_dim:
        raise ValueError(f"Expected {robot} qpos shape (T, {expected_qpos_dim}), got {qpos.shape}.")
    if qpos.shape[0] < 1:
        raise ValueError("qpos must contain at least one frame.")
    if fps <= 0.0 or not np.isfinite(fps):
        raise ValueError(f"fps must be a positive finite value, got {fps}.")

    dt = 1.0 / float(fps)
    root_pos = np.asarray(qpos[:, :3], dtype=np.float64)
    root_quat = enforce_quaternion_sign_continuity(normalize_quaternions(qpos[:, 3:7]))
    joint_pos = np.asarray(qpos[:, 7 : 7 + ROBOT_DOF[robot]], dtype=np.float64)

    root_lin_vel_w = finite_difference(root_pos, dt)
    joint_vel = finite_difference(joint_pos, dt)
    root_ang_vel_w = angular_velocity_world_from_quat(root_quat, dt)
    root_lin_vel_b = rotate_inverse(root_quat, root_lin_vel_w)
    root_ang_vel_b = rotate_inverse(root_quat, root_ang_vel_w)

    root_yaw = np.unwrap(yaw_from_quat_wxyz(root_quat))
    yaw_rate_ref = finite_difference(root_yaw, dt)

    qvel = np.concatenate([root_lin_vel_w, root_ang_vel_b, joint_vel], axis=1)

    dtype = np.dtype(generated_dtype)
    return {
        "qvel": qvel.astype(dtype, copy=False),
        "joint_vel": joint_vel.astype(dtype, copy=False),
        "root_lin_vel_w": root_lin_vel_w.astype(dtype, copy=False),
        "root_lin_vel_b": root_lin_vel_b.astype(dtype, copy=False),
        "root_ang_vel_w": root_ang_vel_w.astype(dtype, copy=False),
        "root_ang_vel_b": root_ang_vel_b.astype(dtype, copy=False),
        "root_yaw": root_yaw.astype(dtype, copy=False),
        "yaw_rate_ref": yaw_rate_ref.astype(dtype, copy=False),
    }


def summarize_velocities(arrays: dict[str, np.ndarray]) -> VelocitySummary:
    generated = [
        arrays["qvel"],
        arrays["joint_vel"],
        arrays["root_lin_vel_w"],
        arrays["root_lin_vel_b"],
        arrays["root_ang_vel_w"],
        arrays["root_ang_vel_b"],
        arrays["root_yaw"],
        arrays["yaw_rate_ref"],
    ]
    nonfinite_count = int(sum(np.size(arr) - np.count_nonzero(np.isfinite(arr)) for arr in generated))
    return VelocitySummary(
        max_joint_vel=float(np.max(np.abs(arrays["joint_vel"]))),
        max_root_lin_vel_w=float(np.max(np.abs(arrays["root_lin_vel_w"]))),
        max_root_ang_vel_w=float(np.max(np.abs(arrays["root_ang_vel_w"]))),
        max_yaw_rate_ref=float(np.max(np.abs(arrays["yaw_rate_ref"]))),
        nonfinite_count=nonfinite_count,
    )


def read_fps(data: np.lib.npyio.NpzFile, fallback_fps: float | None) -> float:
    if "fps" in data:
        fps_value = float(np.asarray(data["fps"]).reshape(-1)[0])
    elif fallback_fps is not None:
        fps_value = float(fallback_fps)
    else:
        raise ValueError("Input file does not contain 'fps'. Pass --fps to provide it explicitly.")
    if fps_value <= 0.0 or not np.isfinite(fps_value):
        raise ValueError(f"fps must be a positive finite value, got {fps_value}.")
    return fps_value


def enrich_file(
    input_path: Path,
    output_path: Path,
    *,
    robot: str,
    fallback_fps: float | None,
    overwrite: bool,
    cast_qpos_float32: bool,
) -> VelocitySummary:
    if output_path.exists() and not overwrite:
        raise FileExistsError(f"Output file already exists: {output_path}. Pass --overwrite to replace it.")

    with np.load(input_path, allow_pickle=True) as data:
        if "qpos" not in data:
            raise ValueError(f"Input file {input_path} does not contain required key 'qpos'.")

        fps = read_fps(data, fallback_fps)
        qpos = np.asarray(data["qpos"])
        if cast_qpos_float32:
            qpos = qpos.astype(np.float32)

        output = {key: data[key] for key in data.files}
        output["qpos"] = qpos
        output["fps"] = np.asarray(data["fps"] if "fps" in data else fps)

    generated = compute_retargeted_velocities(qpos, fps, robot=robot, generated_dtype=np.float32)
    output.update(generated)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(output_path, **output)
    return summarize_velocities(generated)


def resolve_jobs(input_path: Path, output_path: Path | None, *, recursive: bool) -> list[tuple[Path, Path]]:
    if input_path.is_file():
        if output_path is None:
            output = input_path.with_name(f"{input_path.stem}_with_vel.npz")
        elif output_path.suffix == ".npz":
            output = output_path
        else:
            output = output_path / f"{input_path.stem}_with_vel.npz"
        return [(input_path, output)]

    if not input_path.is_dir():
        raise FileNotFoundError(f"Input path does not exist: {input_path}")

    pattern = "**/*.npz" if recursive else "*.npz"
    files = sorted(input_path.glob(pattern))
    if not files:
        raise FileNotFoundError(f"No .npz files found in {input_path} with recursive={recursive}.")

    output_dir = output_path or input_path
    if output_dir.suffix == ".npz":
        raise ValueError("--output must be a directory when --input is a directory.")

    jobs = []
    for src in files:
        rel = src.relative_to(input_path)
        dst_rel = rel.with_name(f"{rel.stem}_with_vel.npz")
        jobs.append((src, output_dir / dst_rel))
    return jobs


def print_summary(input_path: Path, output_path: Path, summary: VelocitySummary) -> None:
    print(f"[OK] {input_path} -> {output_path}")
    print(f"  max |joint_vel|      : {summary.max_joint_vel:.6g}")
    print(f"  max |root_lin_vel_w| : {summary.max_root_lin_vel_w:.6g}")
    print(f"  max |root_ang_vel_w| : {summary.max_root_ang_vel_w:.6g}")
    print(f"  max |yaw_rate_ref|   : {summary.max_yaw_rate_ref:.6g}")
    print(f"  num NaN / Inf        : {summary.nonfinite_count}")


def parse_bool(value: str | bool) -> bool:
    if isinstance(value, bool):
        return value
    normalized = value.strip().lower()
    if normalized in {"1", "true", "t", "yes", "y", "on"}:
        return True
    if normalized in {"0", "false", "f", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"Expected a boolean value, got {value!r}.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Add reference velocities to raw retargeted qpos .npz files.")
    parser.add_argument("--input", required=True, type=Path, help="Input .npz file or directory of .npz files.")
    parser.add_argument("--output", type=Path, default=None, help="Output .npz file or directory.")
    parser.add_argument("--robot", default="ch_robot", choices=sorted(ROBOT_QPOS_DIMS), help="Robot type.")
    parser.add_argument(
        "--recursive",
        nargs="?",
        const=True,
        default=False,
        type=parse_bool,
        help="Process directories recursively. Accepts bare flag, true, or false.",
    )
    parser.add_argument(
        "--overwrite",
        nargs="?",
        const=True,
        default=False,
        type=parse_bool,
        help="Overwrite existing output files. Accepts bare flag, true, or false.",
    )
    parser.add_argument("--fps", type=float, default=None, help="FPS to use when the input file does not store fps.")
    parser.add_argument(
        "--cast-qpos-float32",
        nargs="?",
        const=True,
        default=False,
        type=parse_bool,
        help="Write qpos as float32 instead of preserving the input dtype.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    jobs = resolve_jobs(args.input, args.output, recursive=args.recursive)
    for src, dst in jobs:
        summary = enrich_file(
            src,
            dst,
            robot=args.robot,
            fallback_fps=args.fps,
            overwrite=args.overwrite,
            cast_qpos_float32=args.cast_qpos_float32,
        )
        print_summary(src, dst, summary)


if __name__ == "__main__":
    main()
