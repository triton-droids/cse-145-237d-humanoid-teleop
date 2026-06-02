from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

src_root = Path(__file__).resolve().parents[2]
if str(src_root) not in sys.path:
    sys.path.insert(0, str(src_root))

from holosoma_retargeting.data_conversion.add_retargeted_velocities import parse_bool


DEFAULT_CONTACT_BODIES = (
    "left_foot_sphere_1_link",
    "left_foot_sphere_2_link",
    "left_foot_sphere_3_link",
    "left_foot_sphere_4_link",
    "right_foot_sphere_1_link",
    "right_foot_sphere_2_link",
    "right_foot_sphere_3_link",
    "right_foot_sphere_4_link",
)

DEFAULT_CONTACT_GEOMS = (
    "left_foot_collision_box",
    "right_foot_collision_box",
)


@dataclass(frozen=True)
class FloorNormalizationSummary:
    min_height_before: float
    min_height_after: float
    root_z_shift: float
    num_frames: int


def resolve_model_path(robot: str, model_path: Path | None) -> Path:
    if model_path is not None:
        return model_path
    if robot != "ch_robot":
        raise ValueError(f"Unsupported robot {robot!r}; pass --model-path explicitly.")
    return Path(__file__).resolve().parents[1] / "models" / "ch_robot" / "ch_robot_10dof.xml"


def contact_body_ids(model: mujoco.MjModel, body_names: tuple[str, ...] | list[str]) -> list[int]:
    ids = []
    missing = []
    for name in body_names:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id < 0:
            missing.append(name)
        else:
            ids.append(body_id)
    if missing:
        raise ValueError(f"Contact bodies not found in model: {missing}")
    return ids


def contact_geom_ids(model: mujoco.MjModel, geom_names: tuple[str, ...] | list[str]) -> list[int]:
    ids = []
    missing = []
    for name in geom_names:
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if geom_id < 0:
            missing.append(name)
        else:
            ids.append(geom_id)
    if missing:
        raise ValueError(f"Contact geoms not found in model: {missing}")
    return ids


def contact_heights_for_qpos(model: mujoco.MjModel, qpos: np.ndarray, body_ids: list[int]) -> np.ndarray:
    data = mujoco.MjData(model)
    heights = np.zeros((qpos.shape[0], len(body_ids)), dtype=np.float64)
    for i, q in enumerate(qpos):
        data.qpos[:] = q
        mujoco.mj_forward(model, data)
        heights[i] = data.xpos[body_ids, 2]
    return heights


def box_geom_bottom_z(model: mujoco.MjModel, data: mujoco.MjData, geom_id: int) -> float:
    half_extents = model.geom_size[geom_id]
    xmat = data.geom_xmat[geom_id].reshape(3, 3)
    vertical_half_extent = float(np.sum(np.abs(xmat[2, :]) * half_extents))
    return float(data.geom_xpos[geom_id, 2] - vertical_half_extent)


def geom_bottom_heights_for_qpos(model: mujoco.MjModel, qpos: np.ndarray, geom_ids: list[int]) -> np.ndarray:
    data = mujoco.MjData(model)
    heights = np.zeros((qpos.shape[0], len(geom_ids)), dtype=np.float64)
    for i, q in enumerate(qpos):
        data.qpos[:] = q
        mujoco.mj_forward(model, data)
        for j, geom_id in enumerate(geom_ids):
            if model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_BOX:
                heights[i, j] = box_geom_bottom_z(model, data, geom_id)
            else:
                heights[i, j] = data.geom_xpos[geom_id, 2] - model.geom_size[geom_id, 0]
    return heights


def normalize_qpos_floor(
    qpos: np.ndarray,
    *,
    model: mujoco.MjModel,
    body_ids: list[int] | None = None,
    geom_ids: list[int] | None = None,
    target_floor_z: float = 0.0,
    contact_radius: float = 0.005,
) -> tuple[np.ndarray, FloorNormalizationSummary]:
    if qpos.ndim != 2:
        raise ValueError(f"Expected qpos to be 2D, got shape {qpos.shape}.")
    if qpos.shape[0] < 1:
        raise ValueError("qpos must contain at least one frame.")
    if qpos.shape[1] != model.nq:
        raise ValueError(f"Expected qpos shape (T, {model.nq}) for model nq, got {qpos.shape}.")

    if geom_ids is not None:
        heights_before = geom_bottom_heights_for_qpos(model, qpos, geom_ids)
        min_contact_before = float(np.min(heights_before))
    elif body_ids is not None:
        heights_before = contact_heights_for_qpos(model, qpos, body_ids)
        min_center_before = float(np.min(heights_before))
        min_contact_before = min_center_before - float(contact_radius)
    else:
        raise ValueError("Either body_ids or geom_ids must be provided.")

    root_z_shift = float(target_floor_z) - min_contact_before

    normalized = np.array(qpos, copy=True)
    normalized[:, 2] += root_z_shift

    if geom_ids is not None:
        heights_after = geom_bottom_heights_for_qpos(model, normalized, geom_ids)
        min_contact_after = float(np.min(heights_after))
    else:
        heights_after = contact_heights_for_qpos(model, normalized, body_ids or [])
        min_contact_after = float(np.min(heights_after)) - float(contact_radius)

    return normalized, FloorNormalizationSummary(
        min_height_before=min_contact_before,
        min_height_after=min_contact_after,
        root_z_shift=root_z_shift,
        num_frames=int(qpos.shape[0]),
    )


def output_path_for(input_path: Path, output: Path | None) -> Path:
    if output is None:
        return input_path.with_name(f"{input_path.stem}_floor_norm.npz")
    if output.suffix == ".npz":
        return output
    return output / f"{input_path.stem}_floor_norm.npz"


def resolve_jobs(input_path: Path, output_path: Path | None, *, recursive: bool) -> list[tuple[Path, Path]]:
    if input_path.is_file():
        return [(input_path, output_path_for(input_path, output_path))]
    if not input_path.is_dir():
        raise FileNotFoundError(f"Input path does not exist: {input_path}")
    pattern = "**/*.npz" if recursive else "*.npz"
    files = sorted(input_path.glob(pattern))
    if not files:
        raise FileNotFoundError(f"No .npz files found in {input_path} with recursive={recursive}.")
    if output_path is not None and output_path.suffix == ".npz":
        raise ValueError("--output must be a directory when --input is a directory.")
    output_dir = output_path or input_path
    return [(src, output_dir / src.relative_to(input_path).with_name(f"{src.stem}_floor_norm.npz")) for src in files]


def normalize_file(
    input_path: Path,
    output_path: Path,
    *,
    model: mujoco.MjModel,
    body_ids: list[int] | None,
    geom_ids: list[int] | None,
    target_floor_z: float,
    contact_radius: float,
    overwrite: bool,
) -> FloorNormalizationSummary:
    if output_path.exists() and not overwrite:
        raise FileExistsError(f"Output file already exists: {output_path}. Pass --overwrite to replace it.")
    with np.load(input_path, allow_pickle=True) as data:
        if "qpos" not in data:
            raise ValueError(f"Input file {input_path} does not contain required key 'qpos'.")
        qpos = np.asarray(data["qpos"])
        output: dict[str, Any] = {key: data[key] for key in data.files}

    qpos_normalized, summary = normalize_qpos_floor(
        qpos,
        model=model,
        body_ids=body_ids,
        geom_ids=geom_ids,
        target_floor_z=target_floor_z,
        contact_radius=contact_radius,
    )
    output["qpos"] = qpos_normalized
    output["floor_normalization_root_z_shift"] = np.asarray(summary.root_z_shift, dtype=np.float64)
    output["floor_normalization_min_height_before"] = np.asarray(summary.min_height_before, dtype=np.float64)
    output["floor_normalization_min_height_after"] = np.asarray(summary.min_height_after, dtype=np.float64)
    output["floor_normalization_contact_radius"] = np.asarray(contact_radius, dtype=np.float64)
    output["floor_normalization_target_floor_z"] = np.asarray(target_floor_z, dtype=np.float64)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(output_path, **output)
    return summary


def print_summary(input_path: Path, output_path: Path, summary: FloorNormalizationSummary) -> None:
    print(f"[OK] {input_path} -> {output_path}")
    print(f"  frames             : {summary.num_frames}")
    print(f"  min contact before : {summary.min_height_before:.6g}")
    print(f"  root z shift       : {summary.root_z_shift:.6g}")
    print(f"  min contact after  : {summary.min_height_after:.6g}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Shift retargeted qpos root z so foot contact spheres touch the floor.")
    parser.add_argument("--input", required=True, type=Path, help="Input .npz file or directory.")
    parser.add_argument("--output", type=Path, default=None, help="Output .npz file or directory.")
    parser.add_argument("--robot", default="ch_robot", help="Robot type.")
    parser.add_argument("--model-path", type=Path, default=None, help="MuJoCo XML model path.")
    parser.add_argument(
        "--target",
        choices=("foot_geoms", "foot_spheres"),
        default="foot_geoms",
        help="Geometry used to define the floor contact point.",
    )
    parser.add_argument("--target-floor-z", type=float, default=0.0, help="Desired bottom contact height after shift.")
    parser.add_argument("--contact-radius", type=float, default=0.005, help="Foot contact sphere radius.")
    parser.add_argument("--recursive", nargs="?", const=True, default=False, type=parse_bool)
    parser.add_argument("--overwrite", nargs="?", const=True, default=False, type=parse_bool)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    model_path = resolve_model_path(args.robot, args.model_path)
    model = mujoco.MjModel.from_xml_path(str(model_path))
    body_ids = contact_body_ids(model, DEFAULT_CONTACT_BODIES) if args.target == "foot_spheres" else None
    geom_ids = contact_geom_ids(model, DEFAULT_CONTACT_GEOMS) if args.target == "foot_geoms" else None
    jobs = resolve_jobs(args.input, args.output, recursive=args.recursive)
    for src, dst in jobs:
        summary = normalize_file(
            src,
            dst,
            model=model,
            body_ids=body_ids,
            geom_ids=geom_ids,
            target_floor_z=args.target_floor_z,
            contact_radius=args.contact_radius,
            overwrite=args.overwrite,
        )
        print_summary(src, dst, summary)


if __name__ == "__main__":
    main()
