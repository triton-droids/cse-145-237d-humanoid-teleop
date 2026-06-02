"""Replay IMU-derived ch_robot qpos on the real Holosoma ch_robot MuJoCo model.

The script accepts either:

- a recorded ``human_joint_clip_*.npz`` IMU handoff clip, which is converted to
  ch_robot qpos/qvel first, or
- a saved ch_robot replay ``.npz`` containing ``qpos``.

The ch_robot MJCF and meshes are loaded from the local ``retargeting_holosoma``
branch reference into ``wearable_imu/.cache/ch_robot_model`` on first run.

Usage:
  python demos/demo_mujoco_ch_robot_replay.py ../data/human_joint_clip_20260601_231345.npz
  python demos/demo_mujoco_ch_robot_replay.py ../data/ch_robot_replay_qpos_20260601_231345.npz
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PROJECT_ROOT.parent
for path in (PROJECT_ROOT, REPO_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from ik.ch_robot_retarget import (  # noqa: E402
    CH_ROBOT_JOINT_NAMES,
    QPOS_WIDTH,
    human_joint_clip_to_qpos_qvel,
    load_human_joint_source,
)


BRANCH_REF = "origin/retargeting_holosoma"
CH_ROBOT_BRANCH_PATH = "holosoma-main/src/holosoma_retargeting/holosoma_retargeting/models/ch_robot"
DEFAULT_MODEL_CACHE = PROJECT_ROOT / ".cache" / "ch_robot_model"
FLOOR_TEXTURE_NAME = "floor_grid_texture"
FLOOR_MATERIAL_NAME = "floor_grid"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="human_joint_clip_*.npz, camera .jsonl, or ch_robot qpos replay .npz.")
    parser.add_argument("--model-dir", type=Path, default=DEFAULT_MODEL_CACHE)
    parser.add_argument("--refresh-model", action="store_true", help="Re-extract ch_robot assets from retargeting_holosoma.")
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--frame-key", choices=("joint_pos_origin", "joint_pos_w"), default="joint_pos_origin")
    parser.add_argument("--yaw-mode", choices=("keep", "strip"), default="keep")
    parser.add_argument("--base-height", type=float, default=0.765)
    parser.add_argument("--loop", action="store_true", default=True)
    parser.add_argument("--no-loop", dest="loop", action="store_false")
    parser.add_argument("--no-show", action="store_true", help="Load model/input and print a summary without opening viewer.")
    return parser.parse_args()


def relaunch_with_mjpython_if_needed(args: argparse.Namespace) -> None:
    """On macOS, MuJoCo passive viewer must be launched by ``mjpython``."""

    if args.no_show or sys.platform != "darwin":
        return
    if Path(sys.executable).name == "mjpython":
        return
    if os.environ.get("CH_ROBOT_REPLAY_MJPYTHON") == "1":
        return

    mjpython = shutil.which("mjpython")
    if mjpython is None:
        raise RuntimeError(
            "MuJoCo viewer on macOS requires mjpython, but mjpython was not found "
            "on PATH. Try: mjpython demos/demo_mujoco_ch_robot_replay.py <input.npz>"
        )

    env = os.environ.copy()
    env["CH_ROBOT_REPLAY_MJPYTHON"] = "1"
    print(f"macOS MuJoCo viewer requires mjpython; relaunching with {mjpython}")
    os.execvpe(mjpython, [mjpython, *sys.argv], env)


def ensure_ch_robot_model(model_dir: Path, *, refresh: bool = False) -> Path:
    """Return local ch_robot MJCF path, extracting it from retargeting_holosoma if needed."""

    xml_path = model_dir / "ch_robot_10dof.xml"
    mesh_dir = model_dir / "meshes"
    if not refresh and xml_path.exists() and mesh_dir.exists():
        ensure_visible_floor(xml_path)
        return xml_path

    if refresh and model_dir.exists():
        shutil.rmtree(model_dir)
    model_dir.parent.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory() as tmp:
        archive_path = Path(tmp) / "ch_robot.tar"
        with archive_path.open("wb") as archive_file:
            subprocess.run(
                ["git", "archive", BRANCH_REF, CH_ROBOT_BRANCH_PATH],
                cwd=REPO_ROOT,
                stdout=archive_file,
                check=True,
            )
        with tarfile.open(archive_path) as archive:
            archive.extractall(tmp)

        extracted = Path(tmp) / CH_ROBOT_BRANCH_PATH
        if not extracted.exists():
            raise RuntimeError(f"failed to extract {CH_ROBOT_BRANCH_PATH} from {BRANCH_REF}")
        if model_dir.exists():
            shutil.rmtree(model_dir)
        shutil.copytree(extracted, model_dir)

    if not xml_path.exists():
        raise RuntimeError(f"missing ch_robot MJCF after extraction: {xml_path}")
    ensure_visible_floor(xml_path)
    return xml_path


def ensure_visible_floor(xml_path: Path) -> None:
    """Ensure the cached ch_robot MJCF has a visible checker floor plane."""

    tree = ET.parse(xml_path)
    root = tree.getroot()

    asset = root.find("asset")
    if asset is None:
        asset = ET.Element("asset")
        worldbody = root.find("worldbody")
        insert_index = list(root).index(worldbody) if worldbody is not None else len(root)
        root.insert(insert_index, asset)

    texture = _find_named(asset, "texture", FLOOR_TEXTURE_NAME)
    if texture is None:
        texture = ET.SubElement(asset, "texture")
    texture.attrib.update(
        {
            "name": FLOOR_TEXTURE_NAME,
            "type": "2d",
            "builtin": "checker",
            "rgb1": "0.18 0.20 0.22",
            "rgb2": "0.32 0.34 0.36",
            "width": "512",
            "height": "512",
        }
    )

    material = _find_named(asset, "material", FLOOR_MATERIAL_NAME)
    if material is None:
        material = ET.SubElement(asset, "material")
    material.attrib.update(
        {
            "name": FLOOR_MATERIAL_NAME,
            "texture": FLOOR_TEXTURE_NAME,
            "texrepeat": "8 8",
            "reflectance": "0.12",
        }
    )

    worldbody = root.find("worldbody")
    if worldbody is None:
        worldbody = ET.SubElement(root, "worldbody")

    ground = _find_named(worldbody, "geom", "ground")
    if ground is None:
        ground = ET.SubElement(worldbody, "geom")
    ground.attrib.update(
        {
            "name": "ground",
            "type": "plane",
            "size": "10 10 0.1",
            "pos": "0 0 0",
            "material": FLOOR_MATERIAL_NAME,
        }
    )

    ET.indent(tree, space="  ")
    tree.write(xml_path, encoding="utf-8", xml_declaration=False)


def _find_named(parent: ET.Element, tag: str, name: str) -> ET.Element | None:
    for child in parent.findall(tag):
        if child.attrib.get("name") == name:
            return child
    return None


def load_replay_input(path: Path, *, frame_key: str, yaw_mode: str, base_height: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, float, str]:
    if path.suffix == ".npz":
        data = np.load(path, allow_pickle=True)
    else:
        data = None

    if data is not None and "qpos" in data.files:
        qpos = np.asarray(data["qpos"], dtype=np.float64)
        if qpos.ndim != 2 or qpos.shape[1] != QPOS_WIDTH:
            raise ValueError(f"qpos must have shape (frames, {QPOS_WIDTH}), got {qpos.shape}")
        if "qvel" in data.files:
            qvel = np.asarray(data["qvel"], dtype=np.float64)
        else:
            qvel = np.zeros((qpos.shape[0], 16), dtype=np.float64)
        fps = float(np.asarray(data["fps"]).reshape(-1)[0]) if "fps" in data.files else 50.0
        timestamps = _timestamps_from_data(data, qpos.shape[0], fps)
        return qpos, qvel, timestamps, fps, "ch_robot_qpos"

    clip = load_human_joint_source(path, frame_key=frame_key)
    qpos, qvel = human_joint_clip_to_qpos_qvel(
        clip,
        base_height=base_height,
        yaw_mode=yaw_mode,
    )
    return qpos, qvel, clip.timestamps_s, clip.fps, clip.frame_key


def _timestamps_from_data(data: np.lib.npyio.NpzFile, n_frames: int, fps: float) -> np.ndarray:
    if "timestamps_s" in data.files:
        timestamps = np.asarray(data["timestamps_s"], dtype=np.float64).reshape(-1)
        if timestamps.shape == (n_frames,):
            return timestamps
    step = 1.0 / fps if fps > 0.0 else 1.0 / 50.0
    return np.arange(n_frames, dtype=np.float64) * step


def print_summary(
    *,
    input_path: Path,
    source: str,
    xml_path: Path,
    qpos: np.ndarray,
    qvel: np.ndarray,
    fps: float,
    model: mujoco.MjModel,
) -> None:
    print(f"Input      : {input_path}")
    print(f"source     : {source}")
    print(f"model xml  : {xml_path}")
    print(f"frames     : {qpos.shape[0]}")
    print(f"fps        : {fps:.1f}")
    print(f"qpos       : {qpos.shape}")
    print(f"qvel       : {qvel.shape}")
    print(f"MuJoCo nq  : {model.nq}")
    print(f"MuJoCo nu  : {model.nu}")
    print("joint order: " + ", ".join(CH_ROBOT_JOINT_NAMES))


def main() -> None:
    args = parse_args()
    relaunch_with_mjpython_if_needed(args)
    xml_path = ensure_ch_robot_model(args.model_dir, refresh=args.refresh_model)
    qpos, qvel, timestamps, fps, source = load_replay_input(
        args.input,
        frame_key=args.frame_key,
        yaw_mode=args.yaw_mode,
        base_height=args.base_height,
    )

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    if model.nq != QPOS_WIDTH:
        raise RuntimeError(f"expected ch_robot nq={QPOS_WIDTH}, got {model.nq}")
    if model.nu != len(CH_ROBOT_JOINT_NAMES):
        raise RuntimeError(f"expected ch_robot nu={len(CH_ROBOT_JOINT_NAMES)}, got {model.nu}")

    print_summary(
        input_path=args.input,
        source=source,
        xml_path=xml_path,
        qpos=qpos,
        qvel=qvel,
        fps=fps,
        model=model,
    )
    data.qpos[:] = qpos[0]
    data.ctrl[:] = qpos[0, 7:]
    mujoco.mj_forward(model, data)

    if args.no_show:
        return

    from mujoco import viewer as mujoco_viewer

    timestamps = timestamps - timestamps[0]
    speed = max(args.speed, 1e-3)
    frame_index = 0
    wall_anchor = time.monotonic()
    clip_anchor = float(timestamps[0])

    print("\nMuJoCo viewer running. Close the viewer to exit.\n")
    with mujoco_viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            now = time.monotonic()
            target_t = clip_anchor + (now - wall_anchor) * speed
            while frame_index + 1 < qpos.shape[0] and timestamps[frame_index + 1] <= target_t:
                frame_index += 1

            if target_t > timestamps[-1]:
                if not args.loop:
                    break
                frame_index = 0
                wall_anchor = now
                clip_anchor = float(timestamps[0])

            data.qpos[:] = qpos[frame_index]
            data.qvel[:] = qvel[frame_index]
            data.ctrl[:] = qpos[frame_index, 7:]
            mujoco.mj_forward(model, data)
            viewer.sync()
            time.sleep(0.002)


if __name__ == "__main__":
    main()
