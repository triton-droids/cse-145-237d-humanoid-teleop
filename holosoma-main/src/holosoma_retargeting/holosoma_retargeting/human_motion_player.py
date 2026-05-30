from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import tyro
import viser


LAFAN_JOINTS = [
    "Hips",
    "RightUpLeg",
    "RightLeg",
    "RightFoot",
    "RightToeBase",
    "LeftUpLeg",
    "LeftLeg",
    "LeftFoot",
    "LeftToeBase",
    "Spine",
    "Spine1",
    "Spine2",
    "Neck",
    "Head",
    "RightShoulder",
    "RightArm",
    "RightForeArm",
    "RightHand",
    "LeftShoulder",
    "LeftArm",
    "LeftForeArm",
    "LeftHand",
]

LAFAN_BONES = [
    ("Hips", "RightUpLeg"),
    ("RightUpLeg", "RightLeg"),
    ("RightLeg", "RightFoot"),
    ("RightFoot", "RightToeBase"),
    ("Hips", "LeftUpLeg"),
    ("LeftUpLeg", "LeftLeg"),
    ("LeftLeg", "LeftFoot"),
    ("LeftFoot", "LeftToeBase"),
    ("Hips", "Spine"),
    ("Spine", "Spine1"),
    ("Spine1", "Spine2"),
    ("Spine2", "Neck"),
    ("Neck", "Head"),
    ("Spine2", "RightShoulder"),
    ("RightShoulder", "RightArm"),
    ("RightArm", "RightForeArm"),
    ("RightForeArm", "RightHand"),
    ("Spine2", "LeftShoulder"),
    ("LeftShoulder", "LeftArm"),
    ("LeftArm", "LeftForeArm"),
    ("LeftForeArm", "LeftHand"),
]


@dataclass(frozen=True)
class HumanMotionPlayerConfig:
    input_path: Path
    """Path to a raw .npy motion file, or a retargeted .npz containing human_joints."""

    npz_key: str = "human_joints"
    """Array key to read when input_path is .npz."""

    fps: float | None = None
    """Playback FPS override. Defaults to npz fps if present, otherwise 30."""

    y_up_to_z_up: bool = True
    """For raw LaFAN .npy files, convert [x, up-y, forward-z] into z-up world coordinates."""

    point_size: float = 0.025
    line_width: float = 4.0
    loop: bool = True


def transform_y_up_to_z_up(points: np.ndarray) -> np.ndarray:
    transform_matrix = np.array([[1, 0, 0], [0, 0, 1], [0, 1, 0]], dtype=points.dtype)
    points_flat = points.reshape(-1, 3)
    transformed = (transform_matrix @ points_flat.T).T
    return transformed.reshape(points.shape)


def load_motion(config: HumanMotionPlayerConfig) -> tuple[np.ndarray, float]:
    if config.input_path.suffix == ".npz":
        data = np.load(config.input_path)
        motion = np.asarray(data[config.npz_key], dtype=np.float32)
        fps = float(config.fps if config.fps is not None else data["fps"] if "fps" in data else 30.0)
        return motion, fps

    motion = np.asarray(np.load(config.input_path), dtype=np.float32)
    if config.y_up_to_z_up:
        motion = transform_y_up_to_z_up(motion)
    return motion, float(config.fps if config.fps is not None else 30.0)


def make_bone_indices(joint_names: list[str]) -> np.ndarray:
    name_to_idx = {name: i for i, name in enumerate(joint_names)}
    return np.asarray(
        [(name_to_idx[parent], name_to_idx[child]) for parent, child in LAFAN_BONES],
        dtype=np.int64,
    )


def main(config: HumanMotionPlayerConfig) -> None:
    motion, fps = load_motion(config)
    if motion.ndim != 3 or motion.shape[-1] != 3:
        raise ValueError(f"Expected motion shape (T, J, 3), got {motion.shape}")
    if motion.shape[1] != len(LAFAN_JOINTS):
        raise ValueError(f"Expected {len(LAFAN_JOINTS)} LaFAN joints, got {motion.shape[1]}")

    server = viser.ViserServer()
    print(f"Loaded {motion.shape[0]} frames from {config.input_path}")
    print(f"Open the Viser URL printed above. Playing at {fps:g} FPS.")

    bone_indices = make_bone_indices(LAFAN_JOINTS)
    point_colors = np.full((motion.shape[1], 3), np.array([50, 140, 255]), dtype=np.uint8)
    line_colors = np.full((len(bone_indices), 2, 3), np.array([30, 30, 30]), dtype=np.uint8)

    points_handle = server.scene.add_point_cloud(
        "/human/joints",
        points=motion[0],
        colors=point_colors,
        point_size=config.point_size,
        point_shape="circle",
    )
    bones_handle = server.scene.add_line_segments(
        "/human/skeleton",
        points=motion[0, bone_indices],
        colors=line_colors,
        line_width=config.line_width,
    )

    with server.gui.add_folder("Playback"):
        playing = server.gui.add_checkbox("Playing", initial_value=True)
        frame_slider = server.gui.add_slider(
            "Frame",
            min=0,
            max=motion.shape[0] - 1,
            step=1,
            initial_value=0,
        )

    def update_frame(frame_idx: int) -> None:
        idx = int(np.clip(frame_idx, 0, motion.shape[0] - 1))
        points_handle.points = motion[idx]
        bones_handle.points = motion[idx, bone_indices]
        frame_slider.value = idx

    @frame_slider.on_update
    def _(_event) -> None:
        if not playing.value:
            update_frame(int(frame_slider.value))

    frame_idx = 0
    while True:
        if playing.value:
            update_frame(frame_idx)
            frame_idx += 1
            if frame_idx >= motion.shape[0]:
                frame_idx = 0 if config.loop else motion.shape[0] - 1
                playing.value = bool(config.loop)
            time.sleep(1.0 / fps)
        else:
            time.sleep(0.05)


if __name__ == "__main__":
    main(tyro.cli(HumanMotionPlayerConfig))
