"""Live leg-only retargeting and visualization for ch_robot.

Inputs are 9 Z-up keypoints in LIVE_LEG_JOINTS order, or LAFAN replay arrays
that can be converted into that order.
"""

from __future__ import annotations

import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import numpy as np
import tyro

src_root = Path(__file__).resolve().parents[2]
if str(src_root) not in sys.path:
    sys.path.insert(0, str(src_root))

from holosoma_retargeting.config_types.data_type import (  # noqa: E402
    LAFAN_DEMO_JOINTS,
    LIVE_LEG_DEMO_JOINTS,
    MotionDataConfig,
)
from holosoma_retargeting.config_types.retargeter import RetargeterConfig  # noqa: E402
from holosoma_retargeting.config_types.robot import RobotConfig  # noqa: E402
from holosoma_retargeting.config_types.task import TaskConfig  # noqa: E402
from holosoma_retargeting.examples.robot_retarget import (  # noqa: E402
    build_retargeter_kwargs_from_config,
    create_task_constants,
    setup_object_data,
)
from holosoma_retargeting.src.interaction_mesh_retargeter import InteractionMeshRetargeter  # noqa: E402
from holosoma_retargeting.src.utils import (  # noqa: E402
    adjust_yaw_for_robot_forward_axis,
    calculate_laplacian_coordinates,
    create_interaction_mesh,
    estimate_human_forward_yaw,
    get_adjacency_list,
    transform_y_up_to_z_up,
    yaw_to_quat_wxyz,
)

LIVE_LEG_JOINTS = tuple(LIVE_LEG_DEMO_JOINTS)
LEFT_TOE_IDX = LIVE_LEG_JOINTS.index("LeftToeBase")
RIGHT_TOE_IDX = LIVE_LEG_JOINTS.index("RightToeBase")
SPINE1_IDX = LIVE_LEG_JOINTS.index("Spine1")


@dataclass
class LiveLegRetargetConfig:
    input_mode: Literal["zmq", "replay-npy"] = "replay-npy"
    """Input mode: live ZeroMQ stream or local replay .npy/.npz."""

    zmq_endpoint: str = "tcp://127.0.0.1:5556"
    """ZeroMQ endpoint. The retargeter connects as a SUB socket."""

    replay_path: Path | None = None
    """Path to replay .npy/.npz data."""

    replay_start: int = 0
    """First replay frame to use."""

    replay_count: int | None = None
    """Number of replay frames to use."""

    robot: str = "ch_robot"
    """Robot type. v1 is intended for ch_robot."""

    canonicalize_input: bool = True
    """Subtract initial Spine1 xy and initial toe-ground z from incoming data."""

    input_scale: float = 1.0
    """Scale applied to live-leg data after canonicalization."""

    lafan_replay_scale: float = 1.27 / 1.7
    """Scale applied when replay input is detected as raw LAFAN data."""

    solver_iters: int = 3
    """SQP iterations after initialization."""

    init_solver_iters: int = 20
    """SQP iterations for the first frame."""

    fix_orientation: bool = True
    """Lock base roll/pitch and use yaw estimated from hip line."""

    visualize: bool = True
    """Visualize solved robot poses in Viser."""

    keep_viewer_open: bool = False
    """Keep the process alive after replay so the Viser viewer remains open."""

    save_path: Path | None = None
    """Optional .npz output path for solved qpos history."""

    ground_size: int = 8
    """Ground mesh grid size used in the interaction mesh."""

    poll_timeout_ms: int = 100
    """ZeroMQ receive poll timeout."""


def encode_zmq_frame(
    frame: np.ndarray,
    *,
    frame_idx: int,
    timestamp: float | None = None,
    joint_names: list[str] | tuple[str, ...] | None = None,
) -> tuple[bytes, bytes]:
    """Encode one keypoint frame as [header_json, raw_bytes]."""
    arr = np.ascontiguousarray(frame)
    if arr.ndim != 2 or arr.shape[1] != 3:
        raise ValueError(f"Expected frame shape (J, 3), got {arr.shape}")
    if arr.dtype not in (np.dtype("float32"), np.dtype("float64")):
        arr = arr.astype(np.float32)
    header = {
        "frame_idx": int(frame_idx),
        "timestamp": float(time.time() if timestamp is None else timestamp),
        "shape": list(arr.shape),
        "dtype": str(arr.dtype),
    }
    if joint_names is not None:
        header["joint_names"] = list(joint_names)
    return json.dumps(header).encode("utf-8"), arr.tobytes()


def reorder_frame_by_joint_names(
    frame: np.ndarray,
    joint_names: list[str] | tuple[str, ...],
    expected_joint_names: tuple[str, ...] = LIVE_LEG_JOINTS,
) -> np.ndarray:
    """Reorder a frame containing named joints into live-leg order."""
    name_to_idx = {name: i for i, name in enumerate(joint_names)}
    missing = [name for name in expected_joint_names if name not in name_to_idx]
    if missing:
        raise ValueError(f"Missing required live-leg joints: {missing}")
    return np.asarray([frame[name_to_idx[name]] for name in expected_joint_names], dtype=float)


def decode_zmq_frame(
    parts: list[bytes] | tuple[bytes, bytes],
    *,
    expected_joint_names: tuple[str, ...] = LIVE_LEG_JOINTS,
) -> tuple[int, float, np.ndarray]:
    """Decode [header_json, raw_bytes] into (frame_idx, timestamp, frame)."""
    if len(parts) != 2:
        raise ValueError(f"Expected 2 ZeroMQ message parts, got {len(parts)}")
    header = json.loads(parts[0].decode("utf-8"))
    dtype = np.dtype(header["dtype"])
    if dtype not in (np.dtype("float32"), np.dtype("float64")):
        raise ValueError(f"Unsupported dtype: {dtype}")
    shape = tuple(int(value) for value in header["shape"])
    frame = np.frombuffer(parts[1], dtype=dtype).reshape(shape).astype(float)
    if "joint_names" in header:
        frame = reorder_frame_by_joint_names(frame, header["joint_names"], expected_joint_names)
    if frame.shape != (len(expected_joint_names), 3):
        raise ValueError(f"Expected frame shape {(len(expected_joint_names), 3)}, got {frame.shape}")
    return int(header["frame_idx"]), float(header["timestamp"]), frame


class LiveInputCanonicalizer:
    """Fixed-origin canonicalizer for live leg keypoints."""

    def __init__(self, *, enabled: bool = True, scale: float = 1.0) -> None:
        self.enabled = enabled
        self.scale = float(scale)
        self._origin_xy: np.ndarray | None = None
        self._ground_z: float | None = None

    def apply(self, frame: np.ndarray) -> np.ndarray:
        arr = np.asarray(frame, dtype=float).copy()
        if not self.enabled:
            return arr * self.scale
        if self._origin_xy is None:
            self._origin_xy = arr[SPINE1_IDX, :2].copy()
            self._ground_z = float(min(arr[LEFT_TOE_IDX, 2], arr[RIGHT_TOE_IDX, 2]))
        arr[:, :2] -= self._origin_xy
        arr[:, 2] -= float(self._ground_z)
        return arr * self.scale


def canonicalize_live_leg_sequence(sequence: np.ndarray, *, enabled: bool = True, scale: float = 1.0) -> np.ndarray:
    canonicalizer = LiveInputCanonicalizer(enabled=enabled, scale=scale)
    return np.asarray([canonicalizer.apply(frame) for frame in sequence], dtype=float)


def extract_live_leg_sequence_from_lafan(lafan_motion: np.ndarray) -> np.ndarray:
    """Convert raw LAFAN (T, 22, 3) y-up data into live-leg (T, 9, 3) Z-up order."""
    if lafan_motion.ndim != 3 or lafan_motion.shape[1:] != (len(LAFAN_DEMO_JOINTS), 3):
        raise ValueError(f"Expected LAFAN shape (T, {len(LAFAN_DEMO_JOINTS)}, 3), got {lafan_motion.shape}")
    zup = transform_y_up_to_z_up(np.asarray(lafan_motion, dtype=float).copy())
    zup[:, LAFAN_DEMO_JOINTS.index("Spine1"), 2] -= 0.06
    indices = [LAFAN_DEMO_JOINTS.index(name) for name in LIVE_LEG_JOINTS]
    return zup[:, indices, :]


def load_replay_sequence(path: Path, *, lafan_scale: float = 1.27 / 1.7) -> tuple[np.ndarray, float]:
    """Load replay input and return (T, 9, 3) live-leg data plus default scale."""
    if path.suffix == ".npz":
        with np.load(path) as data:
            if "points" in data:
                arr = data["points"]
            elif "human_joints" in data:
                arr = data["human_joints"]
            else:
                raise ValueError(f"{path} must contain 'points' or 'human_joints'")
    else:
        arr = np.load(path)

    arr = np.asarray(arr, dtype=float)
    if arr.ndim != 3 or arr.shape[2] != 3:
        raise ValueError(f"Expected replay shape (T, J, 3), got {arr.shape}")
    if arr.shape[1] == len(LIVE_LEG_JOINTS):
        return arr, 1.0
    if arr.shape[1] == len(LAFAN_DEMO_JOINTS):
        return extract_live_leg_sequence_from_lafan(arr), float(lafan_scale)
    raise ValueError(f"Cannot interpret replay shape {arr.shape}; expected 9 or 22 joints")


def slice_replay_sequence(sequence: np.ndarray, start: int, count: int | None) -> np.ndarray:
    if start < 0:
        raise ValueError(f"replay_start must be non-negative, got {start}")
    if count is not None and count <= 0:
        raise ValueError(f"replay_count must be positive, got {count}")
    end = None if count is None else start + count
    sliced = sequence[start:end]
    if len(sliced) == 0:
        raise ValueError(f"Replay slice is empty for start={start}, count={count}")
    return sliced


def _estimate_live_yaw(frame: np.ndarray, robot_forward_axis: str) -> np.ndarray:
    one_frame = np.asarray(frame, dtype=float)[None, :, :]
    yaw = estimate_human_forward_yaw(one_frame, list(LIVE_LEG_JOINTS), frame_idx=0)
    return yaw_to_quat_wxyz(adjust_yaw_for_robot_forward_axis(yaw, robot_forward_axis))


class LiveLegRetargeter:
    """Stateful one-frame-at-a-time wrapper around InteractionMeshRetargeter."""

    def __init__(self, cfg: LiveLegRetargetConfig) -> None:
        self.cfg = cfg
        robot_config = RobotConfig(robot_type=cfg.robot)
        motion_data_config = MotionDataConfig(data_format="live_legs", robot_type=cfg.robot)
        task_config = TaskConfig(object_name="ground", ground_size=cfg.ground_size)
        self.constants = create_task_constants(robot_config, motion_data_config, task_config, "robot_only")
        self.object_points_local, self.object_points_local_demo, object_urdf_path = setup_object_data(
            "robot_only",
            self.constants,
            None,
            1.0,
            task_config,
            False,
        )
        retargeter_config = RetargeterConfig(
            activate_foot_sticking=False,
            fix_orientation=cfg.fix_orientation,
            visualize=cfg.visualize,
        )
        kwargs = build_retargeter_kwargs_from_config(retargeter_config, self.constants, object_urdf_path, "robot_only")
        self.retargeter = InteractionMeshRetargeter(**kwargs)
        self.default_q = np.asarray(self.retargeter.robot_model.qpos0, dtype=float).copy()
        self.current_command_q = self.default_q.copy()
        self.q: np.ndarray | None = None
        self.q_last: np.ndarray | None = None
        self.frame_idx = 0
        self.qpos_history: list[np.ndarray] = []
        if self.cfg.visualize:
            self.retargeter.draw_q(self.current_command_q)

    def _initial_q(self, frame: np.ndarray) -> np.ndarray:
        q_joints = np.asarray(self.constants.Q_INIT_JOINTS, dtype=float)
        quat = _estimate_live_yaw(frame, getattr(self.constants, "ROBOT_FORWARD_AXIS", "+x"))
        return np.concatenate([frame[SPINE1_IDX, :3], quat, q_joints])

    def solve_frame(self, frame: np.ndarray) -> tuple[np.ndarray, float]:
        if self.q is None:
            self.q = self._initial_q(frame)
            self.q_last = self.q.copy()
            self.retargeter._stance_root_height_target = float(self.q[2])

        q_locked = np.zeros(self.retargeter.nq)
        q_locked[: len(self.q)] = self.q
        if self.cfg.fix_orientation:
            q_locked[3:7] = _estimate_live_yaw(frame, getattr(self.constants, "ROBOT_FORWARD_AXIS", "+x"))

        human_mapped_joints = frame[self.retargeter.smplh_mapped_joint_indices]
        source_vertices, source_tetrahedra = create_interaction_mesh(
            np.vstack([human_mapped_joints, self.object_points_local_demo])
        )
        adj_list = get_adjacency_list(source_tetrahedra, len(source_vertices))
        target_laplacian = calculate_laplacian_coordinates(source_vertices, adj_list)

        q_solved, cost = self.retargeter.iterate(
            q_locked=q_locked,
            q_n=self.q,
            q_t_last=self.q_last if self.q_last is not None else self.q,
            target_laplacian=target_laplacian,
            adj_list=adj_list,
            obj_pts_local=self.object_points_local,
            foot_sticking={"left": False, "right": False},
            stance_contact_confidence=None,
            init_t=self.frame_idx == 0,
            n_iter=self.cfg.init_solver_iters if self.frame_idx == 0 else self.cfg.solver_iters,
            frame_idx=self.frame_idx,
        )
        self.q_last = self.q
        self.q = q_solved
        self.current_command_q = q_solved.copy()
        self.frame_idx += 1
        self.qpos_history.append(q_solved.copy())
        if self.cfg.visualize:
            self.retargeter.draw_q(q_solved)
        return q_solved, float(cost)

    def hold_current_command(self) -> np.ndarray:
        """Reuse the latest solved command, or the MJCF default before input arrives."""
        if self.cfg.visualize:
            self.retargeter.draw_q(self.current_command_q)
        return self.current_command_q.copy()


class ZmqFrameReceiver:
    def __init__(self, endpoint: str) -> None:
        try:
            import zmq
        except ModuleNotFoundError as exc:
            raise ModuleNotFoundError("pyzmq is required for --input-mode zmq. Install package dependency pyzmq.") from exc

        self.zmq = zmq
        self.context = zmq.Context.instance()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.SUBSCRIBE, b"")
        self.socket.connect(endpoint)
        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)

    def recv_latest(self, timeout_ms: int) -> tuple[int, float, np.ndarray] | None:
        events = dict(self.poller.poll(timeout_ms))
        if self.socket not in events:
            return None
        parts = self.socket.recv_multipart()
        while True:
            try:
                parts = self.socket.recv_multipart(flags=self.zmq.NOBLOCK)
            except self.zmq.Again:
                break
        return decode_zmq_frame(parts)


def save_qpos_history(path: Path, qpos_history: list[np.ndarray], fps: int = 30) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(path, qpos=np.asarray(qpos_history), fps=fps)
    print(f"Saved qpos history to: {path}")


def run_replay(cfg: LiveLegRetargetConfig) -> list[np.ndarray]:
    if cfg.replay_path is None:
        raise ValueError("--replay-path is required when --input-mode replay-npy")
    sequence, detected_scale = load_replay_sequence(cfg.replay_path, lafan_scale=cfg.lafan_replay_scale)
    sequence = slice_replay_sequence(sequence, cfg.replay_start, cfg.replay_count)
    scale = cfg.input_scale * detected_scale
    sequence = canonicalize_live_leg_sequence(sequence, enabled=cfg.canonicalize_input, scale=scale)
    live_rt = LiveLegRetargeter(cfg)
    for i, frame in enumerate(sequence):
        _, cost = live_rt.solve_frame(frame)
        print(f"[replay] frame={i:04d} cost={cost:.4f}")
    if cfg.save_path is not None:
        save_qpos_history(cfg.save_path, live_rt.qpos_history)
    return live_rt.qpos_history


def run_zmq(cfg: LiveLegRetargetConfig) -> list[np.ndarray]:
    receiver = ZmqFrameReceiver(cfg.zmq_endpoint)
    canonicalizer = LiveInputCanonicalizer(enabled=cfg.canonicalize_input, scale=cfg.input_scale)
    live_rt = LiveLegRetargeter(cfg)
    print(f"Listening for live-leg frames on {cfg.zmq_endpoint}")
    try:
        while True:
            decoded = receiver.recv_latest(cfg.poll_timeout_ms)
            if decoded is None:
                live_rt.hold_current_command()
                continue
            frame_idx, timestamp, frame = decoded
            q, cost = live_rt.solve_frame(canonicalizer.apply(frame))
            latency_ms = (time.time() - timestamp) * 1000.0
            print(f"[zmq] frame={frame_idx} cost={cost:.4f} latency_ms={latency_ms:.1f} q_z={q[2]:.3f}")
    except KeyboardInterrupt:
        print("Stopping live retargeter.")
    finally:
        if cfg.save_path is not None:
            save_qpos_history(cfg.save_path, live_rt.qpos_history)
    return live_rt.qpos_history


def main(cfg: LiveLegRetargetConfig) -> None:
    if cfg.input_mode == "replay-npy":
        run_replay(cfg)
        if cfg.visualize and cfg.keep_viewer_open:
            input("Press Enter to exit ...")
        return
    if cfg.input_mode == "zmq":
        run_zmq(cfg)
        return
    raise ValueError(f"Unsupported input mode: {cfg.input_mode}")


if __name__ == "__main__":
    main(tyro.cli(LiveLegRetargetConfig))
