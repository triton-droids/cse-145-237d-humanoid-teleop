"""Fake lower-body IMU pipeline for the humanoid teleop project.

Data flow:
1. Generate fake segment orientations for 7 planned IMUs.
2. Emit those orientations as timestamped quaternion packets in BNO085-style
   order: qx, qy, qz, qw.
3. Convert relative segment orientations into lower-body joint targets.
4. Convert those targets into the normalized action format already expected by
   HumanoidLocomotionEnv, then step the MuJoCo simulator.

This keeps the "sensor source" separate from the simulator. The primary goal is
to define and exercise a clean Jetson-side IMU data interface before the real
ESP32/BNO085 hardware arrives. The MuJoCo hookup is only a secondary sanity
check / visualization path.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import asdict, dataclass
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from envs.locomotion_env import HumanoidLocomotionEnv


IMU_NAMES = (
    "waist",
    "left_thigh",
    "left_shin",
    "left_foot",
    "right_thigh",
    "right_shin",
    "right_foot",
)

SCHEMA_NAME = "triton_humanoid_imu_frame/v1"


@dataclass(frozen=True)
class ImuPacket:
    """Serializable fake IMU packet shaped for easy hardware replacement."""

    name: str
    frame_id: str
    stamp_sec: float
    frame_index: int
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass(frozen=True)
class FakeImuFrame:
    """One frame of fake IMU data."""

    frame_index: int
    sim_time_sec: float
    packets: dict[str, ImuPacket]
    quats_wxyz: dict[str, np.ndarray]

    def to_jsonable(self) -> dict[str, object]:
        stamp_sec = self.packets[IMU_NAMES[0]].stamp_sec
        return {
            "schema": SCHEMA_NAME,
            "source": "fake_imu_pipeline",
            "frame_index": self.frame_index,
            "sim_time_sec": self.sim_time_sec,
            "stamp_sec": stamp_sec,
            "imus": [asdict(self.packets[name]) for name in IMU_NAMES],
        }


def quat_identity() -> np.ndarray:
    return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)


def quat_normalize(q: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(q))
    if norm < 1e-12:
        return quat_identity()
    return q / norm


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)


def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product for quaternions in (w, x, y, z) order."""

    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=float,
    )


def quat_inverse(q: np.ndarray) -> np.ndarray:
    return quat_conjugate(quat_normalize(q))


def quat_from_axis_angle(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis_norm = float(np.linalg.norm(axis))
    if axis_norm < 1e-12:
        return quat_identity()
    axis = axis / axis_norm
    half = 0.5 * float(angle_rad)
    s = math.sin(half)
    return quat_normalize(np.array([math.cos(half), axis[0] * s, axis[1] * s, axis[2] * s], dtype=float))


def quat_to_bno085_order(q_wxyz: np.ndarray) -> tuple[float, float, float, float]:
    q = quat_normalize(q_wxyz)
    return (float(q[1]), float(q[2]), float(q[3]), float(q[0]))


def relative_quat(parent_wxyz: np.ndarray, child_wxyz: np.ndarray) -> np.ndarray:
    return quat_normalize(quat_multiply(quat_inverse(parent_wxyz), child_wxyz))


def signed_angle_about_axis(q_wxyz: np.ndarray, axis: np.ndarray) -> float:
    """Project a relative quaternion onto a single hinge axis.

    This is intentionally simple because the fake motion is generated in a
    single sagittal plane. It is enough for a clean IMU -> joint demo.
    """

    q = quat_normalize(q_wxyz)
    if q[0] < 0.0:
        q = -q

    xyz = q[1:]
    xyz_norm = float(np.linalg.norm(xyz))
    if xyz_norm < 1e-12:
        return 0.0

    axis = np.asarray(axis, dtype=float)
    axis = axis / (np.linalg.norm(axis) + 1e-12)
    rot_axis = xyz / xyz_norm
    angle = 2.0 * math.atan2(xyz_norm, q[0])
    return float(angle * np.dot(rot_axis, axis))


class FakeLowerBodyImuSource:
    """Generates simple fake lower-body orientation patterns."""

    def __init__(self, swing_hz: float = 0.65, motion_mode: str = "sway", motion_scale: float = 1.0):
        self.swing_hz = float(swing_hz)
        self.motion_mode = str(motion_mode)
        self.motion_scale = float(max(0.0, motion_scale))
        self.x_axis = np.array([1.0, 0.0, 0.0], dtype=float)

    def _joint_profile(self, sim_time_sec: float) -> dict[str, float]:
        phase = 2.0 * math.pi * self.swing_hz * sim_time_sec
        left_swing = math.sin(phase)
        right_swing = math.sin(phase + math.pi)

        # Default to a small symmetric sway. It stays much closer to the
        # standing keyframe, which is important because this script does not
        # include a balance controller. The "walk" mode remains available for
        # stronger leg alternation when paired with --lock-root.
        if self.motion_mode == "walk":
            waist_pitch = 0.01 * self.motion_scale * math.sin(0.5 * phase)
            left_hip = 0.30 + 0.06 * self.motion_scale * left_swing
            right_hip = 0.30 + 0.06 * self.motion_scale * right_swing
            left_knee = -0.80 - 0.06 * self.motion_scale * max(0.0, left_swing)
            right_knee = -0.80 - 0.06 * self.motion_scale * max(0.0, right_swing)
            left_ankle = 0.50 - 0.03 * self.motion_scale * left_swing
            right_ankle = 0.50 - 0.03 * self.motion_scale * right_swing
        else:
            bend = math.sin(phase)
            waist_pitch = 0.005 * self.motion_scale * math.sin(0.5 * phase)
            left_hip = 0.30 + 0.02 * self.motion_scale * bend
            right_hip = 0.30 + 0.02 * self.motion_scale * bend
            left_knee = -0.80 - 0.03 * self.motion_scale * bend
            right_knee = -0.80 - 0.03 * self.motion_scale * bend
            left_ankle = 0.50 - 0.015 * self.motion_scale * bend
            right_ankle = 0.50 - 0.015 * self.motion_scale * bend

        return {
            "waist_pitch": waist_pitch,
            "left_hip": left_hip,
            "right_hip": right_hip,
            "left_knee": left_knee,
            "right_knee": right_knee,
            "left_ankle": left_ankle,
            "right_ankle": right_ankle,
        }

    def sample(self, sim_time_sec: float, frame_index: int) -> FakeImuFrame:
        profile = self._joint_profile(sim_time_sec)

        waist_q = quat_from_axis_angle(self.x_axis, profile["waist_pitch"])
        left_thigh_q = quat_multiply(waist_q, quat_from_axis_angle(self.x_axis, profile["left_hip"]))
        right_thigh_q = quat_multiply(waist_q, quat_from_axis_angle(self.x_axis, profile["right_hip"]))
        left_shin_q = quat_multiply(left_thigh_q, quat_from_axis_angle(self.x_axis, profile["left_knee"]))
        right_shin_q = quat_multiply(right_thigh_q, quat_from_axis_angle(self.x_axis, profile["right_knee"]))
        left_foot_q = quat_multiply(left_shin_q, quat_from_axis_angle(self.x_axis, profile["left_ankle"]))
        right_foot_q = quat_multiply(right_shin_q, quat_from_axis_angle(self.x_axis, profile["right_ankle"]))

        quats_wxyz = {
            "waist": quat_normalize(waist_q),
            "left_thigh": quat_normalize(left_thigh_q),
            "left_shin": quat_normalize(left_shin_q),
            "left_foot": quat_normalize(left_foot_q),
            "right_thigh": quat_normalize(right_thigh_q),
            "right_shin": quat_normalize(right_shin_q),
            "right_foot": quat_normalize(right_foot_q),
        }

        stamp_sec = time.time()
        packets: dict[str, ImuPacket] = {}
        for name, quat_wxyz in quats_wxyz.items():
            qx, qy, qz, qw = quat_to_bno085_order(quat_wxyz)
            packets[name] = ImuPacket(
                name=name,
                frame_id=f"{name}_imu",
                stamp_sec=stamp_sec,
                frame_index=frame_index,
                qx=qx,
                qy=qy,
                qz=qz,
                qw=qw,
            )

        return FakeImuFrame(
            frame_index=frame_index,
            sim_time_sec=float(sim_time_sec),
            packets=packets,
            quats_wxyz=quats_wxyz,
        )


class LowerBodyImuToJointTargets:
    """Converts segment IMU orientations into MuJoCo policy-order joint targets."""

    def __init__(self, env: "HumanoidLocomotionEnv"):
        self._env = env
        self._x_axis = np.array([1.0, 0.0, 0.0], dtype=float)
        self._standing_policy = env._standing_qpos[env._policy_qpos_adr].copy()
        self._policy_scales = env._joint_action_scales[env._policy_to_mj].copy()
        self._policy_name_to_index = {name: idx for idx, name in enumerate(env._policy_joint_order)}

    def joint_targets_from_frame(self, frame: FakeImuFrame) -> np.ndarray:
        quats = frame.quats_wxyz
        targets = self._standing_policy.copy()

        left_hip_rel = relative_quat(quats["waist"], quats["left_thigh"])
        right_hip_rel = relative_quat(quats["waist"], quats["right_thigh"])
        left_knee_rel = relative_quat(quats["left_thigh"], quats["left_shin"])
        right_knee_rel = relative_quat(quats["right_thigh"], quats["right_shin"])
        left_ankle_rel = relative_quat(quats["left_shin"], quats["left_foot"])
        right_ankle_rel = relative_quat(quats["right_shin"], quats["right_foot"])

        targets[self._policy_name_to_index["left_hip1_joint"]] = signed_angle_about_axis(left_hip_rel, self._x_axis)
        targets[self._policy_name_to_index["right_hip1_joint"]] = signed_angle_about_axis(right_hip_rel, self._x_axis)
        targets[self._policy_name_to_index["left_knee_joint"]] = signed_angle_about_axis(left_knee_rel, self._x_axis)
        targets[self._policy_name_to_index["right_knee_joint"]] = signed_angle_about_axis(right_knee_rel, self._x_axis)
        targets[self._policy_name_to_index["left_ankle_joint"]] = signed_angle_about_axis(left_ankle_rel, self._x_axis)
        targets[self._policy_name_to_index["right_ankle_joint"]] = signed_angle_about_axis(right_ankle_rel, self._x_axis)

        # This fake source only drives the sagittal-plane chain for now.
        targets[self._policy_name_to_index["left_hip2_joint"]] = 0.0
        targets[self._policy_name_to_index["right_hip2_joint"]] = 0.0
        targets[self._policy_name_to_index["left_thigh_joint"]] = 0.0
        targets[self._policy_name_to_index["right_thigh_joint"]] = 0.0

        return np.clip(targets, self._env._joint_soft_lower_policy, self._env._joint_soft_upper_policy)

    def actions_from_targets(self, targets_policy: np.ndarray) -> np.ndarray:
        delta = targets_policy - self._standing_policy
        denom = self._env._action_scale * self._policy_scales
        actions = delta / (denom + 1e-8)
        return np.clip(actions, -1.0, 1.0)


def _open_jsonl(path: str | None):
    if path is None:
        return None
    return open(path, "w", encoding="utf-8")


def _schema_example_json() -> str:
    frame = FakeLowerBodyImuSource().sample(sim_time_sec=0.0, frame_index=0)
    return json.dumps(frame.to_jsonable(), indent=2)


def _hold_root_at_standing_pose(env: "HumanoidLocomotionEnv") -> None:
    """Pin the floating base for sensor-visualization mode.

    This is useful when you want to watch the leg motion generated from fake
    IMUs without needing a balance controller.
    """

    import mujoco

    env.data.qpos[:7] = env._standing_qpos[:7]
    env.data.qvel[:6] = 0.0
    mujoco.mj_forward(env.model, env.data)


def run_pipeline(args: argparse.Namespace) -> None:
    should_simulate = bool(args.simulate or args.viewer)
    if args.print_only:
        should_simulate = False

    source = FakeLowerBodyImuSource(
        swing_hz=args.swing_hz,
        motion_mode=args.motion_mode,
        motion_scale=args.motion_scale,
    )
    env = None
    converter = None

    if should_simulate:
        try:
            from envs.locomotion_env import HumanoidLocomotionEnv
            import mujoco
        except ModuleNotFoundError as exc:
            raise SystemExit(
                "MuJoCo is required for simulator mode. Install the repo dependencies or run with --print-only."
            ) from exc

        env = HumanoidLocomotionEnv(
            xml_path=args.xml_path,
            act_max_latency=0,
            act_latency_steps=0,
            act_delay_range_by_name={},
            obs_max_latency=0,
            obs_latency_steps=0,
            disturbance_prob=0.0,
            action_smoothing_alpha=0.35,
            action_delta_max=0.08,
        )
        env.reset()
        env._commands[:] = 0.0
        converter = LowerBodyImuToJointTargets(env)

    viewer = None
    if args.viewer:
        if not should_simulate:
            raise SystemExit("--viewer cannot be used with --print-only.")
        import mujoco.viewer

        viewer = mujoco.viewer.launch_passive(env.model, env.data)

    jsonl_file = _open_jsonl(args.dump_jsonl)
    loop_dt = 1.0 / args.fps
    max_steps = args.num_steps
    if max_steps is None and args.duration is not None:
        max_steps = max(1, int(round(args.duration * args.fps)))

    frame_index = 0
    next_tick = time.perf_counter()

    try:
        while True:
            if (max_steps is not None) and (frame_index >= max_steps):
                break
            if viewer is not None and not viewer.is_running():
                break

            sim_time_sec = frame_index * loop_dt
            frame = source.sample(sim_time_sec=sim_time_sec, frame_index=frame_index)
            targets = None
            actions = None
            if env is not None and converter is not None:
                targets = converter.joint_targets_from_frame(frame)
                actions = converter.actions_from_targets(targets)
                env.step(actions)
                if args.root_locked:
                    _hold_root_at_standing_pose(env)

            if args.print_every > 0 and frame_index % args.print_every == 0:
                payload = frame.to_jsonable()
                if env is not None and targets is not None and actions is not None:
                    payload["policy_targets"] = {
                        name: float(targets[idx]) for idx, name in enumerate(env._policy_joint_order)
                    }
                    payload["policy_actions"] = {
                        name: float(actions[idx]) for idx, name in enumerate(env._policy_joint_order)
                    }
                print(json.dumps(payload))

            if jsonl_file is not None:
                jsonl_file.write(json.dumps(frame.to_jsonable()) + "\n")
                jsonl_file.flush()

            if viewer is not None:
                viewer.sync()
            if args.real_time:
                now = time.perf_counter()
                if now < next_tick:
                    time.sleep(next_tick - now)
                next_tick += loop_dt

            frame_index += 1
    finally:
        if viewer is not None:
            viewer.close()
        if jsonl_file is not None:
            jsonl_file.close()


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the fake lower-body IMU pipeline.")
    parser.add_argument(
        "--xml-path",
        type=str,
        default="robot_description/scene.xml",
        help="Path to the MuJoCo scene xml.",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=50.0,
        help="Fake IMU and control update rate.",
    )
    parser.add_argument(
        "--swing-hz",
        type=float,
        default=0.65,
        help="Frequency of the fake lower-body swing cycle.",
    )
    parser.add_argument(
        "--motion-mode",
        type=str,
        choices=("sway", "walk"),
        default="sway",
        help="Fake motion profile. 'sway' is more stable. 'walk' is for larger alternating leg motion.",
    )
    parser.add_argument(
        "--motion-scale",
        type=float,
        default=1.0,
        help="Scale factor applied to the fake motion amplitudes.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Optional run duration in seconds. If omitted, run until Ctrl+C.",
    )
    parser.add_argument(
        "--num-steps",
        type=int,
        default=None,
        help="Optional fixed number of frames to run.",
    )
    parser.add_argument(
        "--simulate",
        action="store_true",
        help="Convert fake IMU frames into joint commands and step the MuJoCo env.",
    )
    parser.add_argument(
        "--viewer",
        action="store_true",
        help="Launch the MuJoCo passive viewer. Implies --simulate.",
    )
    parser.add_argument(
        "--lock-root",
        action="store_true",
        help="Pin the floating base to the standing pose during simulation.",
    )
    parser.add_argument(
        "--free-root",
        action="store_true",
        help="Disable the default root lock used for viewer-based visualization.",
    )
    parser.add_argument(
        "--print-only",
        action="store_true",
        help="Run the fake IMU source without importing MuJoCo or stepping the simulator.",
    )
    parser.add_argument(
        "--show-schema",
        action="store_true",
        help="Print an example Jetson-facing IMU frame and exit.",
    )
    parser.add_argument(
        "--real-time",
        action="store_true",
        help="Sleep to match the requested FPS.",
    )
    parser.add_argument(
        "--print-every",
        type=int,
        default=None,
        help="Print one JSON payload every N frames. Set to 1 for every frame, 0 to disable.",
    )
    parser.add_argument(
        "--dump-jsonl",
        type=str,
        default=None,
        help="Optional path to write one JSON object per frame.",
    )
    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()
    if args.show_schema:
        print(_schema_example_json())
        return

    should_simulate = bool(args.simulate or args.viewer)
    if args.print_only and args.viewer:
        raise SystemExit("--print-only cannot be combined with --viewer.")

    # Viewer mode is for visualization, not free-body balance testing, so pin
    # the base by default unless the caller explicitly opts out.
    args.root_locked = bool(args.lock_root or (args.viewer and not args.free_root))

    if args.print_every is None:
        args.print_every = 25 if should_simulate else 1

    run_pipeline(args)


if __name__ == "__main__":
    main()
