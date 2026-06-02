from __future__ import annotations

import numpy as np

from holosoma_retargeting.data_conversion.add_retargeted_velocities import (
    compute_retargeted_velocities,
    enforce_quaternion_sign_continuity,
    enrich_file,
    resolve_jobs,
)


def quat_from_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=np.float64)
    axis = axis / np.linalg.norm(axis)
    half = 0.5 * angle
    return np.concatenate([[np.cos(half)], axis * np.sin(half)])


def quat_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        dtype=np.float64,
    )


def make_qpos(num_frames: int, fps: float = 30.0) -> np.ndarray:
    qpos = np.zeros((num_frames, 17), dtype=np.float64)
    qpos[:, 3] = 1.0
    qpos[:, 7:] = np.linspace(0.0, 1.0, num_frames)[:, None]
    qpos[:, 0] = np.arange(num_frames, dtype=np.float64) / fps
    return qpos


def test_static_pose_has_zero_velocities() -> None:
    qpos = make_qpos(4)
    qpos[:, 0] = 0.0
    qpos[:, 7:] = 0.0

    velocities = compute_retargeted_velocities(qpos, fps=30.0)

    assert velocities["qvel"].shape == (4, 16)
    assert velocities["joint_vel"].shape == (4, 10)
    assert np.allclose(velocities["qvel"], 0.0)
    assert velocities["qvel"].dtype == np.float32


def test_constant_translation_and_joint_ramp() -> None:
    fps = 30.0
    qpos = make_qpos(5, fps=fps)

    velocities = compute_retargeted_velocities(qpos, fps=fps)

    assert np.allclose(velocities["root_lin_vel_w"][:, 0], 1.0)
    assert np.allclose(velocities["root_lin_vel_w"][:, 1:], 0.0)
    expected_joint_vel = np.full((5, 10), fps / 4.0)
    assert np.allclose(velocities["joint_vel"], expected_joint_vel)
    assert np.allclose(velocities["qvel"][:, :3], velocities["root_lin_vel_w"])


def test_yaw_rate_and_quaternion_sign_continuity() -> None:
    fps = 10.0
    yaw_rate = 0.25
    times = np.arange(6, dtype=np.float64) / fps
    qpos = make_qpos(6, fps=fps)
    for i, yaw in enumerate(yaw_rate * times):
        qpos[i, 3:7] = quat_from_axis_angle(np.array([0.0, 0.0, 1.0]), yaw)
    qpos[3:, 3:7] *= -1.0

    continuous = enforce_quaternion_sign_continuity(qpos[:, 3:7])
    velocities = compute_retargeted_velocities(qpos, fps=fps)

    assert all(np.dot(continuous[i], continuous[i - 1]) >= 0.0 for i in range(1, continuous.shape[0]))
    assert np.allclose(velocities["yaw_rate_ref"], yaw_rate, atol=1e-6)
    assert np.allclose(velocities["root_ang_vel_w"][:, 2], yaw_rate, atol=1e-6)
    assert np.allclose(velocities["root_ang_vel_b"][:, 2], yaw_rate, atol=1e-6)


def test_qvel_uses_body_frame_angular_velocity() -> None:
    fps = 20.0
    roll_rate = 0.4
    times = np.arange(5, dtype=np.float64) / fps
    yaw_90 = quat_from_axis_angle(np.array([0.0, 0.0, 1.0]), np.pi / 2.0)
    qpos = make_qpos(5, fps=fps)
    for i, t in enumerate(times):
        roll = quat_from_axis_angle(np.array([1.0, 0.0, 0.0]), roll_rate * t)
        qpos[i, 3:7] = quat_multiply(yaw_90, roll)

    velocities = compute_retargeted_velocities(qpos, fps=fps)

    assert np.allclose(velocities["root_ang_vel_w"][:, 0], 0.0, atol=1e-6)
    assert np.allclose(velocities["root_ang_vel_w"][:, 1], roll_rate, atol=1e-6)
    assert np.allclose(velocities["root_ang_vel_b"][:, 0], roll_rate, atol=1e-6)
    assert np.allclose(velocities["root_ang_vel_b"][:, 1:], 0.0, atol=1e-6)
    assert np.allclose(velocities["qvel"][:, 3:6], velocities["root_ang_vel_b"])


def test_short_clips() -> None:
    one = compute_retargeted_velocities(make_qpos(1), fps=30.0)
    two = compute_retargeted_velocities(make_qpos(2), fps=30.0)

    assert np.allclose(one["qvel"], 0.0)
    assert two["qvel"].shape == (2, 16)
    assert np.allclose(two["root_lin_vel_w"][:, 0], 1.0)
    assert np.allclose(two["joint_vel"], 30.0)


def test_enrich_file_preserves_existing_keys_and_writes_outputs(tmp_path) -> None:
    input_path = tmp_path / "motion.npz"
    output_path = tmp_path / "motion_with_vel.npz"
    qpos = make_qpos(3)
    human_joints = np.zeros((3, 22, 3), dtype=np.float64)
    np.savez(input_path, qpos=qpos, human_joints=human_joints, fps=30, cost=1.25)

    summary = enrich_file(
        input_path,
        output_path,
        robot="ch_robot",
        fallback_fps=None,
        overwrite=False,
        cast_qpos_float32=False,
    )

    assert summary.nonfinite_count == 0
    with np.load(output_path) as data:
        assert set(["qpos", "qvel", "joint_vel", "root_ang_vel_b", "yaw_rate_ref"]).issubset(data.files)
        assert data["qpos"].dtype == qpos.dtype
        assert data["qvel"].shape == (3, 16)
        assert data["human_joints"].shape == human_joints.shape
        assert np.isclose(float(data["cost"]), 1.25)


def test_resolve_jobs_default_single_file_output(tmp_path) -> None:
    input_path = tmp_path / "motion.npz"
    input_path.touch()

    jobs = resolve_jobs(input_path, None, recursive=False)

    assert jobs == [(input_path, tmp_path / "motion_with_vel.npz")]
