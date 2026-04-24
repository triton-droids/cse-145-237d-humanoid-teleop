import numpy as np
from scipy.spatial.transform import Rotation

from ik.imu_orientation import (
    X_FORWARD,
    Y_LEFT,
    Z_UP,
    default_lower_limb_mounts,
    front_pelvis_mount,
    imu_orientation_from_segment,
    solve_lower_limb_joints_from_imus,
)


def test_default_mount_axes_match_described_wearable_placement() -> None:
    left_mounts = default_lower_limb_mounts("left")
    right_mounts = default_lower_limb_mounts("right")

    np.testing.assert_allclose(left_mounts["thigh"].apply([1.0, 0.0, 0.0]), Z_UP)
    np.testing.assert_allclose(left_mounts["thigh"].apply([0.0, 0.0, 1.0]), Y_LEFT)

    np.testing.assert_allclose(right_mounts["shank"].apply([1.0, 0.0, 0.0]), Z_UP)
    np.testing.assert_allclose(right_mounts["shank"].apply([0.0, 0.0, 1.0]), -Y_LEFT)

    np.testing.assert_allclose(left_mounts["foot"].apply([1.0, 0.0, 0.0]), X_FORWARD)
    np.testing.assert_allclose(left_mounts["foot"].apply([0.0, 0.0, 1.0]), Z_UP)

    pelvis_mount = front_pelvis_mount()
    np.testing.assert_allclose(pelvis_mount.apply([1.0, 0.0, 0.0]), Z_UP)
    np.testing.assert_allclose(pelvis_mount.apply([0.0, 0.0, 1.0]), X_FORWARD)


def test_joint_rotations_round_trip_from_mounted_imus() -> None:
    side = "left"
    mounts = default_lower_limb_mounts(side)
    mounts["pelvis"] = front_pelvis_mount()

    pelvis_world = Rotation.from_euler("xyz", [2.0, -1.0, 4.0], degrees=True)
    expected_hip = Rotation.from_euler("xyz", [5.0, 20.0, -3.0], degrees=True)
    expected_knee = Rotation.from_euler("xyz", [0.0, 48.0, 0.0], degrees=True)
    expected_ankle = Rotation.from_euler("xyz", [1.0, -12.0, 5.0], degrees=True)

    thigh_world = pelvis_world * expected_hip
    shank_world = thigh_world * expected_knee
    foot_world = shank_world * expected_ankle

    imu_world_sensor = {
        "pelvis": imu_orientation_from_segment(pelvis_world, mounts["pelvis"]),
        "thigh": imu_orientation_from_segment(thigh_world, mounts["thigh"]),
        "shank": imu_orientation_from_segment(shank_world, mounts["shank"]),
        "foot": imu_orientation_from_segment(foot_world, mounts["foot"]),
    }

    solution = solve_lower_limb_joints_from_imus(
        imu_world_sensor,
        side=side,
        mounts=mounts,
    )

    assert solution.joints.hip is not None
    np.testing.assert_allclose(
        solution.joints.hip.as_matrix(),
        expected_hip.as_matrix(),
        atol=1e-12,
    )
    np.testing.assert_allclose(
        solution.joints.knee.as_matrix(),
        expected_knee.as_matrix(),
        atol=1e-12,
    )
    np.testing.assert_allclose(
        solution.joints.ankle.as_matrix(),
        expected_ankle.as_matrix(),
        atol=1e-12,
    )


def test_hip_requires_pelvis_mount_if_pelvis_imu_is_supplied() -> None:
    mounts = default_lower_limb_mounts("left")
    imu_world_sensor = {
        "pelvis": Rotation.identity(),
        "thigh": imu_orientation_from_segment(Rotation.identity(), mounts["thigh"]),
        "shank": imu_orientation_from_segment(Rotation.identity(), mounts["shank"]),
        "foot": imu_orientation_from_segment(Rotation.identity(), mounts["foot"]),
    }

    try:
        solve_lower_limb_joints_from_imus(imu_world_sensor, side="left", mounts=mounts)
    except ValueError as exc:
        assert "pelvis IMU requires a pelvis mount calibration" in str(exc)
    else:
        raise AssertionError("pelvis IMU without pelvis mount should fail")
