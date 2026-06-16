import numpy as np

from model.lower_body import build_lower_body_model


def test_lateral_imu_mounts_are_on_outside_of_each_leg() -> None:
    model = build_lower_body_model()

    left_thigh = model.segments["left_thigh"]
    right_thigh = model.segments["right_thigh"]
    left_mount = model.imu_mounts["left_thigh"]
    right_mount = model.imu_mounts["right_thigh"]

    left_offset = left_mount.position - left_thigh.origin
    right_offset = right_mount.position - right_thigh.origin

    assert left_offset[1] > 0.0
    assert right_offset[1] < 0.0


def test_foot_imu_mount_sits_above_the_foot_segment() -> None:
    model = build_lower_body_model()

    left_foot = model.segments["left_foot"]
    left_mount = model.imu_mounts["left_foot"]
    foot_up_axis = left_foot.rotation.apply([0.0, 0.0, 1.0])

    mount_offset = left_mount.position - left_foot.origin
    assert float(np.dot(mount_offset, foot_up_axis)) > 0.0


def test_sensor_face_normals_match_mount_description() -> None:
    model = build_lower_body_model()

    left_thigh_mount = model.imu_mounts["left_thigh"]
    right_shank_mount = model.imu_mounts["right_shank"]
    left_foot_mount = model.imu_mounts["left_foot"]

    left_face_normal = left_thigh_mount.sensor_rotation_world.apply([0.0, 0.0, 1.0])
    right_face_normal = right_shank_mount.sensor_rotation_world.apply([0.0, 0.0, 1.0])
    foot_face_normal = left_foot_mount.sensor_rotation_world.apply([0.0, 0.0, 1.0])

    assert left_face_normal[1] > 0.0
    assert right_face_normal[1] < 0.0
    assert foot_face_normal[2] > 0.0
