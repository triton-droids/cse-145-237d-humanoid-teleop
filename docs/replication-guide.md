# Replication Guide

Here are the high-level steps to reproduce our project.

## Hardware

- 5 wearable IMU nodes using BNO085 orientation sensors and ESP32-S3 microcontrollers
- Laptop (or any host on the same hotspot) as the UDP receiver; a robot-side Jetson is the eventual on-robot target
- Triton humanoid robot model
- Optional Intel RealSense D435 camera for the RGB-D baseline

## Wearable IMU Pipeline

See [`../wearable_imu/README.md`](../wearable_imu/README.md).

Typical flow:

```text
flash ESP32-S3 firmware
assign sensor_id and segment_id
stream BNO085 quaternion packets over UDP
receive packets on the laptop
filter and smooth quaternions
run neutral-pose calibration
feed calibrated segment orientations into IK
```

## Depth-Camera Baseline

See [`../perception/depth_camera/README.md`](../perception/depth_camera/README.md).

Typical flow:

```text
install MediaPipe, pyrealsense2, OpenCV, NumPy
download the MediaPipe pose landmarker model
connect Intel RealSense D435
run pose_realsense_simple.py
inspect depth-backed landmark coordinates
```

## Isaac Lab Simulation

See [`../simulation/README.md`](../simulation/README.md), then follow the README in our dedicated simulation repository:

https://github.com/triton-droids/simulation

Typical flow:

```text
install Isaac Lab
clone triton-droids/simulation at the pinned commit
install the tritonhumanoid extension in editable mode from that repository
list available tasks
train or play a humanoid policy
```

## Retargeting

See [`../retargeting/README.md`](../retargeting/README.md).

Typical flow:

```text
produce human lower-body pose or keypoints
retarget to ch_robot qpos with 7 base values + 10 joint positions
apply the Holosoma ch_robot converter patch if using Holosoma
convert retargeted motion into Isaac Lab / RL tracking data
validate joint_pos width 17 and joint_vel width 16
```

## Reports and Media

We place class reports, presentation files, and demo videos under:

- `reports/`
- `reports/presentations/`
- `media/videos/`
- `media/images/`
