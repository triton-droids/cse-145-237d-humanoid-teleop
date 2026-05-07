# Demo Workflows

This folder contains runnable scripts that connect multiple modules.

Current demos:

- `demo_udp_quaternion_receiver.py`: Jetson-side UDP monitor for ESP32-S3
  BNO085 quaternion packets, with receive jitter and drop estimates
- `demo_udp_latency_ping.py`: round-trip UDP ping latency test for one ESP32-S3
  node
- `demo_imu_orientation_ik.py`: synthetic quaternion-to-joint estimate demo
- `demo_inverse_kinematics.py`: toy planar leg IK demo
- `demo_visualize_lower_body_model.py`: static lower-body model visualization
- `demo_mujoco_lower_body_viewer.py`: archived MuJoCo visual harness with fake
  IMU quaternions and live estimator display

Demos are allowed to be practical and integrated. Core logic should still live
in workflow folders like `sensor/`, `calibration/`, `ik/`, and `simulator/`.
