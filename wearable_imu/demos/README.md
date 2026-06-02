# Demo Workflows

This folder has runnable scripts we built to connect multiple modules.

Current demos:

- `demo_launcher.py`: clickable Tk launcher for the main wearable IMU demo
  workflows
- `demo_udp_quaternion_receiver.py`: Jetson-side UDP monitor for ESP32-S3
  BNO085 quaternion packets, with receive jitter and drop estimates
- `demo_udp_latency_ping.py`: round-trip UDP ping latency test for one ESP32-S3
  node
- `demo_imu_orientation_ik.py`: synthetic quaternion-to-joint estimate demo
- `demo_inverse_kinematics.py`: toy planar leg IK demo
- `demo_lower_body_aggregation.py`: one-frame lower-body skeleton aggregation
  from segment orientations
- `demo_live_lower_body_aggregation.py`: animated synthetic lower-body
  aggregation viewer
- `demo_partial_imu_live_viewer.py`: live skeleton viewer for pelvis+thigh,
  pelvis+thigh+shank, or full seven-segment IMU sets
- `demo_visualize_lower_body_model.py`: static lower-body model visualization
- `demo_mujoco_lower_body_viewer.py`: archived MuJoCo visual harness with fake
  IMU quaternions and live estimator display

We keep demos practical and integrated. Core logic stays in workflow folders like
`sensor/`, `calibration/`, `ik/`, and `simulator/`.
