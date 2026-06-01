# Demo Workflows

This folder contains runnable scripts that connect multiple modules.

To run demos from one clickable place:

```powershell
conda run --no-capture-output -n humanoid-sim python demos\demo_launcher.py
```

The launcher starts each demo as a separate Python process, shows its output,
and provides a Stop button for live or hardware-dependent workflows.

Current demos:

- `demo_launcher.py`: clickable control panel for launching the demos below
- `demo_udp_quaternion_receiver.py`: receiver-side UDP monitor for ESP32-S3
  BNO085 quaternion packets, with receive jitter and drop estimates
- `demo_udp_latency_ping.py`: round-trip UDP ping latency test for one ESP32-S3
  node
- `demo_imu_orientation_ik.py`: synthetic quaternion-to-joint estimate demo
- `demo_inverse_kinematics.py`: toy planar leg IK demo
- `demo_lower_body_aggregation.py`: static lower-body segment aggregation demo
- `demo_live_lower_body_aggregation.py`: synthetic live lower-body aggregation
  visualization
- `demo_partial_imu_live_viewer.py`: live UDP partial-IMU skeleton viewer;
  the launcher exposes separate Thighs, Shanks, and Full entries
- `demo_visualize_lower_body_model.py`: static lower-body model visualization
- `demo_mujoco_lower_body_viewer.py`: archived MuJoCo visual harness with fake
  IMU quaternions and live estimator display

Demos are allowed to be practical and integrated. Core logic should still live
in workflow folders like `sensor/`, `calibration/`, `ik/`, and `simulator/`.
