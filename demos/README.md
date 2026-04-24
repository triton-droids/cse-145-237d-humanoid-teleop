# Demo Workflows

This folder contains runnable scripts that connect multiple modules.

Current demos:

- `demo_imu_orientation_ik.py`: synthetic quaternion-to-joint estimate demo
- `demo_inverse_kinematics.py`: toy planar leg IK demo
- `demo_visualize_lower_body_model.py`: static lower-body model visualization
- `demo_mujoco_lower_body_viewer.py`: archived MuJoCo visual harness with fake
  IMU quaternions and live estimator display

Demos are allowed to be practical and integrated. Core logic should still live
in workflow folders like `sensor/`, `calibration/`, `ik/`, and `simulator/`.
