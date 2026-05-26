# Simulator Workflow

This folder is now an archived/supporting simulation harness.

MuJoCo is useful for:

- visualizing the current lower-body model
- generating clean synthetic IMU quaternions
- generating BNO085-like quaternion packets through the same sensor contract
- injecting fake strap/mount errors later
- regression testing estimator behavior

It is not the main product workflow anymore. Real-world calibration and BNO085
quaternion handling should live in `hardware/`, `sensor/`, and `calibration/`.

Simulation should follow the same packet assumptions as hardware whenever data
leaves the simulator:

```text
sensor_id
segment_id
sequence
sensor_time_us
quat_wxyz
accuracy/status placeholders
```

Current viewer:

```powershell
conda run -p ..\.conda python ..\demos\demo_mujoco_lower_body_viewer.py
```
