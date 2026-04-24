# Simulator Workflow

This folder is now an archived/supporting simulation harness.

MuJoCo is useful for:

- visualizing the current lower-body model
- generating clean synthetic IMU quaternions
- injecting fake strap/mount errors later
- regression testing estimator behavior

It is not the main product workflow anymore. Real-world calibration and BNO085
quaternion handling should live in `hardware/`, `sensor/`, and `calibration/`.

Current viewer:

```powershell
conda run -p ..\.conda python ..\demos\demo_mujoco_lower_body_viewer.py
```
