# Wearable IMU Source Branch

We originally developed the wearable IMU, inverse-kinematics, calibration, hardware, demo, and test code on this branch:

https://github.com/triton-droids/cse-145-237d-humanoid-teleop/tree/inverse-kinematics

In our course-project integration branch, we organized that branch's repository-root content under `wearable_imu/` so it can live beside the perception, retargeting, simulation, reports, and media folders.

## Layout Mapping

| Source branch path | Integrated path |
|---|---|
| `hardware/` | `wearable_imu/hardware/` |
| `sensor/` | `wearable_imu/sensor/` |
| `calibration/` | `wearable_imu/calibration/` |
| `ik/` | `wearable_imu/ik/` |
| `model/` | `wearable_imu/model/` |
| `simulator/` | `wearable_imu/simulator/` |
| `demos/` | `wearable_imu/demos/` |
| `tests/` | `wearable_imu/tests/` |
| `env/` | `wearable_imu/env/` |

## Sync Policy

We keep the integrated `wearable_imu/` tree focused on source code, tests, hardware
sketches, setup files, and small documentation. We don't copy branch-root
`.DS_Store` files, local captures, generated plots, environment folders, or
temporary hardware logs into this branch.

When `inverse-kinematics` changes, we review the diff first and sync only the
source/documentation changes that should be part of the main project repository.
