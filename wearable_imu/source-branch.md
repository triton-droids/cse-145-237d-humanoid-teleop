# Wearable IMU Source Branch

The wearable IMU, inverse-kinematics, calibration, hardware, demo, and test code
was originally developed on this branch:

https://github.com/triton-droids/cse-145-237d-humanoid-teleop/tree/inverse-kinematics

In the course-project integration branch, that branch's repository-root content
is organized under `wearable_imu/` so it can live beside the perception,
retargeting, simulation, reports, and media folders.

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

Keep the integrated `wearable_imu/` tree focused on source code, tests, hardware
sketches, setup files, and small documentation. Do not copy branch-root
`.DS_Store` files, local captures, generated plots, environment folders, or
temporary hardware logs into this branch.

When `inverse-kinematics` changes, review the diff first and sync only the
source/documentation changes that should be part of the main project repository.
