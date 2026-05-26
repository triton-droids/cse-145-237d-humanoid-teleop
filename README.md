# CSE 145/237D Humanoid Teleoperation

Wearable lower-body motion capture and simulation-based policy training for humanoid robot teleoperation.

## Abstract

This project builds a humanoid teleoperation pipeline that lets a humanoid robot imitate lower-body human motion. The system combines wearable IMU nodes, a depth-camera perception baseline, calibration and inverse kinematics, retargeting to the Triton humanoid model, and reinforcement-learning simulation in Isaac Lab.

The target pipeline is:

```text
Wearable IMUs / RGB-D camera
    -> human lower-body pose estimation
    -> calibration and retargeting
    -> humanoid motion-tracking policy in simulation
    -> sim-to-real deployment on the robot
```

## Repository Organization

| Path | Purpose |
|---|---|
| `wearable_imu/` | Real-world BNO085 + ESP32-S3 IMU pipeline, UDP packets, calibration, filtering, IK, tests, and demos. |
| `perception/depth_camera/` | Intel RealSense D435 + MediaPipe pose-estimation baseline. |
| `retargeting/` | Triton humanoid retargeting contract, `ch_robot` joint order, and Holosoma patch notes. |
| `simulation/` | Pointer to the dedicated Isaac Lab training repository and pinned simulation commit. |
| `data/retargeting_samples/` | Small sample retargeting captures used for pipeline validation. |
| `docs/` | Project overview, architecture, setup notes, migration notes, and internal planning documents. |
| `reports/` | CSE 145/237D reports, milestone material, presentations, and final deliverables. |
| `media/` | Demo videos, screenshots, and images for the project page and final presentation. |

## Current Status

- Wearable IMU hardware path: BNO085 quaternion streaming over ESP32-S3 UDP is scaffolded.
- Jetson-side packet parsing, latest-packet buffering, filtering, smoothing, and neutral calibration are implemented.
- Lower-body orientation IK and MuJoCo synthetic test harness exist for validation.
- RealSense D435 + MediaPipe baseline estimates depth-backed pose landmarks.
- Triton humanoid retargeting contract and CH robot Holosoma converter patch are documented.
- Isaac Lab humanoid locomotion simulation exists and is being extended toward motion tracking.
- Final sim-to-real deployment is still in progress.

## Quick Start

Each subsystem has its own setup notes:

- Wearable IMU pipeline: [`wearable_imu/README.md`](wearable_imu/README.md)
- Depth-camera baseline: [`perception/depth_camera/README.md`](perception/depth_camera/README.md)
- Retargeting contract: [`retargeting/README.md`](retargeting/README.md)
- Isaac Lab simulation: [`simulation/README.md`](simulation/README.md)
- High-level replication guide: [`docs/replication-guide.md`](docs/replication-guide.md)

## Team

Triton Droids, UC San Diego.

Primary project areas:

- Embedded wearable sensing and Jetson communication
- Human pose estimation and RGB-D baseline
- Retargeting and inverse kinematics
- Isaac Lab simulation, policy training, and sim-to-real preparation
- Documentation, reports, and class deliverables

## CSE 145/237D Materials

Course-facing deliverables are collected under [`reports/`](reports/):

- Milestone report: [`reports/milestone_report.md`](reports/milestone_report.md)
- Presentations: [`reports/presentations/`](reports/presentations/)
- Demo media: [`media/`](media/)

## Artifact Policy

Training checkpoints, TensorBoard logs, generated Hydra outputs, large raw captures, and third-party source drops should not be committed directly to the main branch. Use GitHub Releases, external storage, or a clearly documented private artifact location, then link them from `reports/` or `docs/`.

See [`docs/artifact-policy.md`](docs/artifact-policy.md).
