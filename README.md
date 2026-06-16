# CSE 145/237D Humanoid Teleoperation

Wearable lower-body motion capture and simulation-based policy training for humanoid robot teleoperation.

## Abstract

We built a humanoid teleoperation pipeline that lets a humanoid robot imitate lower-body human motion. Our system combines wearable IMU nodes, a depth-camera perception baseline, calibration and inverse kinematics, retargeting to the Triton humanoid model, and reinforcement-learning simulation in Isaac Lab.

Our pipeline is:

```text
Wearable IMUs / RGB-D camera
    -> human lower-body pose estimation
    -> calibration and retargeting
    -> humanoid motion-tracking policy in simulation
    -> sim-to-real deployment on the robot
```

## Repository Organization

| Path                            | Purpose                                                                                               |
| ------------------------------- | ----------------------------------------------------------------------------------------------------- |
| `wearable_imu/`                 | Real-world BNO085 + ESP32-S3 IMU pipeline, UDP packets, calibration, filtering, IK, tests, and demos. |
| `perception/depth_camera/`      | Intel RealSense D435 + MediaPipe pose-estimation baseline.                                            |
| `retargeting/`                  | Triton humanoid retargeting contract, `ch_robot` joint order, and Holosoma patch notes.               |
| `simulation/`                   | Pointer to the dedicated Isaac Lab training repository and pinned simulation commit.                  |
| `data/retargeting_samples/`     | Small sample retargeting captures used for pipeline validation.                                       |
| `data/retargeting_experiments/` | Curated small experiment outputs that verify retargeting and conversion contracts.                    |
| `docs/`                         | Project overview, architecture, setup notes, migration notes, and internal planning documents.        |
| `reports/`                      | CSE 145/237D reports, milestone material, presentations, and final deliverables.                      |
| `media/`                        | Demo videos, screenshots, and images for the project page and final presentation.                     |

## Current Status

- Wearable IMU hardware path: BNO085 quaternion streaming over ESP32-S3 UDP is scaffolded.
- Receiver-side (laptop) packet parsing, latest-packet buffering, filtering, smoothing, and neutral calibration are implemented.
- Lower-body orientation IK and MuJoCo synthetic test harness exist for validation.
- RealSense D435 + MediaPipe baseline estimates depth-backed pose landmarks.
- Triton humanoid retargeting contract and CH robot Holosoma converter patch are documented.
- Isaac Lab humanoid locomotion simulation is running and we are extending it toward motion tracking.
- Final sim-to-real deployment is still in progress.

## Quick Start

Each subsystem has its own setup notes — start there if you're new to a component:

- Wearable IMU pipeline: [`wearable_imu/README.md`](wearable_imu/README.md)
- Depth-camera baseline: [`perception/depth_camera/README.md`](perception/depth_camera/README.md)
- Retargeting contract: [`retargeting/README.md`](retargeting/README.md)
- Isaac Lab simulation: [`simulation/README.md`](simulation/README.md)
- High-level replication guide: [`docs/replication-guide.md`](docs/replication-guide.md)

## Team

Triton Droids, UC San Diego.

| Name               | Role              | Focus                                                                                                            |
| ------------------ | ----------------- | --------------------------------------------------------------------------------------------------------------- |
| Darin Djapri       | Team Lead         | ML/RL, policy & reward design, IsaacLab simulation, sim-to-real                                                  |
| Fong-Yu (Yang) Lin | ML Engineer       | RL, policy & reward functions, Sim2Sim (IsaacLab → MuJoCo), data pipeline                                        |
| Cindy Chen         | ML Engineer       | Human pose extraction, 3D keypoint retargeting, Holosoma pipeline                                                |
| Parth Trivedi      | Embedded Engineer | IMU + ESP32 hardware, Jetson integration, depth estimation baseline                                             |
| Neal Jian          | Embedded Engineer | Wearable IMU hardware & battery design, ESP32 firmware, UART/wireless pipeline, IK from quaternions, motion retargeting, free-root walking |
| Tauhid Malik       | Embedded Engineer | IMU aggregation pipeline, Jetson networking, hardware bring-up                                                   |

## CSE 145/237D Materials

Course-facing deliverables are collected under [`reports/`](reports/):

- Milestone report: [`reports/milestone_report.md`](reports/milestone_report.md)
- Presentations: [`reports/presentations/`](reports/presentations/)
- Demo media: [`media/`](media/)

## Source Mapping

| Source                        | Destination                      | Notes                                                                                    |
| ----------------------------- | -------------------------------- | ---------------------------------------------------------------------------------------- |
| `origin/inverse-kinematics`   | `wearable_imu/`                  | Real-world IMU packet, calibration, IK, demo, and test code.                             |
| `origin/depthCamera`          | `perception/depth_camera/`       | RealSense D435 + MediaPipe pose-estimation baseline.                                     |
| `origin/retargeting_data`     | `data/retargeting_samples/`      | Small sample JSONL captures.                                                             |
| `origin/retargeting_holosoma` | `retargeting/`                   | Project-specific CH robot retargeting notes and Holosoma converter patch only.           |
| `triton-droids/simulation`    | `simulation/README.md` link only | Current Isaac Lab humanoid simulation code stays in the dedicated simulation repository. |
