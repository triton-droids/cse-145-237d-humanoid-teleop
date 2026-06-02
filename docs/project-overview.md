# Project Overview

This repository is the CSE 145/237D project hub for Triton Droids humanoid
teleoperation. It connects the class reports, media, lightweight integration
code, and documentation for a pipeline that turns human lower-body motion into
humanoid robot motion targets.

The project combines four development threads:

- wearable IMU lower-body pose estimation
- depth-camera pose estimation as a non-wearable baseline
- Holosoma-based retargeting to the Triton humanoid `ch_robot` model
- Isaac Lab simulation and policy training in a dedicated simulation repository

## System Goal

Our end-to-end goal is humanoid teleoperation: estimate a human lower-body pose,
retarget it to the Triton humanoid joint contract, and use simulation-trained
policies to make the robot track the resulting motion.

```text
Wearable IMUs or RGB-D camera
    -> human lower-body pose / keypoints
    -> calibration and inverse kinematics
    -> retargeting to Triton humanoid `ch_robot`
    -> motion-tracking data for Isaac Lab
    -> policy evaluation and sim-to-real deployment
```

The final sim-to-real deployment path is still in progress. This repository
documents the pipeline and preserves the portable integration pieces needed to
reproduce the class project.

## Repository Layout

| Path | Purpose |
|---|---|
| `wearable_imu/` | BNO085 + ESP32-S3 wearable IMU pipeline, UDP packet parsing, filtering, calibration, IK, demos, and tests. |
| `perception/depth_camera/` | Intel RealSense D435 + MediaPipe depth-backed pose-estimation baseline. |
| `retargeting/` | Triton humanoid retargeting contract, `ch_robot` joint order, Holosoma notes, and portable patch files. |
| `simulation/` | Pointer to the dedicated Isaac Lab simulation repository and pinned simulation commit. |
| `data/retargeting_samples/` | Small sample retargeting captures used for validation. |
| `data/retargeting_experiments/` | Curated small converted outputs that verify retargeting and conversion contracts. |
| `docs/` | Architecture, replication, repository organization, and project overview docs. |
| `reports/` | CSE 145/237D reports, presentations, and final deliverables. |
| `media/` | Demo videos and images used for reports and presentations. |

## Current Status

The wearable IMU branch now supports a practical lower-body pipeline:

- ESP32-S3/BNO085 quaternion firmware scaffolds for UART and UDP workflows
- 40-byte little-endian `IMUQ` quaternion packet parsing
- latest-packet buffering, packet freshness checks, and jitter/drop diagnostics
- quaternion norm, stale-data, spike filtering, and SLERP smoothing
- neutral standing calibration and lower-body orientation IK
- partial IMU configurations for 3, 5, or 7 sensor setups
- live skeleton viewing with calibration and recording controls
- `.npz` human joint clip recording and playback for ML retargeting handoff
- a GUI demo launcher with editable arguments and stable UI scaling

The retargeting branch now includes a full Holosoma-based working branch and a
portable subset in this integration branch:

- `ch_robot` joint-order contract for the 10-DOF Triton humanoid lower body
- compact retargeted motion contract with `qpos`, `fps`, `human_joints`, and `cost`
- converter notes for Isaac Lab / RL tracking output
- dynamic robot-DOF slicing instead of hardcoded G1 29-DOF assumptions
- FPS parsing fixes and headless conversion support
- live leg-retargeting workflow over ZeroMQ using 9 lower-body keypoints
- validation on synthetic data and an OMOMO clip converted to 50 FPS tracking data

The integration branch intentionally does not vendor the full Holosoma source
tree. Use `origin/retargeting_holosoma` for the complete Holosoma worktree, and
use `retargeting/` in this branch for the project-specific contract, notes, and
patches.

## Source Branches

| Source | Integrated Area | Notes |
|---|---|---|
| `origin/inverse-kinematics` | `wearable_imu/` | Real-world IMU packet, calibration, IK, demo, test, recording, and playback code. Latest inspected tip: `66a6454` (`fix UI scaling`). |
| `origin/depthCamera` | `perception/depth_camera/` | RealSense D435 + MediaPipe depth-backed pose-estimation baseline. |
| `origin/retargeting_data` | `data/retargeting_samples/` | Small JSONL captures for retargeting validation. |
| `origin/retargeting_holosoma` | `retargeting/` plus branch pointer | Complete Holosoma worktree and live retargeting instructions. Latest inspected tip: `2bc4b3c` (`added penalties for flat feet on parallel retargeting`). |
| `triton-droids/simulation` | `simulation/README.md` link only | Isaac Lab training source, robot assets, logs, and generated files stay in the dedicated simulation repository. |

## Data Contracts

### Wearable IMU Packets

The BNO085 reports fused orientation quaternions. ESP32-S3 nodes stream those
quaternions directly to the receiver over UDP; the pelvis node is not a packet
hub.

```text
magic[4]          "IMUQ"
version           uint8
sensor_id         uint8
segment_id        uint8
quat_order        uint8
sequence          uint32
sensor_time_us    uint32
qw qx qy qz       float32
accuracy          float32
status            uint8
report_type       uint8
reserved          uint16
```

Segment IDs:

```text
0 pelvis
1 left_thigh
2 left_shank
3 left_foot
4 right_thigh
5 right_shank
6 right_foot
255 unknown
```

### Human Joint Handoff

The ML retargeting handoff uses 9 lower-body keypoints in meters, Z-up:

```text
Spine1
LeftUpLeg
LeftLeg
LeftFoot
LeftToeBase
RightUpLeg
RightLeg
RightFoot
RightToeBase
```

Recorded `.npz` clips should include `joint_names`, `joint_pos_origin`,
`timestamps_s`, and `fps`. Additional fields such as `joint_pos_w`,
`root_quat_wxyz`, `config`, and `required_segments` help debug partial IMU
recordings.

### Robot Retargeting Output

For `ch_robot`, compact retargeted `qpos` has width 17:

```text
7 floating-base values + 10 robot joint positions
```

The 10 robot joints are:

```text
left_hip1_joint
left_hip2_joint
left_thigh_joint
left_knee_joint
left_ankle_joint
right_hip1_joint
right_hip2_joint
right_thigh_joint
right_knee_joint
right_ankle_joint
```

The converted Isaac Lab / RL tracking output includes:

```text
joint_pos
joint_vel
body_pos_w
body_quat_w
body_lin_vel_w
body_ang_vel_w
joint_names
body_names
fps
```

## Running Key Workflows

Start with subsystem README files:

- wearable IMU pipeline: [`../wearable_imu/README.md`](../wearable_imu/README.md)
- wearable demos: [`../wearable_imu/demos/README.md`](../wearable_imu/demos/README.md)
- depth-camera baseline: [`../perception/depth_camera/README.md`](../perception/depth_camera/README.md)
- retargeting contract: [`../retargeting/README.md`](../retargeting/README.md)
- simulation pointer: [`../simulation/README.md`](../simulation/README.md)
- replication guide: [`replication-guide.md`](replication-guide.md)

Useful wearable IMU commands from `wearable_imu/`:

```bash
python demos/demo_launcher.py
python demos/demo_launcher.py --list
python demos/demo_partial_imu_live_viewer.py --host 0.0.0.0 --port 5005 --config full
python demos/demo_record_human_joint_clip.py --config shanks --duration-s 10 --fps 30 --output data/recordings/example_walk.npz
python demos/demo_play_human_joint_clip.py data/recordings/example_walk.npz --origin
python -m pytest -q
```

The live Holosoma retargeting branch expects 9 keypoints per frame and can run a
ZeroMQ receiver plus a fake publisher for replay data. See the
`RUN_RETARGETING_LIVE.md` file on `origin/retargeting_holosoma` for the full
branch-local setup.

## Engineering Boundaries

This integration branch keeps the repository small and course-facing:

- do keep portable contracts, docs, patches, tests, small samples, and final media
- do not vendor the full Holosoma source tree here
- do not duplicate Isaac Lab simulation assets, checkpoints, generated USDs, or logs
- keep downloaded datasets, raw long-duration captures, and large generated files out of Git
- use the dedicated branch or repository for full-source development work

This boundary keeps the public-facing project understandable while preserving
enough technical detail to reproduce the pipeline.
