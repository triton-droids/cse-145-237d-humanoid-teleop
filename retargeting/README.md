# Retargeting

This folder contains the project-specific retargeting contract for the Triton humanoid robot. It does not vendor the full Holosoma source tree.

## Role in the Pipeline

```text
wearable_imu/ or perception/depth_camera/
    -> human lower-body pose / keypoints
    -> retargeting to Triton humanoid joint targets
    -> converted motion-tracking data
    -> simulation repository policy training
```

The dedicated simulation code lives in:

https://github.com/triton-droids/simulation

## Contents

| Path | Purpose |
|---|---|
| `ch_robot_contract.py` | Joint order and shape contract for Triton humanoid retargeting files. |
| `holosoma-ch-robot-converter-update.md` | Notes from the CH robot Holosoma converter update and validation runs. |
| `patches/0001-Add-ch_robot-support-to-retargeting-converter.patch` | Minimal Holosoma patch that adds `ch_robot` converter support. |
| `examples/retargeting_contract_example.json` | Small example of the expected contract and output shapes. |

## Retargeted Motion Contract

The retargeter output is a compact `.npz` file with:

```text
qpos
fps
human_joints
cost
```

For `ch_robot`, `qpos` has width 17:

```text
7 floating-base values + 10 robot joint positions = 17
```

The 10 robot joints are ordered as:

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

## Converted Motion-Tracking Contract

The converter replays the retargeted motion through the MuJoCo robot model and exports the Isaac Lab / RL tracking format:

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

For `ch_robot`, `joint_pos` has width 17 and `joint_vel` has width 16 because MuJoCo free-joint position is 7D while free-joint velocity is 6D.

## Holosoma Integration

Apply the patch in `patches/` to a Holosoma checkout instead of copying Holosoma into this repository. The patch adds:

- `ch_robot` default joint order
- dynamic robot DOF slicing instead of hardcoded G1 29-DOF slicing
- FPS parsing for both timestep-like and FPS-like values
- `--headless` conversion mode for batch runs
- path quoting fixes in setup scripts

The robot meshes and full MuJoCo/URDF assets should stay in the simulation or Holosoma worktree, not in this course repository.
