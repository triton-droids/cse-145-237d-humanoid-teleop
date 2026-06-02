# ch_robot Retargeting Converter Update

## Summary

We added `ch_robot` support to the Holosoma retargeting converter and verified the full pipeline from retargeted motion to IsaacLab/RL tracking data.

The retargeter output is not the final format consumed by IsaacLab. Our retargeter produces a compact motion file with:

```text
qpos
fps
human_joints
cost
```

For `ch_robot`, `qpos` has shape `(T, 17)`:

```text
7 floating-base values + 10 ch_robot joint positions = 17
```

The converter then replays this motion through the MuJoCo model and exports the IsaacLab/RL tracking format:

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

## Code Changes

### Added ch_robot joint order

We added a default joint order for `ch_robot` to the converter:

```python
"ch_robot": [
    "left_hip1_joint",
    "left_hip2_joint",
    "left_thigh_joint",
    "left_knee_joint",
    "left_ankle_joint",
    "right_hip1_joint",
    "right_hip2_joint",
    "right_thigh_joint",
    "right_knee_joint",
    "right_ankle_joint",
]
```

This is required so the converter can map retargeted `qpos[:, 7:]` values into the MuJoCo robot joint order.

### Removed the hardcoded G1 DOF assumption

Previously, the converter sliced robot joint positions with:

```python
motion[:, 7:36]
```

That assumes a 29-DOF robot and only works for G1:

```text
36 - 7 = 29
```

`ch_robot` has 10 DOF, so we updated the converter to slice based on the selected robot's joint count:

```python
motion[:, 7 : 7 + robot_dof]
```

For `ch_robot`, this becomes:

```text
motion[:, 7:17]
```

### Fixed FPS parsing

The retargeted `.npz` stores `fps` as an FPS value, for example:

```text
fps = 30
```

We updated the converter to handle FPS values like `30` correctly instead of treating them as timestep-like values.

### Added headless conversion

We added `--headless` support:

```bash
--headless
```

This allows batch conversion without launching the MuJoCo viewer. The converter still calls `mj_forward()` and records body positions, orientations, and velocities.

### Fixed setup scripts for paths with spaces

Our local workspace path contains a space:

```text
Triton Droids
```

Some setup scripts used unquoted shell variables, which broke path resolution. We quoted path variables in:

```text
scripts/setup_retargeting.sh
scripts/source_retargeting_setup.sh
```

## Validation

We tested the update with both synthetic data and real processed OMOMO data.

### Synthetic test

Our synthetic test produced a valid retargeted `ch_robot` file:

```text
demo_results/ch_robot/minimal/tiny_stand.npz
```

Verified:

```text
keys: ['qpos', 'human_joints', 'fps', 'cost']
qpos: (5, 17)
fps: 30
human_joints: (5, 52, 3)
```

The converter then produced:

```text
converted_res/ch_robot/minimal/tiny_stand_mj_fps50.npz
```

Verified:

```text
joint_pos: (7, 17)
joint_vel: (7, 16)
body_pos_w: (7, 28, 3)
body_quat_w: (7, 28, 4)
body_lin_vel_w: (7, 28, 3)
body_ang_vel_w: (7, 28, 3)
joint_names: (10,)
body_names: (28,)
fps: [50]
```

### Real OMOMO test

We downloaded the processed OMOMO dataset as:

```text
OMOMO_new.zip
```

We extracted:

```text
demo_data/OMOMO_new/sub10_largebox_049.pt
```

The full sequence has 222 frames. Direct retargeting to `ch_robot` became infeasible around frame 158:

```text
RuntimeError: CVXPY solve failed: infeasible
```

This is expected for some sequences because `ch_robot` has only 10 DOF and may not satisfy all retargeting constraints across the entire motion.

To validate the full pipeline with real data, we clipped the first 120 frames:

```text
demo_data/OMOMO_new/sub10_largebox_049_clip120.pt
```

Retargeting output:

```text
demo_results/ch_robot/robot_only/omomo/sub10_largebox_049_clip120.npz
```

Verified:

```text
keys: ['qpos', 'human_joints', 'fps', 'cost']
qpos: (120, 17)
fps: 30
human_joints: (120, 52, 3)
```

Converter output:

```text
converted_res/ch_robot/robot_only/omomo/sub10_largebox_049_clip120_mj_fps50.npz
```

Verified:

```text
joint_pos: (199, 17)
joint_vel: (199, 16)
body_pos_w: (199, 28, 3)
body_quat_w: (199, 28, 4)
body_lin_vel_w: (199, 28, 3)
body_ang_vel_w: (199, 28, 3)
joint_names: (10,)
body_names: (28,)
fps: [50]
```

`joint_vel` has one fewer base dimension than `joint_pos` because MuJoCo free-joint velocity is 6D while free-joint position is 7D:

```text
joint_pos = 7 base qpos + 10 joints = 17
joint_vel = 6 base qvel + 10 joints = 16
```

## Commands

### Retarget real OMOMO clip to ch_robot

```bash
python examples/robot_retarget.py \
  --data_path demo_data/OMOMO_new \
  --task-type robot_only \
  --task-name sub10_largebox_049_clip120 \
  --data_format smplh \
  --robot ch_robot \
  --save-dir demo_results/ch_robot/robot_only/omomo \
  --retargeter.fix-orientation
```

### Convert retargeted motion to IsaacLab/RL tracking format

```bash
python data_conversion/convert_data_format_mj.py \
  --input-file demo_results/ch_robot/robot_only/omomo/sub10_largebox_049_clip120.npz \
  --robot ch_robot \
  --data-format smplh \
  --object-name ground \
  --output-fps 50 \
  --output-name converted_res/ch_robot/robot_only/omomo/sub10_largebox_049_clip120_mj_fps50.npz \
  --headless \
  --once
```

### Inspect converter output

```bash
python - <<'PY'
import numpy as np

p = "converted_res/ch_robot/robot_only/omomo/sub10_largebox_049_clip120_mj_fps50.npz"
d = np.load(p, allow_pickle=True)

print("file:", p)
print("keys:", d.files)
for key in [
    "joint_pos",
    "joint_vel",
    "body_pos_w",
    "body_quat_w",
    "body_lin_vel_w",
    "body_ang_vel_w",
    "joint_names",
    "body_names",
    "fps",
]:
    print(key, d[key].shape)

print("joint_names:", d["joint_names"].tolist())
print("fps:", d["fps"].tolist())
PY
```
