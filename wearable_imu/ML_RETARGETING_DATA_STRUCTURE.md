# ML Retargeting Data Structure

This document defines the lower-body joint position data that the embedded
pipeline should hand to the ML retargeting pipeline.

The ML request is one frame shaped like:

```json
{
  "Spine1": [x, y, z],
  "LeftUpLeg": [x, y, z],
  "LeftLeg": [x, y, z],
  "LeftFoot": [x, y, z],
  "LeftToeBase": [x, y, z],
  "RightUpLeg": [x, y, z],
  "RightLeg": [x, y, z],
  "RightFoot": [x, y, z],
  "RightToeBase": [x, y, z]
}
```

`Spine1` is the pelvis/root. It is not a joint position in the same anatomical
sense as the hip, knee, or ankle points; it is the pelvis reference point.
`LeftToeBase` and `RightToeBase` are generated points because we do not mount
IMUs on the toes.

## Recommended Output

Use one recorded clip file per movement trial.

Preferred file format:

```text
.npz
```

Required arrays:

```text
joint_names:              string[J]
joint_pos_origin:         float[T, J, 3]
timestamps_s:             float[T]
fps:                      float[1]
```

Recommended additional arrays:

```text
joint_pos_w:              float[T, J, 3]
pelvis_ground_origin_w:   float[3]
frame0_pelvis_w:          float[3]
root_quat_wxyz:           float[T, 4]
required_segments:        string[N]
config:                   string[1]
foot_definition:          string[1]
```

Where:

```text
T = number of recorded frames
J = number of exported human points
N = number of required IMU segments for the selected recording mode
```

## Joint Order

Use this exact order for `joint_names` and the second dimension of
`joint_pos_origin` / `joint_pos_w`:

```text
0 Spine1
1 LeftUpLeg
2 LeftLeg
3 LeftFoot
4 LeftToeBase
5 RightUpLeg
6 RightLeg
7 RightFoot
8 RightToeBase
```

`Spine1` is the pelvis center/root. It is not a physical joint; it is the human
reference frame for the lower body.

## Coordinate Convention

Use meters.

The current lower-body code uses this world convention:

```text
x = forward
y = left
z = up
```

`joint_pos_w[t, j]` is the model/world-frame XYZ position of point `j` at frame
`t`.

`joint_pos_origin[t, j]` is the preferred ML handoff. It is expressed relative
to the frame-0 pelvis projected onto the floor:

```text
frame0_pelvis_w = joint_pos_w[0, Spine1]
pelvis_ground_origin_w = [frame0_pelvis_x, frame0_pelvis_y, 0]
joint_pos_origin[t, j] = joint_pos_w[t, j] - pelvis_ground_origin_w
```

For non-walking clips, it is acceptable for the pelvis to be fixed. In that
case `Spine1` stays fixed relative to `pelvis_ground_origin_w`.

For walking clips, any pelvis motion should be relative to the original frame-0
pelvis position:

```text
relative_pelvis_w[t] = joint_pos_w[t, Spine1] - frame0_pelvis_w
```

The current IMU-only pipeline does not directly measure pelvis translation, so
walking pelvis motion would need to come from another source or from a synthetic
trajectory.

## Current Mapping From Repo Code

The current skeleton aggregation code already creates the needed points.

Source:

```text
ik/lower_body_aggregation.py
```

Current skeleton joint names:

```text
pelvis
left_hip
right_hip
left_knee
right_knee
left_ankle
right_ankle
left_heel
left_toe
right_heel
right_toe
```

Handoff mapping:

```text
Spine1       <- skeleton.joints["pelvis"]
LeftUpLeg    <- skeleton.joints["left_hip"]
LeftLeg      <- skeleton.joints["left_knee"]
LeftFoot     <- skeleton.joints["left_ankle"]
LeftToeBase  <- skeleton.joints["left_toe"]
RightUpLeg   <- skeleton.joints["right_hip"]
RightLeg     <- skeleton.joints["right_knee"]
RightFoot    <- skeleton.joints["right_ankle"]
RightToeBase <- skeleton.joints["right_toe"]
```

The toe base points are generated toe points from the body model, not directly
measured anatomical joints.

## IMU Segment IDs

The embedded packet layer identifies both the physical board and body segment.

Source:

```text
sensor/packet.py
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

`sensor_id` means the physical ESP32 board.

`segment_id` means the body location for the current session.

The ML data should be organized by `segment_id` / body point, not by physical
ESP32 board ID.

## Recording Modes

The live partial-IMU viewer already defines three useful segment sets.

Source:

```text
demos/demo_partial_imu_live_viewer.py
```

Recommended modes:

```text
thighs:
  pelvis, left_thigh, right_thigh

shanks:
  pelvis, left_thigh, right_thigh, left_shank, right_shank

full:
  pelvis, left_thigh, left_shank, left_foot,
  right_thigh, right_shank, right_foot
```

For the current 5-ESP32 setup, use `shanks`.

## Measured vs Estimated Points

The aggregation function supports partial sensor sets.

If a distal segment is missing, it fills that segment with the proximal
orientation:

```text
missing shank -> use thigh orientation
missing foot  -> use shank orientation
```

This means the exported XYZ positions are always complete, but some points may
be model-estimated rather than fully measured.

For a `shanks` recording:

```text
measured orientation inputs:
  pelvis, left_thigh, right_thigh, left_shank, right_shank

estimated orientation inputs:
  left_foot, right_foot

generated points:
  LeftToeBase, RightToeBase are toe points generated from the shank/foot model
```

For an ML handoff, save `required_segments` and `config` so they know how much
of the skeleton came from real IMUs.

## Important Limitation

With only IMU orientation data, the system does not directly measure global
pelvis translation.

The current model places the pelvis/root at a fixed model position unless an
external `pelvis_center` is supplied. The joint XYZ values are therefore
orientation-derived kinematic positions, not full motion-capture world
positions with walking translation.

That is still useful for non-walking lower-body retargeting. If the ML team
needs `Spine1` to move through world space during walking, that requires another
source of root translation or a synthetic root trajectory. Future `Spine1`
positions should be expressed relative to the original frame-0 pelvis position.

## Example Data Shape

For a 10 second recording at 30 FPS:

```text
T = 300
J = 9

joint_names.shape             = (9,)
joint_pos_origin.shape        = (300, 9, 3)
joint_pos_w.shape             = (300, 9, 3)
timestamps_s.shape            = (300,)
pelvis_ground_origin_w.shape  = (3,)
root_quat_wxyz.shape          = (300, 4)
```

Example `joint_names`:

```python
[
    "Spine1",
    "LeftUpLeg",
    "LeftLeg",
    "LeftFoot",
    "LeftToeBase",
    "RightUpLeg",
    "RightLeg",
    "RightFoot",
    "RightToeBase",
]
```

Example frame:

```text
joint_pos_origin[0] =
[
  [spine1_x,        spine1_y,        spine1_z],
  [left_upleg_x,    left_upleg_y,    left_upleg_z],
  [left_leg_x,      left_leg_y,      left_leg_z],
  [left_foot_x,     left_foot_y,     left_foot_z],
  [left_toebase_x,  left_toebase_y,  left_toebase_z],
  [right_upleg_x,   right_upleg_y,   right_upleg_z],
  [right_leg_x,     right_leg_y,     right_leg_z],
  [right_foot_x,    right_foot_y,    right_foot_z],
  [right_toebase_x, right_toebase_y, right_toebase_z],
]
```

## Recommended CLI Contract

The recorder should accept:

```text
--host 0.0.0.0
--port 5005
--config shanks
--duration-s 10
--fps 30
--output data/recordings/example_walk.npz
```

The expected command from the project root is:

```bash
conda run --no-capture-output -n humanoid-sim python demos/demo_record_human_joint_clip.py --config shanks --duration-s 10 --fps 30 --output data/recordings/example_walk.npz
```

## Handoff Summary

Send ML a `.npz` with:

```text
joint_names
joint_pos_origin
joint_pos_w
timestamps_s
fps
pelvis_ground_origin_w
frame0_pelvis_w
root_quat_wxyz
config
required_segments
foot_definition
```

The core required payload is `joint_names + joint_pos_origin + timestamps_s`.
The extra fields make it much easier for ML to debug, retarget, and compare
clips across different IMU configurations.
