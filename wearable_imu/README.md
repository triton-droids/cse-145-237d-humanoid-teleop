# Real-World Lower-Body IMU Pose Estimation

## Goal
Build a real-time lower-body pose estimator for wearable IMUs.

Source branch history:
[`inverse-kinematics`](https://github.com/triton-droids/cse-145-237d-humanoid-teleop/tree/inverse-kinematics).
See [`source-branch.md`](source-branch.md) for the branch-to-folder mapping.

The target hardware path is:

```text
BNO085 quaternion streams on the active ESP32-S3 nodes
    -> direct Wi-Fi UDP to laptop receiver
    -> sensor packet normalization
    -> neutral-pose and functional calibration
    -> sensor-to-segment alignment
    -> inverse kinematics
    -> estimated pelvis/leg/foot pose
    -> visualization and evaluation
```

MuJoCo is now a supporting tool, not the center of the project. It remains useful
as a synthetic test harness for fake IMU data, but the main workflow is the
real-world wearable pipeline.

## Current Sensor Assumption
The BNO085 provides fused orientation quaternions directly. Our current input
contract is therefore quaternion packets, not raw accel/gyro fusion.

Each ESP32-S3 streams directly to the laptop receiver over UDP. (A robot-side
Jetson is the eventual target once we move onto the humanoid; until then the
receiver is whatever laptop is on the same hotspot.) The pelvis ESP32 is not the
hub in the first architecture; it is just another sensor node.

Expected packet shape:

```text
QuaternionPacket:
    magic/version
    sensor_id
    segment_id
    sequence
    sensor_time_us
    quat_wxyz
    accuracy
    status
    report_type
```

Current packet format is a 40-byte little-endian binary UDP payload with `IMUQ`
magic and `wxyz` quaternion order.

Identity rules:

- `sensor_id` means which physical ESP32-S3 board sent the packet.
- `segment_id` means where that board is mounted for this session.
- calibration should bind primarily to `segment_id`.
- diagnostics should also track `sensor_id` so a bad physical board can be found.

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

The software contract supports the full seven-segment lower-body set, and the
demos expose three selectable subsets (see `PARTIAL_CONFIGS` in
`demos/demo_partial_imu_live_viewer.py`):

```text
thighs (3 nodes): pelvis, left_thigh, right_thigh
shanks (5 nodes): pelvis, left_thigh, right_thigh, left_shank, right_shank
full   (7 nodes): pelvis + both thigh/shank/foot
```

The current hardware MVP runs the five-node `shanks` set (pelvis, both thighs,
both shanks; feet estimated as neutral). All subsets use the same packet format
and segment IDs.

## Wearable Placement
Default placement for the lower-body setup:

- pelvis: front center, around belt line
- thigh: lateral/outside thigh, around mid-thigh
- shank: lateral/outside lower leg, around mid-shank
- foot: top of foot, around midfoot/laces

For quaternion-only IK, exact sensor position matters less than orientation. It
will matter more later when we simulate or use accelerometer signals.

Frame convention: the body/world frame is `+X` forward, `+Y` left, `+Z` up; the
BNO085 sensor frame is `+X` device top, `+Y` device left edge, `+Z` out of the
chip face. The fixed per-segment sensor-to-segment mount rotations and exact
mounting orientations are defined in `ik/imu_orientation.py` and `model/`.

## Calibration Comes First
Real sensors are not fixed to bones. They can be rotated on the strap, mounted
slightly differently every session, and disturbed by soft tissue motion. Before
IK, we need calibration.

Calibration sequence (implemented):

```text
1. stand neutral for 2-5 seconds
2. average each sensor quaternion as the neutral reference
3. express live orientations relative to that neutral reference
```

The `calibration.neutral.CalibrationProfile` captured per session contains:

```text
CalibrationProfile:
    neutral_orientations   # averaged neutral quaternion per segment_id
    sensor_ids             # which physical board supplied each segment
    sample_counts          # samples averaged per segment
```

The profile is held in memory for the running session; it is not yet persisted
to disk. Richer functional calibration is planned but not implemented:

```text
planned (not yet implemented):
    sensor_to_segment_rotations   # strap-twist correction
    leg_heading_corrections       # thigh/shank heading alignment
    knee_axis_estimates           # functional knee-hinge calibration
    user_height / segment_lengths # anthropometric scaling
```

## Folder Workflows
| Folder | Workflow | Purpose |
|---|---|---|
| `hardware/` | Real device IO | BNO085 connection, device IDs, live quaternion acquisition |
| `sensor/` | Sensor packet layer | Quaternion packet types, frame conventions, timestamps, validity |
| `calibration/` | Real-world alignment | Neutral pose, strap twist correction, functional calibration |
| `ik/` | Pose estimation | Convert calibrated segment orientations into joint estimates |
| `model/` | Body model | Segment definitions, dimensions, IMU placement assumptions |
| `simulator/` | Archived test harness | MuJoCo synthetic body and fake IMU quaternion generator |
| `evaluation/` | Accuracy checks | Compare estimates to known ground truth or labeled captures |
| `demos/` | Runnable workflows | Small scripts that connect modules for inspection |
| `data/` | Captures and outputs | Calibration captures, synthetic logs, plots, visualizations |
| `tests/` | Verification | Unit tests and consistency checks |
| `env/` | Environment setup | Conda environment definition |

## Current Useful Commands
Because local shell activation has been unreliable, prefer `conda run`:

```powershell
conda run -p .\.conda python -m pytest -q
```

Archived MuJoCo viewer:

```powershell
conda run -p .\.conda python demos\demo_mujoco_lower_body_viewer.py
```

Orientation IK demo:

```powershell
conda run -p .\.conda python demos\demo_imu_orientation_ik.py
```

## Development Direction
The next real work should happen in this order:

```text
1. flash one ESP32-S3/BNO085 node and receive UDP packets on the laptop
2. flash the active 5-node MVP set, or all 7 nodes for the full lower-body set,
   with unique sensor_id and segment_id settings
3. tune quaternion spike filtering with real packet traces
4. validate neutral standing calibration
5. add thigh/shank heading alignment
6. add functional knee-hinge calibration
7. route calibrated quaternions into IK
8. keep MuJoCo only as a fake-data generator and regression harness
```

The guiding rule: the estimator should be designed for messy straps and real
people first, then tested with simulation as a convenience.

## Current Real-World Pipeline Code

Implemented so far:

- ESP32-S3/BNO085 UDP packet firmware scaffold
- receiver-side binary quaternion packet parser
- latest-packet UDP receiver buffer
- quaternion norm/stale/spike filtering
- SLERP smoothing
- neutral standing calibration profile from quaternion samples

Not implemented yet:

- thigh/shank heading alignment
- functional knee-hinge calibration
- magnetometer disturbance gating
- live calibrated packet-to-IK integration
