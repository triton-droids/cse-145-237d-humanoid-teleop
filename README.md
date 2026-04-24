# Real-World Lower-Body IMU Pose Estimation

## Goal
Build a real-time lower-body pose estimator for wearable IMUs.

The target hardware path is:

```text
BNO085 quaternion streams
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
The BNO085 can provide fused orientation quaternions directly. Our near-term
input should therefore be quaternion packets, not raw accel/gyro fusion.

Expected packet shape:

```text
QuaternionPacket:
    t
    sensor_id
    segment_hint
    quat_wxyz or quat_xyzw
    status_optional
```

The exact quaternion convention must be normalized at the sensor boundary before
the estimator sees it.

## Wearable Placement
Default placement for the lower-body setup:

- pelvis: front center, around belt line
- thigh: lateral/outside thigh, around mid-thigh
- shank: lateral/outside lower leg, around mid-shank
- foot: top of foot, around midfoot/laces

For quaternion-only IK, exact sensor position matters less than orientation. It
will matter more later when we simulate or use accelerometer signals.

## Calibration Comes First
Real sensors are not fixed to bones. They can be rotated on the strap, mounted
slightly differently every session, and disturbed by soft tissue motion. Before
IK, we need calibration.

Recommended first calibration sequence:

```text
1. stand neutral for 2-5 seconds
2. average each sensor quaternion as the neutral reference
3. align thigh/shank headings within each leg
4. do slow knee flexion or squat samples
5. estimate strap twist corrections from the knee hinge constraint
6. save a calibration profile
```

The calibration profile should eventually contain:

```text
CalibrationProfile:
    user_height_optional
    segment_lengths
    neutral_sensor_quaternions
    sensor_to_segment_rotations
    leg_heading_corrections
    knee_axis_estimates
    ankle_axis_estimates_optional
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
1. define BNO085-like quaternion packet objects
2. add neutral standing calibration
3. add thigh/shank heading alignment
4. add functional knee-hinge calibration
5. route calibrated quaternions into IK
6. keep MuJoCo only as a fake-data generator and regression harness
```

The guiding rule: the estimator should be designed for messy straps and real
people first, then tested with simulation as a convenience.
