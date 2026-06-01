# Real-World Lower-Body IMU Pose Estimation

## Goal
Build a real-time lower-body pose estimator for wearable IMUs.

The target hardware path is:

```text
BNO085 quaternion streams on 7 ESP32-S3 nodes
    -> Wi-Fi UDP over the shared phone hotspot
    -> laptop or other receiver on the same hotspot
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

Each ESP32-S3 streams directly to the receiver over UDP. The phone hotspot is
only the Wi-Fi network; it is not the packet hub. The pelvis ESP32 is also just
another sensor node.

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

The calibration profile should contain:

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
conda run --no-capture-output -n humanoid-sim python -m pytest -q
```

Demo launcher:

```powershell
conda run --no-capture-output -n humanoid-sim python demos\demo_launcher.py
```

Archived MuJoCo viewer:

```powershell
conda run --no-capture-output -n humanoid-sim python demos\demo_mujoco_lower_body_viewer.py
```

Orientation IK demo:

```powershell
conda run --no-capture-output -n humanoid-sim python demos\demo_imu_orientation_ik.py
```

## Development Direction
The next real work should happen in this order:

```text
1. flash one ESP32-S3/BNO085 node and receive UDP packets on the laptop over the phone hotspot
2. flash all 7 nodes with unique sensor_id and segment_id settings
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
