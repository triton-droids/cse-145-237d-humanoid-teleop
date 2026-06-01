# IK Workflow

This folder estimates lower-body joint pose from calibrated segment
orientations.

Current prototype:

- `imu_orientation.py` estimates hip, knee, and ankle rotations from segment
  IMU quaternions.
- `planar_leg.py` is a small toy planar IK solver and should stay separate from
  the real wearable path.
- `lower_body_aggregation.py` combines seven calibrated segment orientations
  into one pelvis/legs/feet skeleton with joint positions and relative joint
  rotations.

The real-world input should be built from seven calibrated quaternion streams:

```text
calibrated segment orientations
    pelvis
    left_thigh
    left_shank
    left_foot
    right_thigh
    right_shank
    right_foot
```

The output should be joint estimates plus confidence/residual information. The
solver should respect anatomical constraints instead of trusting every raw
quaternion equally.

The IK layer should not parse UDP packets and should not know about ESP32 board
IDs. It should consume calibrated segment orientations after `sensor/` and
`calibration/` have already normalized the data.

Aggregation demo:

```powershell
conda run --no-capture-output -n humanoid-sim python demos\demo_lower_body_aggregation.py
```
