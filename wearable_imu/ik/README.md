# IK Workflow

This folder is where we estimate lower-body joint pose from calibrated segment
orientations.

Current prototype:

- `imu_orientation.py` estimates hip, knee, and ankle rotations from segment
  IMU quaternions.
- `planar_leg.py` is a small toy planar IK solver and should stay separate from
  the real wearable path.
- `lower_body_aggregation.py` combines seven calibrated segment orientations
  into one pelvis/legs/feet skeleton with joint positions and relative joint
  rotations.

Our real-world input comes from seven calibrated quaternion streams:

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

We output joint estimates plus confidence/residual information. Our solver
respects anatomical constraints instead of trusting every raw quaternion equally.

Our IK layer doesn't parse UDP packets and doesn't know about ESP32 board
IDs. It consumes calibrated segment orientations after `sensor/` and
`calibration/` have already normalized the data.

Aggregation demo:

```powershell
conda run -p .\.conda python demos\demo_lower_body_aggregation.py
```
