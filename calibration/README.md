# Calibration Workflow

This folder is for turning strapped IMU orientations into segment orientations.

The real-world issue is that sensors are not perfectly mounted. A thigh or
shank IMU can rotate around the limb, and the strap may not land the same way
twice.

Initial calibration plan:

```text
neutral standing capture
    -> average sensor quaternions
    -> define session reference
    -> align thigh/shank headings
    -> estimate sensor-to-segment correction
```

Functional calibration plan:

```text
slow knee flexion or squat samples
    -> estimate knee hinge axis
    -> reduce off-axis thigh/shank disagreement
    -> refine strap twist correction
```

Saved output should become a `CalibrationProfile` containing neutral
quaternions, sensor-to-segment rotations, heading corrections, estimated joint
axes, and user/body dimensions.
