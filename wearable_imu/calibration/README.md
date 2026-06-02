# Calibration Workflow

This folder is for turning strapped IMU orientations into segment orientations.

The real-world issue is that sensors are not perfectly mounted. A thigh or
shank IMU can rotate around the limb, and the strap may not land the same way
twice.

Initial calibration plan:

```text
neutral standing capture
    -> collect fresh packets for all 7 segment_id values
    -> average sensor quaternions per segment
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

## Current Code

`calibration.neutral.NeutralCalibrationAccumulator` can already build a neutral
standing profile from synthetic or real `QuaternionPacket` samples.

Current behavior:

- collect samples per `segment_id`
- require a minimum sample count per required segment
- average neutral quaternions per segment
- record which physical `sensor_id` supplied each segment
- apply live orientations relative to neutral

This is enough to test the first calibration behavior before hardware arrives.

Identity assumption:

- calibration profiles should bind to `segment_id`, because that is the body
  placement being calibrated.
- profiles may also record `sensor_id`, because that helps diagnose a flaky or
  swapped physical ESP32-S3.

Important limitation: quaternions alone do not determine leg length. Segment
lengths should come from manual measurement, user height ratios, or external
constraints.
