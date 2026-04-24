# Sensor Workflow

This folder owns the data contract between hardware/simulation and calibration.

Expected input:

```text
QuaternionPacket:
    t
    sensor_id
    segment_hint
    quat
    quat_order
    frame_convention
    status_optional
```

Responsibilities:

- normalize quaternion order, such as `wxyz` vs `xyzw`
- define whether orientation means world-from-sensor or sensor-from-world
- preserve timestamps
- carry sensor IDs and segment hints
- reject or flag invalid packets

This layer should not estimate joints. It should make sensor data boring and
consistent before calibration sees it.
