# Hardware Workflow

This folder is for real wearable device input.

Near-term target:

```text
BNO085 sensors
    -> live quaternion reports
    -> sensor_id mapping
    -> timestamped packets
    -> sensor/ normalization layer
```

The BNO085 can provide fused quaternion reports directly. We should preserve
the raw reported quaternion and metadata, then normalize ordering and frame
meaning in `sensor/`.

Responsibilities:

- connect to physical sensors
- assign stable sensor IDs to body segments
- read quaternion reports
- attach timestamps and quality/status information
- pass packets forward without doing IK or calibration
