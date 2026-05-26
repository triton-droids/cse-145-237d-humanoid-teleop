# Sensor Workflow

This folder owns the data contract between hardware/simulation and calibration.

Expected input:

```text
QuaternionPacket:
    sensor_id
    segment_id
    sequence
    sensor_time_us
    quat_wxyz
    accuracy
    status
    report_type
    receive_time_s
```

Responsibilities:

- enforce `wxyz` quaternion order at the packet boundary
- define whether orientation means world-from-sensor or sensor-from-world before calibration
- preserve timestamps
- carry `sensor_id` and `segment_id`
- reject or flag invalid packets
- reject obvious quaternion spikes before calibration
- smooth accepted orientations with quaternion SLERP

This layer should not estimate joints. It should make sensor data boring and
consistent before calibration sees it.

## Current Binary UDP Contract

The first ESP32 firmware sends one 40-byte little-endian packet per quaternion:

```text
magic[4]          "IMUQ"
version           uint8, currently 1
sensor_id         uint8
segment_id        uint8
quat_order        uint8, 1 means wxyz
sequence          uint32
sensor_time_us    uint32 from ESP32 micros()
qw qx qy qz       float32
accuracy          float32 from BNO085 rotation-vector report
status            uint8 from BNO085 event status
report_type       uint8, 1 means rotation vector
reserved          uint16
```

The Python parser is `sensor.packet.parse_quaternion_packet`.

Current identity assumptions:

- `sensor_id` identifies the physical ESP32-S3 board.
- `segment_id` identifies the body placement.
- valid lower-body segment IDs are `0..6`.
- `255` means unknown/unassigned.

Run the first Jetson-side receiver demo:

```powershell
conda run -p .\.conda python demos\demo_udp_quaternion_receiver.py
```

## Current Filtering

`sensor.filtering.QuaternionPacketFilter` currently handles:

- quaternion norm validation
- unknown segment rejection
- stale sequence rejection
- angular spike rejection
- quaternion SLERP smoothing

This can be tested without hardware by creating synthetic `QuaternionPacket`
objects.
