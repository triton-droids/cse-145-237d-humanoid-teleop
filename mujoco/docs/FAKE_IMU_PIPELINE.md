# Fake IMU Pipeline

## Objective

This pipeline exists to simulate the future IMU hardware interface before the
real ESP32/BNO085 stack arrives.

The main goal is:

1. define a clean Jetson-side packet format
2. generate fake data for all 7 IMUs
3. keep the fake source swappable with real hardware later

It is **not** meant to be the final teleoperation controller.

## IMUs

The canonical IMU names are:

- `waist`
- `left_thigh`
- `left_shin`
- `left_foot`
- `right_thigh`
- `right_shin`
- `right_foot`

## Packet Schema

The script emits JSON frames with schema name:

- `triton_humanoid_imu_frame/v1`

Each frame contains:

- `schema`
- `source`
- `frame_index`
- `sim_time_sec`
- `stamp_sec`
- `imus`

Each IMU entry contains:

- `name`
- `frame_id`
- `stamp_sec`
- `frame_index`
- `qx`
- `qy`
- `qz`
- `qw`

Quaternion order matches the planned BNO085 fused orientation ordering:

- `qx, qy, qz, qw`

## Recommended Commands

Start the fake hardware source only:

```bash
python3 fake_imu_pipeline.py --real-time
```

Print one example frame schema and exit:

```bash
python3 fake_imu_pipeline.py --show-schema
```

Write a JSONL log of fake IMU frames:

```bash
python3 fake_imu_pipeline.py --real-time --print-every 0 --dump-jsonl fake_imu_frames.jsonl
```

Use MuJoCo only for sanity visualization:

```bash
mjpython fake_imu_pipeline.py --viewer --real-time
```

`--viewer` root-locks the base by default so you can inspect the leg motion
without needing a balance controller.

## Real Hardware Replacement

When the ESP32/BNO085 pipeline is ready:

1. replace the fake generator with a packet reader
2. keep the Jetson-side schema and IMU names unchanged
3. keep the quaternion-to-joint conversion code if it is still useful

The intended swap point is the fake source in:

- [fake_imu_pipeline.py](../fake_imu_pipeline.py)

The rest of the frame format should stay stable.

## ESP32 Note

The hardware can still be split across 3 ESP32s. The important part is that the
Jetson should normalize incoming data into the canonical IMU names above before
any downstream conversion or simulation code runs.
