# Demos

Runnable scripts that wire multiple modules together so you can see the pipeline
working. Core logic stays in the workflow folders (`sensor/`, `calibration/`,
`ik/`, `model/`, `simulator/`); demos are the practical, integrated entry points.

> **Setup first.** These scripts need the project environment. See
> [Environment Setup](../README.md#environment-setup) in the main README for the
> conda and venv options. With the env active, run the commands below from the
> **project root**.

## The launcher (easiest start)

The launcher is a clickable control panel: pick a demo from the list, edit its
arguments, and use Run/Stop. It starts each demo as a separate process and
streams its output.

```bash
python demos/demo_launcher.py
python demos/demo_launcher.py --list   # print demo keys, no GUI
```

Or, with conda and without activating the env:

```bash
conda run --no-capture-output -n humanoid-sim python demos/demo_launcher.py
```

---

## No hardware required (synthetic data)

These run entirely on generated data — good for verifying your install and for
development.

| Script | What it does | Useful flags |
|---|---|---|
| `demo_live_lower_body_aggregation.py` | Animated synthetic walking skeleton (Matplotlib 3D). | `--frames N`, `--no-show` (headless, saves a PNG), `--interval-ms` |
| `demo_lower_body_aggregation.py` | Aggregates one static seven-segment pose and prints joint positions + rotations. | — |
| `demo_imu_orientation_ik.py` | Recovers hip/knee/ankle rotations from synthetic mounted-IMU orientations. | — |
| `demo_inverse_kinematics.py` | Toy planar-leg IK solver. | `--plot`, `--x`, `--z`, `--angle`, `--knee-direction` |
| `demo_visualize_lower_body_model.py` | Renders the body model + IMU mount frames to a PNG. | `--output PATH` |
| `demo_mujoco_lower_body_viewer.py` | MuJoCo viewer with control + live-estimator windows, driven by **simulated** IMUs. Needs a display. | `--animate`, `--preset {standing,squat,step}`, `--no-control-window`, `--no-estimator-window` |

Examples:

```bash
python demos/demo_live_lower_body_aggregation.py
python demos/demo_inverse_kinematics.py --plot
python demos/demo_live_lower_body_aggregation.py --no-show --frames 3   # smoke test
```

---

## With hardware (live ESP32-S3 / BNO085 over UDP)

These consume live UDP packets from the ESP32 nodes. The receiver and the nodes
must be on the same Wi-Fi network, and each node's `JETSON_IP` must point at the
receiver (see [`hardware/README.md`](../hardware/README.md)).

| Script | What it does | Key flags |
|---|---|---|
| `demo_partial_imu_live_viewer.py` | Live lower-body skeleton from real IMU packets; missing distal segments are estimated (dashed). Includes Calibrate, Clear calibration, Record, and Stop rec buttons. In the launcher this is a single entry — pick the IMU set with the **IMU set** radio buttons on the right. | `--config {thighs,shanks,full}`, `--host`, `--port`, `--max-age-ms`, `--min-samples`, `--record-duration-s`, `--record-fps`, `--record-output` |
| `demo_record_human_joint_clip.py` | Offline recorder that saves ML retargeting joint positions (`Spine1`, hips, knees, ankles, and generated toe points) to `.npz`. | `--config {thighs,shanks,full}`, `--duration-s`, `--fps`, `--output` |
| `demo_udp_quaternion_receiver.py` | Text-only packet monitor: per-segment rate, age, receive/sensor jitter, drops, raw quaternions. | `--host`, `--port`, `--max-age-ms` |
| `demo_udp_latency_ping.py` | Round-trip UDP latency test to one node. | positional `esp32_ip`, `--port`, `--count`, `--interval-ms`, `--timeout-ms` |

Examples:

```bash
# Full 7-IMU live skeleton
python demos/demo_partial_imu_live_viewer.py --host 0.0.0.0 --port 5005 --config full

# Five-IMU live skeleton + in-window recorder
python demos/demo_partial_imu_live_viewer.py --config shanks --record-duration-s 10 --record-output data/recordings/example_walk.npz

# Headless-style offline recorder without the live viewer
python demos/demo_record_human_joint_clip.py --config shanks --duration-s 10 --fps 30 --output data/recordings/example_walk.npz

# Check packets are arriving before launching the viewer
python demos/demo_udp_quaternion_receiver.py --host 0.0.0.0 --port 5005

# Measure latency to one node (IP from the ESP32 Serial Monitor)
python demos/demo_udp_latency_ping.py 192.168.1.164 --port 5006
```

**Partial IMU configs:**

- `thighs` — pelvis + left/right thigh (3 IMUs); shanks and feet are estimated
- `shanks` — adds both shanks (5 IMUs); feet are estimated
- `full` — all seven lower-body segments

In the viewer, **solid lines are measured** segments and **dashed lines are
estimated** (neutral-joint assumption). The 3D view opens immediately showing the
raw, uncalibrated pose, and the live view keeps running the whole time:

- **Calibrate** button (or press **C**/**R**) — capture a neutral standing
  reference; calibration runs in the background so the view never freezes.
- **Clear calibration** button — drop the profile and return to the raw pose.
- **Record** button (or press **Space**) — after calibration, record the live
  calibrated skeleton to an ML `.npz` clip using the `Spine1`, `LeftUpLeg`,
  `LeftLeg`, `LeftFoot`, `LeftToeBase`, `RightUpLeg`, `RightLeg`,
  `RightFoot`, `RightToeBase` joint order.
- **Stop rec** button (or press **Esc**) — save the clip early with whatever
  valid frames have been captured.
- **Front / Rear / Left / Right** buttons — snap the camera to that face of the
  pelvis/body (`+X` forward, `+Y` left, `+Z` up). You can still drag to orbit
  freely; periodic redraws pause while you rotate or move the window to avoid
  flicker.

---

## Notes

- Most demos accept `--help` for the full argument list, e.g.
  `python demos/demo_inverse_kinematics.py --help`.
- The live UDP demos default to `0.0.0.0:5005` for incoming quaternion packets
  and `5006` for latency pings, matching the firmware defaults in
  [`hardware/`](../hardware/).
