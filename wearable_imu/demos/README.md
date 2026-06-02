# Demos

Runnable scripts that wire multiple modules together so you can see the pipeline
working. Core logic stays in the workflow folders (`sensor/`, `calibration/`,
`ik/`, `model/`, `simulator/`); demos are the practical, integrated entry points.

> **Setup first.** These scripts need the project environment. See
> [Environment Setup](../README.md#environment-setup) in the main README for the
> conda and venv options. With the env active, run the commands below from the
> **project root**.

## The launcher (easiest start)

The launcher is a clickable control panel: pick a demo from the list, set its
arguments on the right, and use Run/Stop. It starts each demo as a separate
process and streams its output.

Each demo exposes its relevant arguments directly in the panel:

- **Radio buttons** for fixed choices (e.g. the IMU set: thighs / shanks / full).
- **Entry fields** for values like host, port, record FPS, draw FPS, record
  duration, output path, or a clip path. Leaving a field on its greyed
  placeholder uses the script's own default.
- **Checkboxes** for on/off flags (e.g. `--origin` in the clip player).
- An **Extra args** box is still there for anything not surfaced as a control.

```bash
python demos/demo_launcher.py
python demos/demo_launcher.py --list   # print demo keys + their controls, no GUI
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
| `demo_play_human_joint_clip.py` | Plays back a recorded `.npz` joint clip as an animated 3D skeleton (Play/Pause + scrub slider). | positional `clip`, `--origin`, `--speed`, `--no-show` |
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
| `demo_live_retarget.py` | Headless live Plan A bridge: IMU packets -> calibrated lower-body skeleton -> ch_robot `qpos[17]` and `qvel[16]`, emitted as JSON lines over stdout or UDP. | `--config {thighs,shanks,full}`, `--output {stdout,udp,none}`, `--target-host`, `--target-port`, `--fps`, `--yaw-mode {keep,strip}` |
| `demo_replay_ch_robot_retarget.py` | Replay a recorded `human_joint_clip_*.npz` or camera `.jsonl` clip through Plan A, visualize the skeleton, and show source human/root values plus resulting ch_robot joint angles. | positional `clip`, `--save-output`, `--frame-key {joint_pos_origin,joint_pos_w}`, `--yaw-mode {keep,strip}`, `--base-motion {root_xy,fixed,root_xyz}`, `--hide-input-data`, `--no-show` |
| `demo_mujoco_ch_robot_replay.py` | Replay a recorded IMU/camera handoff clip or saved ch_robot qpos replay on the real Holosoma `ch_robot_10dof.xml` MuJoCo model. Auto-extracts XML/meshes from `retargeting_holosoma` into `.cache/`. | positional `input`, `--speed`, `--frame-key {joint_pos_origin,joint_pos_w}`, `--yaw-mode {keep,strip}`, `--base-motion {root_xy,fixed,root_xyz}`, `--no-show`, `--refresh-model` |
| `demo_compare_human_clip_ch_robot.py` | Open the raw human `.npz` skeleton player and retargeted ch_robot MuJoCo replay together for before/after inspection. | positional `clip`, `--speed`, `--human-origin`, `--frame-key {joint_pos_origin,joint_pos_w}`, `--yaw-mode {keep,strip}`, `--base-motion {root_xy,fixed,root_xyz}`, `--no-show` |
| `demo_zmq_human_joint_publisher.py` | Publish a recorded `human_joint_clip_*.npz` or camera `.jsonl` as mock-live 9-joint frames over ZeroMQ at 50 Hz or a chosen rate. | positional `clip`, `--endpoint`, `--fps`, `--frame-key {joint_pos_origin,joint_pos_w}`, `--max-frames` |
| `demo_mujoco_ch_robot_zmq.py` | Subscribe to mock-live ZMQ human joint frames, convert each frame to ch_robot `qpos/qvel`, and update the real MuJoCo robot online. | `--endpoint`, `--yaw-mode {keep,strip}`, `--base-motion {root_xy,fixed,root_xyz}`, `--no-show`, `--max-frames` |
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

# Live ch_robot qpos/qvel JSON lines for a downstream policy process
python demos/demo_live_retarget.py --config shanks --output stdout --fps 100

# Live ch_robot qpos/qvel over UDP
python demos/demo_live_retarget.py --config full --output udp --target-host 127.0.0.1 --target-port 6010

# Replay a recorded IMU handoff clip through ch_robot retargeting with visualization
python demos/demo_replay_ch_robot_retarget.py ../data/demo_1.npz

# Run the recorded IMU handoff on the actual ch_robot MuJoCo model from retargeting_holosoma
python demos/demo_mujoco_ch_robot_replay.py ../data/demo_1.npz --base-motion root_xy

# Open raw human skeleton and retargeted ch_robot MuJoCo replay together
python demos/demo_compare_human_clip_ch_robot.py ../data/demo_1.npz --base-motion root_xy

# Or run a saved qpos/qvel replay on the actual ch_robot MuJoCo model
python demos/demo_mujoco_ch_robot_replay.py ../data/ch_robot_replay_qpos_20260601_231345.npz

# Mock-live ZMQ stream from a recorded dataset at 50 Hz
python demos/demo_zmq_human_joint_publisher.py ../data/demo_1.npz --fps 50

# Mock-live ZMQ stream from a camera JSONL capture at 50 Hz
python demos/demo_zmq_human_joint_publisher.py ../data/smplh_capture_3.jsonl --fps 50

# In another terminal, consume that ZMQ stream and drive the actual ch_robot MuJoCo model
python demos/demo_mujoco_ch_robot_zmq.py --base-motion root_xy

# Base motion note:
#   root_xy/root_xyz need a live root position source, such as the mock-live
#   recorded joint stream above or a future camera/VIO/foot-odometry source.
#   The direct ESP32 IMU live bridge is orientation-only today, so its base
#   intentionally stays fixed.

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
