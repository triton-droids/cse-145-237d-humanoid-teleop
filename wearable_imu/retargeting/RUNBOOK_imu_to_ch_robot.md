# IMU / Camera to ch_robot Retargeting Runbook

This runbook explains how to run retargeting, MuJoCo visualization, and 50 Hz
ZMQ mock-live replay from recorded IMU or camera datasets. The commands assume
you are on macOS, using the `hsretargeting` conda environment, with the repo at:

```bash
/Users/yanglin/Documents/UCSD/Clubs/Triton Droids/cse-145-237d-humanoid-teleop
```

## 1. Check Out the Correct Branch

```bash
cd "/Users/yanglin/Documents/UCSD/Clubs/Triton Droids/cse-145-237d-humanoid-teleop"
git fetch origin
git switch imu-retarget
git pull --ff-only origin imu-retarget
```

Confirm the current branch:

```bash
git status --short --branch
```

Expected output:

```text
## imu-retarget...origin/imu-retarget
```

## 2. Activate the Conda Environment

If conda shell integration is already enabled:

```bash
conda activate hsretargeting
```

If your shell cannot find `conda activate`:

```bash
source /Users/yanglin/.holosoma_deps/miniconda3/etc/profile.d/conda.sh
conda activate hsretargeting
```

Enter the wearable IMU workspace:

```bash
cd "/Users/yanglin/Documents/UCSD/Clubs/Triton Droids/cse-145-237d-humanoid-teleop/wearable_imu"
```

Quick dependency check:

```bash
python -c "import numpy, scipy, mujoco, zmq, matplotlib; print('deps ok')"
```

If a package is missing, try:

```bash
python -m pip install numpy scipy matplotlib mujoco pyzmq pytest
```

Or update from the environment file:

```bash
conda env update -f env/environment.yml
```

## 3. Available Input Data

Common input files:

```bash
../data/demo_1.npz
../data/human_joint_clip_20260601_231346.npz
../data/smplh_capture_3.jsonl
```

The `.npz` files are IMU handoff-style 9-joint position clips. The `.jsonl`
file is a camera SMPL-H lower-body capture. The loader converts both formats
into the same 9-joint handoff layout:

```text
Spine1, LeftUpLeg, LeftLeg, LeftFoot, LeftToeBase,
RightUpLeg, RightLeg, RightFoot, RightToeBase
```

## 4. Base Motion Modes

Recorded replay and ZMQ mock-live support:

```bash
--base-motion root_xy_forward
--base-motion root_xy
--base-motion fixed
--base-motion root_xyz
```

Meaning:

```text
root_xy_forward : Demo-friendly forward-walking mode. It preserves lateral motion and uses forward distance so the robot does not appear to walk backward when a clip doubles back.
root_xy         : Raw input root/pelvis horizontal displacement in the ch_robot frame.
fixed           : Keep the base fixed and only inspect leg retargeting. The robot walks in place.
root_xyz        : Raw input root/pelvis x/y/z displacement. Only use this if the source has reliable vertical root motion.
```

Frame convention:

```text
Human frame         : +X forward, +Y left, +Z up
ch_robot/MuJoCo    : -Y forward, +X left, +Z up
```

The code already converts root translation from the human frame into the
ch_robot/MuJoCo frame.

## 5. Offline MuJoCo Visualization: Camera JSONL

First run a no-window smoke test:

```bash
python demos/demo_mujoco_ch_robot_replay.py ../data/smplh_capture_3.jsonl --no-show --base-motion root_xy_forward
```

Expected summary:

```text
source     : smplh_camera_jsonl
frames     : 149
qpos       : (149, 17)
qvel       : (149, 16)
base motion: root_xy_forward
MuJoCo nq  : 17
MuJoCo nu  : 10
```

Open the MuJoCo viewer:

```bash
python demos/demo_mujoco_ch_robot_replay.py ../data/smplh_capture_3.jsonl --base-motion root_xy_forward
```

On macOS, MuJoCo passive viewer needs `mjpython`. The script should relaunch
itself with `mjpython` automatically. If you still see
`launch_passive requires mjpython`, run:

```bash
mjpython demos/demo_mujoco_ch_robot_replay.py ../data/smplh_capture_3.jsonl --base-motion root_xy_forward
```

## 6. Offline MuJoCo Visualization: Recorded IMU NPZ

No-window smoke test:

```bash
python demos/demo_mujoco_ch_robot_replay.py ../data/demo_1.npz --no-show --base-motion root_xy_forward
```

Open the viewer:

```bash
python demos/demo_mujoco_ch_robot_replay.py ../data/demo_1.npz --base-motion root_xy_forward
```

If you only want in-place leg motion:

```bash
python demos/demo_mujoco_ch_robot_replay.py ../data/demo_1.npz --base-motion fixed
```

Change playback speed:

```bash
python demos/demo_mujoco_ch_robot_replay.py ../data/demo_1.npz --base-motion root_xy_forward --speed 0.5
python demos/demo_mujoco_ch_robot_replay.py ../data/demo_1.npz --base-motion root_xy_forward --speed 2.0
```

Disable looping:

```bash
python demos/demo_mujoco_ch_robot_replay.py ../data/demo_1.npz --base-motion root_xy_forward --no-loop
```

## 7. Play a Human Joint Clip

Use this first when you want to inspect the recorded human 9-joint clip before
retargeting it to ch_robot. This script only accepts recorded `.npz` human
joint clips, not camera `.jsonl` captures.

Print a summary without opening a window:

```bash
python demos/demo_play_human_joint_clip.py ../data/demo_1.npz --no-show
```

Open the 3D human skeleton player:

```bash
python demos/demo_play_human_joint_clip.py ../data/demo_1.npz
```

Show pelvis-origin-relative joint positions instead of world positions:

```bash
python demos/demo_play_human_joint_clip.py ../data/demo_1.npz --origin
```

Change playback speed:

```bash
python demos/demo_play_human_joint_clip.py ../data/demo_1.npz --speed 0.5
python demos/demo_play_human_joint_clip.py ../data/demo_1.npz --speed 2.0
```

Useful interpretation:

```text
demo_play_human_joint_clip.py      : inspect the raw recorded human joint positions
demo_replay_ch_robot_retarget.py   : inspect human skeleton plus retargeted ch_robot joint angles
demo_mujoco_ch_robot_replay.py     : inspect the retargeted motion on the ch_robot MuJoCo model
```

## 8. Side-by-Side Human Clip and MuJoCo Robot

Use this when you want to see the before-retargeted human skeleton and the
after-retargeted ch_robot MuJoCo robot at the same time. The wrapper starts
both child demos and stops both when either window closes.

Basic before/after comparison:

```bash
python demos/demo_compare_human_clip_ch_robot.py ../data/demo_1.npz --base-motion root_xy_forward
```

Show the human skeleton in pelvis-origin coordinates while the robot uses
`root_xy_forward` base motion:

```bash
python demos/demo_compare_human_clip_ch_robot.py \
  ../data/demo_1.npz \
  --human-origin \
  --base-motion root_xy_forward
```

Slow down or speed up both windows together:

```bash
python demos/demo_compare_human_clip_ch_robot.py ../data/demo_1.npz --base-motion root_xy_forward --speed 0.5
python demos/demo_compare_human_clip_ch_robot.py ../data/demo_1.npz --base-motion root_xy_forward --speed 2.0
```

No-window smoke test for the combined command:

```bash
python demos/demo_compare_human_clip_ch_robot.py \
  ../data/demo_1.npz \
  --base-motion root_xy_forward \
  --no-show
```

This wrapper only accepts recorded human `.npz` clips. It does not accept
camera `.jsonl` captures because the raw human clip player only reads `.npz`.
For camera `.jsonl`, use the standalone MuJoCo replay or the Matplotlib
retargeting debug view.

## 9. Matplotlib Retargeting Debug View

This path does not open MuJoCo. It visualizes the human skeleton and shows the
resulting ch_robot joint angles. The right-side readout also shows the
retargeting input data before conversion: source human joint `xyz` values,
`root_pos_w`, and `root_quat_wxyz` when those arrays are present in the `.npz`.

Camera JSONL:

```bash
python demos/demo_replay_ch_robot_retarget.py ../data/smplh_capture_3.jsonl --base-motion root_xy_forward
```

IMU NPZ:

```bash
python demos/demo_replay_ch_robot_retarget.py ../data/demo_1.npz --base-motion root_xy_forward
```

This is the command for the view with the human skeleton and numeric readout on
the right:

```bash
python demos/demo_replay_ch_robot_retarget.py ../data/demo_1.npz --base-motion root_xy_forward
```

Hide the source input data and show only ch_robot joint angles:

```bash
python demos/demo_replay_ch_robot_retarget.py ../data/demo_1.npz --base-motion root_xy_forward --hide-input-data
```

Convert only and print a summary:

```bash
python demos/demo_replay_ch_robot_retarget.py ../data/smplh_capture_3.jsonl --no-show --base-motion root_xy_forward
```

Save the converted `qpos`/`qvel` as a replay NPZ:

```bash
python demos/demo_replay_ch_robot_retarget.py \
  ../data/smplh_capture_3.jsonl \
  --base-motion root_xy_forward \
  --save-output ../data/ch_robot_replay_qpos_smplh_capture_3.npz
```

Replay the saved `qpos`/`qvel` in MuJoCo:

```bash
python demos/demo_mujoco_ch_robot_replay.py ../data/ch_robot_replay_qpos_smplh_capture_3.npz
```

## 10. 50 Hz ZMQ Mock-Live: Camera JSONL

This is the closest recorded-data path to live operation. The publisher sends
each 9-joint position frame at 50 Hz. The subscriber retargets every incoming
frame into ch_robot `qpos`/`qvel` and updates MuJoCo online.

Terminal 1: start the MuJoCo ZMQ subscriber.

```bash
cd "/Users/yanglin/Documents/UCSD/Clubs/Triton Droids/cse-145-237d-humanoid-teleop/wearable_imu"
conda activate hsretargeting
python demos/demo_mujoco_ch_robot_zmq.py --endpoint tcp://127.0.0.1:5556 --base-motion root_xy_forward
```

Terminal 2: start the 50 Hz publisher.

```bash
cd "/Users/yanglin/Documents/UCSD/Clubs/Triton Droids/cse-145-237d-humanoid-teleop/wearable_imu"
conda activate hsretargeting
python demos/demo_zmq_human_joint_publisher.py ../data/smplh_capture_3.jsonl --endpoint tcp://127.0.0.1:5556 --fps 50
```

No-window smoke test:

Terminal 1:

```bash
python demos/demo_mujoco_ch_robot_zmq.py \
  --endpoint tcp://127.0.0.1:5565 \
  --no-show \
  --max-frames 10 \
  --status-every 1 \
  --base-motion root_xy_forward
```

Terminal 2:

```bash
python demos/demo_zmq_human_joint_publisher.py \
  ../data/smplh_capture_3.jsonl \
  --endpoint tcp://127.0.0.1:5565 \
  --fps 50 \
  --max-frames 10 \
  --no-loop \
  --start-delay-s 1.0 \
  --status-every 1
```

Expected subscriber output:

```text
base     : root_xy_forward
received=1 frame=0
...
received=10 frame=9
Converted 10 ZMQ frames.
```

## 11. 50 Hz ZMQ Mock-Live: Recorded IMU NPZ

Terminal 1:

```bash
python demos/demo_mujoco_ch_robot_zmq.py --endpoint tcp://127.0.0.1:5556 --base-motion root_xy_forward
```

Terminal 2:

```bash
python demos/demo_zmq_human_joint_publisher.py \
  ../data/demo_1.npz \
  --endpoint tcp://127.0.0.1:5556 \
  --fps 50
```

## 12. Real IMU Live Retargeting

The real ESP32/BNO085 IMU live path is currently orientation-only. It can emit
live joint rotations retargeted into ch_robot `qpos`/`qvel`, but it does not
have global root/pelvis position. That means the MuJoCo freejoint base remains
fixed for this direct IMU path. This is not a ZMQ mock-live limitation; it is a
sensor-input limitation.

Live `qpos`/`qvel` JSON lines:

```bash
python demos/demo_live_retarget.py --config shanks --output stdout --fps 100
```

Full 7-IMU config:

```bash
python demos/demo_live_retarget.py --config full --output stdout --fps 100
```

Send live retargeted frames to a UDP downstream process:

```bash
python demos/demo_live_retarget.py \
  --config full \
  --output udp \
  --target-host 127.0.0.1 \
  --target-port 6010 \
  --fps 100
```

Skip calibration:

```bash
python demos/demo_live_retarget.py --config shanks --output stdout --fps 100 --no-calibration
```

To make real live retargeting move across the MuJoCo floor like mock-live, add
a separate live root-position source, such as camera SMPL-H, VIO, mocap, or
foot odometry. Once that source exists, use the same base motion policy:

```text
root_position_source -> base_position_from_joint_points(...) or equivalent
-> legposes_to_qpos(..., base_position=...)
```

## 13. Demo Launcher

Open the GUI launcher:

```bash
python demos/demo_launcher.py
```

The launcher pre-fills the common local defaults, so for the retargeting demos
you can usually select a row and press `Run Demo` directly. The default dataset
paths are:

```text
Camera JSONL : ../data/smplh_capture_3.jsonl
Human NPZ    : ../data/demo_1.npz
ZMQ endpoint : tcp://127.0.0.1:5556
```

Common launcher entries:

```text
Replay ch_robot Retarget  : Matplotlib skeleton + joint-angle debug
MuJoCo ch_robot Replay    : offline MuJoCo replay
Compare Human + ch_robot  : raw human .npz skeleton and MuJoCo robot together
ZMQ Human Joint Publisher : mock-live publisher
MuJoCo ch_robot ZMQ       : mock-live subscriber + MuJoCo viewer
Live ch_robot Retarget    : real ESP32 IMU orientation-only bridge
Play Human Joint Clip     : raw recorded human .npz skeleton player
```

For replay, MuJoCo replay, and ZMQ subscriber entries, the launcher exposes a
`Base motion` option:

```text
root_xy_forward, root_xy, fixed, root_xyz
```

## 14. Model Cache and Floor

`demo_mujoco_ch_robot_replay.py` and `demo_mujoco_ch_robot_zmq.py` extract the
ch_robot MJCF and meshes from `origin/retargeting_holosoma` into:

```bash
wearable_imu/.cache/ch_robot_model
```

The cache is created automatically on first run. To force re-extraction:

```bash
python demos/demo_mujoco_ch_robot_replay.py ../data/smplh_capture_3.jsonl --no-show --refresh-model
```

The cached MuJoCo XML is automatically patched with a checker floor and a
viewer-only left/right foot-sole visual correction. To inspect the cached XML:

```bash
python -c "from pathlib import Path; print(Path('.cache/ch_robot_model/ch_robot_10dof.xml').read_text()[:1000])"
```

## 15. Validation Commands

Syntax check:

```bash
python -m py_compile \
  ik/ch_robot_retarget.py \
  demos/demo_mujoco_ch_robot_replay.py \
  demos/demo_replay_ch_robot_retarget.py \
  demos/demo_mujoco_ch_robot_zmq.py \
  demos/demo_launcher.py \
  ik/__init__.py
```

Retargeting tests:

```bash
python -m pytest tests/test_ch_robot_retarget.py -q
```

Full wearable_imu test suite:

```bash
python -m pytest -q
```

MuJoCo no-window smoke test:

```bash
python demos/demo_mujoco_ch_robot_replay.py ../data/smplh_capture_3.jsonl --no-show --base-motion root_xy_forward
```

ZMQ no-window smoke test, Terminal 1:

```bash
python demos/demo_mujoco_ch_robot_zmq.py \
  --endpoint tcp://127.0.0.1:5565 \
  --no-show \
  --max-frames 10 \
  --status-every 1 \
  --base-motion root_xy_forward
```

ZMQ no-window smoke test, Terminal 2:

```bash
python demos/demo_zmq_human_joint_publisher.py \
  ../data/smplh_capture_3.jsonl \
  --endpoint tcp://127.0.0.1:5565 \
  --fps 50 \
  --max-frames 10 \
  --no-loop \
  --start-delay-s 1.0 \
  --status-every 1
```

## 16. Troubleshooting

### `launch_passive requires mjpython`

On macOS, run the viewer with `mjpython`:

```bash
mjpython demos/demo_mujoco_ch_robot_replay.py ../data/smplh_capture_3.jsonl --base-motion root_xy_forward
```

For the ZMQ subscriber:

```bash
mjpython demos/demo_mujoco_ch_robot_zmq.py --base-motion root_xy_forward
```

### The Robot Still Walks in Place

Confirm you are not using fixed base motion:

```bash
python demos/demo_mujoco_ch_robot_replay.py ../data/smplh_capture_3.jsonl --no-show --base-motion root_xy_forward
```

The summary should include:

```text
base motion: root_xy_forward
```

If the input root/pelvis itself does not move, `root_xy_forward` and `root_xy`
cannot move the robot.
Check whether the camera or IMU data contains root displacement.

### ZMQ Publisher Bind Error

If you see:

```text
zmq.error.ZMQError: Operation not permitted
```

This is usually a sandbox or local TCP permission issue. Running the same
commands in your own terminal should normally avoid the restriction. You can
also use a different port:

```bash
python demos/demo_mujoco_ch_robot_zmq.py --endpoint tcp://127.0.0.1:5570 --base-motion root_xy_forward
python demos/demo_zmq_human_joint_publisher.py ../data/smplh_capture_3.jsonl --endpoint tcp://127.0.0.1:5570 --fps 50
```

### ZMQ Subscriber Receives No Frames

Start the subscriber first, then start the publisher. Confirm both terminals
use exactly the same endpoint:

```bash
tcp://127.0.0.1:5556
```

You can also verify with the no-window path:

```bash
python demos/demo_mujoco_ch_robot_zmq.py --endpoint tcp://127.0.0.1:5565 --no-show --max-frames 10 --status-every 1 --base-motion root_xy_forward
```

### MuJoCo Model Cache Is Broken or Floor Is Missing

Re-extract the model:

```bash
python demos/demo_mujoco_ch_robot_replay.py ../data/smplh_capture_3.jsonl --no-show --refresh-model --base-motion root_xy_forward
```

### Import Cannot Find `ik`

Run from the `wearable_imu/` directory:

```bash
cd "/Users/yanglin/Documents/UCSD/Clubs/Triton Droids/cse-145-237d-humanoid-teleop/wearable_imu"
python -m pytest tests/test_ch_robot_retarget.py -q
```

Do not run `wearable_imu/tests/...` directly from the repo root unless you set
`PYTHONPATH` manually.

## 17. Recommended Workflow

At the start of a session:

```bash
cd "/Users/yanglin/Documents/UCSD/Clubs/Triton Droids/cse-145-237d-humanoid-teleop"
git switch imu-retarget
git pull --ff-only origin imu-retarget
source /Users/yanglin/.holosoma_deps/miniconda3/etc/profile.d/conda.sh
conda activate hsretargeting
cd wearable_imu
python demos/demo_mujoco_ch_robot_replay.py ../data/smplh_capture_3.jsonl --no-show --base-motion root_xy_forward
```

If the no-window check passes, open visualization:

```bash
python demos/demo_mujoco_ch_robot_replay.py ../data/smplh_capture_3.jsonl --base-motion root_xy_forward
```

To compare raw human `.npz` skeleton and retargeted robot together:

```bash
python demos/demo_compare_human_clip_ch_robot.py ../data/demo_1.npz --base-motion root_xy_forward
```

To test the live-like pipeline:

```bash
python demos/demo_mujoco_ch_robot_zmq.py --endpoint tcp://127.0.0.1:5556 --base-motion root_xy_forward
```

In another terminal:

```bash
python demos/demo_zmq_human_joint_publisher.py ../data/smplh_capture_3.jsonl --endpoint tcp://127.0.0.1:5556 --fps 50
```
