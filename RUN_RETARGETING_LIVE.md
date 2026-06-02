# Live Leg Retargeting Setup And Run Guide

This guide sets up and runs the `ch_robot` live leg-only retargeting receiver, plus a fake publisher that streams replay data over ZeroMQ.

## 1. Install On A New Computer

From the parent workspace:

```bash
cd /workspace
git clone <repo-url> cse-145-237d-humanoid-teleop
cd /workspace/cse-145-237d-humanoid-teleop/holosoma-main
```

Install the retargeting environment:

```bash
CONDA_ENV_NAME=hsretargeting bash scripts/setup_retargeting.sh
```

Activate it:

```bash
source scripts/source_retargeting_setup.sh
```

Use `source scripts/source_retargeting_setup.sh` every time you open a new shell. It also clears bad `python`, `pip`, `python3`, and `pip3` aliases.

## 2. Verify The Install

```bash
cd /workspace/cse-145-237d-humanoid-teleop/holosoma-main/src/holosoma_retargeting/holosoma_retargeting

python -m py_compile \
  examples/live_leg_retarget.py \
  examples/live_leg_publisher.py
```

## 3. Start The Live Receiver

Terminal 1:

```bash
source /workspace/cse-145-237d-humanoid-teleop/holosoma-main/scripts/source_retargeting_setup.sh

cd /workspace/cse-145-237d-humanoid-teleop/holosoma-main/src/holosoma_retargeting/holosoma_retargeting

python examples/live_leg_retarget.py \
  --input-mode zmq \
  --zmq-endpoint tcp://127.0.0.1:5556 \
  --robot ch_robot \
  --visualize \
  --solver-iters 3 \
  --init-solver-iters 20 \
  --save-path /tmp/live_leg_qpos.npz
```

Open the Viser URL printed by the script, usually:

```text
http://localhost:8080
```

If running on a remote machine over SSH, forward the Viser port from your local computer:

```bash
ssh -L 8080:localhost:8080 user@machine
```

Before any incoming frame arrives, the robot holds the MJCF default pose. Between incoming frames, it keeps holding the latest retargeted command.

## 4. Start A Fake Publisher From LAFAN

Terminal 2:

```bash
source /workspace/cse-145-237d-humanoid-teleop/holosoma-main/scripts/source_retargeting_setup.sh

cd /workspace/cse-145-237d-humanoid-teleop/holosoma-main/src/holosoma_retargeting/holosoma_retargeting

python examples/live_leg_publisher.py \
  --replay-path /cephfs/holosoma/data/lafan/processed_npy/dance2_subject1.npy \
  --zmq-endpoint tcp://127.0.0.1:5556 \
  --replay-start 3135 \
  --replay-count 500 \
  --fps 30
```

If `/cephfs/holosoma/data/lafan/processed_npy/dance2_subject1.npy` does not exist on the new computer, mount CephFS or copy the `.npy` locally and replace `--replay-path`.

## 5. Run Without Viser

Use this if the Viser websocket logs are distracting or if you only want saved qpos output:

```bash
python examples/live_leg_retarget.py \
  --input-mode zmq \
  --zmq-endpoint tcp://127.0.0.1:5556 \
  --robot ch_robot \
  --no-visualize \
  --save-path /tmp/live_leg_qpos.npz
```

## 6. Replay Directly Without ZeroMQ

This is useful for debugging the retargeter without a publisher:

```bash
python examples/live_leg_retarget.py \
  --input-mode replay-npy \
  --replay-path /cephfs/holosoma/data/lafan/processed_npy/dance2_subject1.npy \
  --replay-start 3135 \
  --replay-count 500 \
  --robot ch_robot \
  --visualize \
  --save-path /tmp/live_leg_replay_qpos.npz
```

## 7. Live Input Format

The live retargeter expects 9 keypoints per frame:

```python
{
    "Spine1":       [x, y, z],
    "LeftUpLeg":    [x, y, z],
    "LeftLeg":      [x, y, z],
    "LeftFoot":     [x, y, z],
    "LeftToeBase":  [x, y, z],
    "RightUpLeg":   [x, y, z],
    "RightLeg":     [x, y, z],
    "RightFoot":    [x, y, z],
    "RightToeBase": [x, y, z],
}
```

The values should be in meters and Z-up.

The ZeroMQ protocol sends a NumPy array, not the dict directly. Convert the dict into this fixed order:

```python
frame = np.array([
    data["Spine1"],
    data["LeftUpLeg"],
    data["LeftLeg"],
    data["LeftFoot"],
    data["LeftToeBase"],
    data["RightUpLeg"],
    data["RightLeg"],
    data["RightFoot"],
    data["RightToeBase"],
], dtype=np.float32)
```

The expected shape is:

```python
(9, 3)
```

Then publish it with:

```python
import time

from holosoma_retargeting.examples.live_leg_retarget import (
    LIVE_LEG_JOINTS,
    encode_zmq_frame,
)

socket.send_multipart(
    encode_zmq_frame(
        frame,
        frame_idx=i,
        timestamp=time.time(),
        joint_names=LIVE_LEG_JOINTS,
    )
)
```

## 8. Mapping Used By Retargeting

For `live_legs + ch_robot`, the mapping is intentionally swapped left/right to match the current robot marker convention:

```text
Spine1       -> pelvis_marker
LeftUpLeg    -> right_hip_marker
LeftLeg      -> right_knee_marker
LeftFoot     -> right_ankle_marker
LeftToeBase  -> right_foot_sphere_3_link
RightUpLeg   -> left_hip_marker
RightLeg     -> left_knee_marker
RightFoot    -> left_ankle_marker
RightToeBase -> left_foot_sphere_3_link
```

