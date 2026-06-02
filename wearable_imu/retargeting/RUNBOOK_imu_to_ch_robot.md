# IMU / Camera to ch_robot Retargeting Runbook

本 runbook 說明如何從 recorded IMU / camera dataset 跑 retargeting、MuJoCo
visualization、以及 50 Hz ZMQ mock-live replay。命令假設你在 macOS、使用
`hsretargeting` conda env，並且 repo 在：

```bash
/Users/yanglin/Documents/UCSD/Clubs/Triton Droids/cse-145-237d-humanoid-teleop
```

## 1. 切到正確分支

```bash
cd "/Users/yanglin/Documents/UCSD/Clubs/Triton Droids/cse-145-237d-humanoid-teleop"
git fetch origin
git switch imu-retarget
git pull --ff-only origin imu-retarget
```

確認分支：

```bash
git status --short --branch
```

預期會看到：

```text
## imu-retarget...origin/imu-retarget
```

## 2. 啟動 conda env

如果 `conda` shell integration 已啟用：

```bash
conda activate hsretargeting
```

如果你的 shell 找不到 `conda activate`：

```bash
source /Users/yanglin/.holosoma_deps/miniconda3/etc/profile.d/conda.sh
conda activate hsretargeting
```

進入 wearable IMU workspace：

```bash
cd "/Users/yanglin/Documents/UCSD/Clubs/Triton Droids/cse-145-237d-humanoid-teleop/wearable_imu"
```

快速確認 Python package：

```bash
python -c "import numpy, scipy, mujoco, zmq, matplotlib; print('deps ok')"
```

如果缺 package，先試：

```bash
python -m pip install numpy scipy matplotlib mujoco pyzmq pytest
```

或用環境檔更新：

```bash
conda env update -f env/environment.yml
```

## 3. 可用資料

目前常用 input：

```bash
../data/human_joint_clip_20260601_231345.npz
../data/human_joint_clip_20260601_231346.npz
../data/smplh_capture_3.jsonl
```

`.npz` 是 IMU handoff style 的 9-joint position clip。`.jsonl` 是 camera
SMPL-H lower-body capture，loader 會轉成同一個 9-joint handoff layout：

```text
Spine1, LeftUpLeg, LeftLeg, LeftFoot, LeftToeBase,
RightUpLeg, RightLeg, RightFoot, RightToeBase
```

## 4. Base Motion 選項

recorded replay 和 ZMQ mock-live 都支援：

```bash
--base-motion root_xy
--base-motion fixed
--base-motion root_xyz
```

含義：

```text
root_xy  : 使用 input root/pelvis 的水平位移，讓 MuJoCo freejoint 在地板上移動。建議預設。
fixed    : base 固定，只看腿部 retargeting，robot 會原地走。
root_xyz : 使用 root/pelvis 的 x/y/z 位移；只有 source 的 z motion 可信時才用。
```

注意：human frame 是 `+X forward, +Y left, +Z up`，ch_robot/MuJoCo frame
是 `+Y forward, +X right, +Z up`。程式已經會把 root translation 轉到
ch_robot/MuJoCo frame。

## 5. 離線 MuJoCo visualization：camera JSONL

先做 no-window smoke test：

```bash
python demos/demo_mujoco_ch_robot_replay.py ../data/smplh_capture_3.jsonl --no-show --base-motion root_xy
```

預期 summary 類似：

```text
source     : smplh_camera_jsonl
frames     : 149
qpos       : (149, 17)
qvel       : (149, 16)
base motion: root_xy
MuJoCo nq  : 17
MuJoCo nu  : 10
```

開 MuJoCo viewer：

```bash
python demos/demo_mujoco_ch_robot_replay.py ../data/smplh_capture_3.jsonl --base-motion root_xy
```

macOS 上 MuJoCo passive viewer 需要 `mjpython`。目前 script 會自動用
`mjpython` relaunch；如果你仍看到 `launch_passive requires mjpython`，
直接跑：

```bash
mjpython demos/demo_mujoco_ch_robot_replay.py ../data/smplh_capture_3.jsonl --base-motion root_xy
```

## 6. 離線 MuJoCo visualization：recorded IMU NPZ

no-window smoke test：

```bash
python demos/demo_mujoco_ch_robot_replay.py ../data/human_joint_clip_20260601_231345.npz --no-show --base-motion root_xy
```

開 viewer：

```bash
python demos/demo_mujoco_ch_robot_replay.py ../data/human_joint_clip_20260601_231345.npz --base-motion root_xy
```

如果你只想看 in-place leg motion：

```bash
python demos/demo_mujoco_ch_robot_replay.py ../data/human_joint_clip_20260601_231345.npz --base-motion fixed
```

調播放速度：

```bash
python demos/demo_mujoco_ch_robot_replay.py ../data/human_joint_clip_20260601_231345.npz --base-motion root_xy --speed 0.5
python demos/demo_mujoco_ch_robot_replay.py ../data/human_joint_clip_20260601_231345.npz --base-motion root_xy --speed 2.0
```

不 loop：

```bash
python demos/demo_mujoco_ch_robot_replay.py ../data/human_joint_clip_20260601_231345.npz --base-motion root_xy --no-loop
```

## 7. Matplotlib retargeting debug view

這個不開 MuJoCo，只看 human skeleton 和輸出的 ch_robot joint angles。

Camera JSONL：

```bash
python demos/demo_replay_ch_robot_retarget.py ../data/smplh_capture_3.jsonl --base-motion root_xy
```

IMU NPZ：

```bash
python demos/demo_replay_ch_robot_retarget.py ../data/human_joint_clip_20260601_231345.npz --base-motion root_xy
```

只轉換並印 summary：

```bash
python demos/demo_replay_ch_robot_retarget.py ../data/smplh_capture_3.jsonl --no-show --base-motion root_xy
```

把 qpos/qvel 存成 replay NPZ：

```bash
python demos/demo_replay_ch_robot_retarget.py \
  ../data/smplh_capture_3.jsonl \
  --base-motion root_xy \
  --save-output ../data/ch_robot_replay_qpos_smplh_capture_3.npz
```

再用 MuJoCo replay 讀存好的 qpos：

```bash
python demos/demo_mujoco_ch_robot_replay.py ../data/ch_robot_replay_qpos_smplh_capture_3.npz
```

## 8. 50 Hz ZMQ mock-live：camera JSONL

這是最接近 live 的 recorded-data path：publisher 用 50 Hz 餵每幀 9-joint
position，subscriber 每收到一幀就立刻 retarget 成 ch_robot qpos/qvel，並更新
MuJoCo。

Terminal 1：啟動 MuJoCo ZMQ subscriber。

```bash
cd "/Users/yanglin/Documents/UCSD/Clubs/Triton Droids/cse-145-237d-humanoid-teleop/wearable_imu"
conda activate hsretargeting
python demos/demo_mujoco_ch_robot_zmq.py --endpoint tcp://127.0.0.1:5556 --base-motion root_xy
```

Terminal 2：啟動 50 Hz publisher。

```bash
cd "/Users/yanglin/Documents/UCSD/Clubs/Triton Droids/cse-145-237d-humanoid-teleop/wearable_imu"
conda activate hsretargeting
python demos/demo_zmq_human_joint_publisher.py ../data/smplh_capture_3.jsonl --endpoint tcp://127.0.0.1:5556 --fps 50
```

如果要只做 no-window smoke test：

Terminal 1：

```bash
python demos/demo_mujoco_ch_robot_zmq.py \
  --endpoint tcp://127.0.0.1:5565 \
  --no-show \
  --max-frames 10 \
  --status-every 1 \
  --base-motion root_xy
```

Terminal 2：

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

預期 subscriber 會印：

```text
base     : root_xy
received=1 frame=0
...
received=10 frame=9
Converted 10 ZMQ frames.
```

## 9. 50 Hz ZMQ mock-live：recorded IMU NPZ

Terminal 1：

```bash
python demos/demo_mujoco_ch_robot_zmq.py --endpoint tcp://127.0.0.1:5556 --base-motion root_xy
```

Terminal 2：

```bash
python demos/demo_zmq_human_joint_publisher.py \
  ../data/human_joint_clip_20260601_231345.npz \
  --endpoint tcp://127.0.0.1:5556 \
  --fps 50
```

## 10. 真實 IMU live retargeting

真實 ESP32/BNO085 IMU live path 目前是 orientation-only。它可以即時輸出
joint rotations -> ch_robot qpos/qvel，但沒有全域 root/pelvis position，所以
freejoint base 目前會固定；這不是 ZMQ mock-live 的限制，而是 sensor input 沒有
translation source。

如果你只要 live qpos/qvel JSON lines：

```bash
python demos/demo_live_retarget.py --config shanks --output stdout --fps 100
```

full 7-IMU config：

```bash
python demos/demo_live_retarget.py --config full --output stdout --fps 100
```

送到 UDP downstream process：

```bash
python demos/demo_live_retarget.py \
  --config full \
  --output udp \
  --target-host 127.0.0.1 \
  --target-port 6010 \
  --fps 100
```

跳過 calibration：

```bash
python demos/demo_live_retarget.py --config shanks --output stdout --fps 100 --no-calibration
```

如果要讓真實 live 也像 mock-live 一樣在 MuJoCo 地板上移動，需要額外 live
root-position source，例如 camera SMPL-H、VIO、mocap、或 foot odometry。接上後
應使用同一套 base motion policy：

```text
root_position_source -> base_position_from_joint_points(...) or equivalent
-> legposes_to_qpos(..., base_position=...)
```

## 11. Demo launcher

如果你想用 GUI launcher：

```bash
python demos/demo_launcher.py
```

常用項目：

```text
Replay ch_robot Retarget  : Matplotlib skeleton + joint-angle debug
MuJoCo ch_robot Replay    : offline MuJoCo replay
ZMQ Human Joint Publisher : mock-live publisher
MuJoCo ch_robot ZMQ       : mock-live subscriber + MuJoCo viewer
Live ch_robot Retarget    : real ESP32 IMU orientation-only bridge
```

對 replay / MuJoCo / ZMQ subscriber，launcher 右側有 `Base motion`：

```text
root_xy, fixed, root_xyz
```

## 12. Model cache and floor

`demo_mujoco_ch_robot_replay.py` 和 `demo_mujoco_ch_robot_zmq.py` 會從
`origin/retargeting_holosoma` 抽出 ch_robot MJCF 和 meshes 到：

```bash
wearable_imu/.cache/ch_robot_model
```

第一次執行會自動建立 cache。若要重新抽取 model：

```bash
python demos/demo_mujoco_ch_robot_replay.py ../data/smplh_capture_3.jsonl --no-show --refresh-model
```

MuJoCo XML cache 會自動加入 checker floor。如果你想確認 XML：

```bash
python -c "from pathlib import Path; print(Path('.cache/ch_robot_model/ch_robot_10dof.xml').read_text()[:1000])"
```

## 13. Validation commands

語法檢查：

```bash
python -m py_compile \
  ik/ch_robot_retarget.py \
  demos/demo_mujoco_ch_robot_replay.py \
  demos/demo_replay_ch_robot_retarget.py \
  demos/demo_mujoco_ch_robot_zmq.py \
  demos/demo_launcher.py \
  ik/__init__.py
```

retargeting tests：

```bash
python -m pytest tests/test_ch_robot_retarget.py -q
```

完整 wearable_imu tests：

```bash
python -m pytest -q
```

MuJoCo no-window smoke：

```bash
python demos/demo_mujoco_ch_robot_replay.py ../data/smplh_capture_3.jsonl --no-show --base-motion root_xy
```

ZMQ no-window smoke，Terminal 1：

```bash
python demos/demo_mujoco_ch_robot_zmq.py \
  --endpoint tcp://127.0.0.1:5565 \
  --no-show \
  --max-frames 10 \
  --status-every 1 \
  --base-motion root_xy
```

ZMQ no-window smoke，Terminal 2：

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

## 14. Troubleshooting

### `launch_passive requires mjpython`

在 macOS 上直接用：

```bash
mjpython demos/demo_mujoco_ch_robot_replay.py ../data/smplh_capture_3.jsonl --base-motion root_xy
```

或 ZMQ subscriber：

```bash
mjpython demos/demo_mujoco_ch_robot_zmq.py --base-motion root_xy
```

### Robot 仍然原地走

確認你不是用 fixed base：

```bash
python demos/demo_mujoco_ch_robot_replay.py ../data/smplh_capture_3.jsonl --no-show --base-motion root_xy
```

summary 應該包含：

```text
base motion: root_xy
```

若 input 本身 root/pelvis 沒有移動，`root_xy` 也不會讓 robot 移動。可以先看
camera / IMU data 的 root displacement 是否存在。

### ZMQ publisher bind error

如果看到：

```text
zmq.error.ZMQError: Operation not permitted
```

通常是 sandbox / local TCP 權限問題。你在自己的 terminal 直接跑通常不會有這個
限制。也可以換 port：

```bash
python demos/demo_mujoco_ch_robot_zmq.py --endpoint tcp://127.0.0.1:5570 --base-motion root_xy
python demos/demo_zmq_human_joint_publisher.py ../data/smplh_capture_3.jsonl --endpoint tcp://127.0.0.1:5570 --fps 50
```

### ZMQ subscriber 沒收到 frame

先開 subscriber，再開 publisher。確認兩邊 endpoint 完全一樣：

```bash
tcp://127.0.0.1:5556
```

也可用 no-window 版本確認：

```bash
python demos/demo_mujoco_ch_robot_zmq.py --endpoint tcp://127.0.0.1:5565 --no-show --max-frames 10 --status-every 1 --base-motion root_xy
```

### MuJoCo model cache 壞掉或沒有 floor

重新抽取 model：

```bash
python demos/demo_mujoco_ch_robot_replay.py ../data/smplh_capture_3.jsonl --no-show --refresh-model --base-motion root_xy
```

### Import 找不到 `ik`

請從 `wearable_imu/` 目錄跑：

```bash
cd "/Users/yanglin/Documents/UCSD/Clubs/Triton Droids/cse-145-237d-humanoid-teleop/wearable_imu"
python -m pytest tests/test_ch_robot_retarget.py -q
```

不要從 repo root 直接跑 `wearable_imu/tests/...`，除非你手動設定
`PYTHONPATH`。

## 15. Recommended workflow

每天開始先跑：

```bash
cd "/Users/yanglin/Documents/UCSD/Clubs/Triton Droids/cse-145-237d-humanoid-teleop"
git switch imu-retarget
git pull --ff-only origin imu-retarget
source /Users/yanglin/.holosoma_deps/miniconda3/etc/profile.d/conda.sh
conda activate hsretargeting
cd wearable_imu
python demos/demo_mujoco_ch_robot_replay.py ../data/smplh_capture_3.jsonl --no-show --base-motion root_xy
```

如果 no-show 通過，再開 visualization：

```bash
python demos/demo_mujoco_ch_robot_replay.py ../data/smplh_capture_3.jsonl --base-motion root_xy
```

如果要測「像 live 一樣」的 pipeline：

```bash
python demos/demo_mujoco_ch_robot_zmq.py --endpoint tcp://127.0.0.1:5556 --base-motion root_xy
```

另一個 terminal：

```bash
python demos/demo_zmq_human_joint_publisher.py ../data/smplh_capture_3.jsonl --endpoint tcp://127.0.0.1:5556 --fps 50
```
