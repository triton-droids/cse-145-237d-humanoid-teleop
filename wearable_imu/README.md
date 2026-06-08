# Real-World Lower-Body IMU Pose Estimation

Real-time lower-body pose estimation from wearable IMUs. Seven BNO085 sensors on
ESP32-S3 nodes stream orientation quaternions over Wi-Fi/UDP to a compute node
(a laptop, or any receiver on the same network), which calibrates them, solves
joint angles, and renders a live lower-body skeleton.

```text
BNO085 quaternions on 7 ESP32-S3 nodes
    -> Wi-Fi UDP over the shared phone hotspot
    -> laptop or other receiver on the same hotspot
    -> sensor packet normalization      (sensor/)
    -> neutral-pose + functional calibration (calibration/)
    -> sensor-to-segment alignment      (calibration/)
    -> orientation inverse kinematics    (ik/)
    -> estimated pelvis/leg/foot pose
    -> visualization and evaluation      (demos/, evaluation/)
```

The phone hotspot is only the Wi-Fi network; it is not the packet hub. Each
ESP32-S3 streams directly to the receiver, and the pelvis node is just another
sensor — so one dropped sensor does not block the rest.

You do **not** need any hardware to try this out. Several demos run on synthetic
data, so you can install the environment and see a moving skeleton in minutes.
MuJoCo is a supporting tool here (a synthetic-data generator and regression
harness), not the center of the project.

---

## Table of Contents

- [Quick Start](#quick-start)
- [Environment Setup](#environment-setup)
  - [Option A: Miniconda / Conda (recommended)](#option-a-miniconda--conda-recommended)
  - [Option B: Raw Python venv (no conda)](#option-b-raw-python-venv-no-conda)
- [Running the Demos](#running-the-demos)
- [Running the Tests](#running-the-tests)
- [Repository Layout](#repository-layout)
- [Hardware & Wearable Placement](#hardware--wearable-placement)
- [Sensor Data Contract](#sensor-data-contract)
- [Calibration](#calibration)
- [Development Direction](#development-direction)

---

## Quick Start

If you already have a working Python 3.11 environment with the dependencies
installed (see [Environment Setup](#environment-setup)), the fastest way to see
something move is the synthetic skeleton demo — no hardware required:

```bash
# from the project root
python demos/demo_live_lower_body_aggregation.py
```

Or open the clickable launcher that lists every demo with a Run/Stop button:

```bash
python demos/demo_launcher.py
```

For the full IMU/camera -> ch_robot MuJoCo runbook, including all command
lines for offline replay, 50 Hz ZMQ mock-live, and troubleshooting, see
[`retargeting/RUNBOOK_imu_to_ch_robot.md`](retargeting/RUNBOOK_imu_to_ch_robot.md).

---

## Environment Setup

The project targets **Python 3.11**. Pick **one** of the two options below.
Option A mirrors the original project setup; Option B is lighter if you do not
want conda.

> **Why 3.11?** Some dependencies (notably `mujoco` and `scipy`) may not yet
> ship prebuilt wheels for the very newest Python releases. Python 3.11 is the
> tested, known-good version.

The runtime dependencies are listed in [`requirements.txt`](requirements.txt):
`numpy`, `scipy`, `matplotlib`, `mujoco`, `pyzmq`, and `pytest`. (`tkinter`,
used by the GUI demos, ships with Python and is not a pip package — see the
per-platform note below.)

### Option A: Miniconda / Conda (recommended)

Conda gives you an isolated Python 3.11 without touching your system Python, and
also provides `tkinter` automatically.

**1. Install Miniconda** (skip if you already have conda or Anaconda):

| Platform | Install |
|---|---|
| **Linux** | `curl -fsSL -o ~/miniconda.sh https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh && bash ~/miniconda.sh -b -p ~/miniconda3` |
| **macOS (Apple Silicon)** | `curl -fsSL -o ~/miniconda.sh https://repo.anaconda.com/miniconda/Miniconda3-latest-MacOSX-arm64.sh && bash ~/miniconda.sh -b -p ~/miniconda3` |
| **macOS (Intel)** | `curl -fsSL -o ~/miniconda.sh https://repo.anaconda.com/miniconda/Miniconda3-latest-MacOSX-x86_64.sh && bash ~/miniconda.sh -b -p ~/miniconda3` |
| **Windows** | Download and run the [Miniconda installer](https://docs.conda.io/en/latest/miniconda.html), or `winget install Anaconda.Miniconda3` |

**2. Enter the wearable IMU directory.** Replace the placeholder with your
clone location:

```bash
cd /path/to/cse-145-237d-humanoid-teleop/wearable_imu
```

**3. Create the environment.** The checked-in file installs Python 3.11 and all
required Python packages:

```bash
conda env create -f env/environment.yml
```

If `humanoid-sim` already exists, update it instead:

```bash
conda env update -n humanoid-sim -f env/environment.yml --prune
```

The equivalent manual setup is:

```bash
conda create -y -n humanoid-sim -c conda-forge --override-channels python=3.11 pip
conda run -n humanoid-sim pip install -r requirements.txt
```

**4. Activate and verify the environment:**

```bash
conda activate humanoid-sim
python -c "import numpy, scipy, matplotlib, mujoco, zmq; print('dependencies OK')"
python -m pytest -q
```

**5. Run things.** With the environment active:

```bash
python demos/demo_launcher.py
```

…or run one-off commands without activating (handy if shell activation is
flaky on your machine):

```bash
conda run --no-capture-output -n humanoid-sim python demos/demo_launcher.py
```

> **Windows paths:** use backslashes, e.g.
> `conda run --no-capture-output -n humanoid-sim python demos\demo_launcher.py`.

### Option B: Raw Python venv (no conda)

Use this if you already have **Python 3.11** installed and prefer the standard
library's virtual environments.

**Linux / macOS:**

```bash
python3.11 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

**Windows (PowerShell):**

```powershell
py -3.11 -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install --upgrade pip
pip install -r requirements.txt
```

Then run any demo with the venv active:

```bash
python demos/demo_launcher.py
```

**Platform notes for the raw route:**

- **tkinter** powers the GUI demos (`demo_launcher.py`,
  `demo_mujoco_lower_body_viewer.py`) and is usually bundled with Python. If you
  hit `ModuleNotFoundError: No module named 'tkinter'`:
  - *Debian/Ubuntu:* `sudo apt install python3-tk`
  - *Fedora:* `sudo dnf install python3-tkinter`
  - *macOS (Homebrew Python):* `brew install python-tk`
  - *Windows:* re-run the python.org installer and enable "tcl/tk and IDLE".
- **MuJoCo** needs OpenGL for its viewer. On a headless Linux box the non-GUI
  demos still work; the MuJoCo viewer needs a display (or a software GL stack).

---

## Running the Demos

All demos live in [`demos/`](demos/) and are launched as standalone scripts.
The easiest entry point is the launcher, which lists every demo with a
description, an editable args field, and Run/Stop buttons:

```bash
python demos/demo_launcher.py
python demos/demo_launcher.py --list   # print demo keys without opening the GUI
```

### No hardware required (synthetic data)

```bash
# Live synthetic walking skeleton (Matplotlib 3D)
python demos/demo_live_lower_body_aggregation.py

# Static seven-segment aggregation, printed to the terminal
python demos/demo_lower_body_aggregation.py

# Recover hip/knee/ankle angles from synthetic mounted-IMU orientations
python demos/demo_imu_orientation_ik.py

# Toy planar-leg IK, optionally saving a plot
python demos/demo_inverse_kinematics.py --plot

# Render the body model + IMU mount frames to a PNG
python demos/demo_visualize_lower_body_model.py

# MuJoCo viewer driven by simulated IMUs (needs a display)
python demos/demo_mujoco_lower_body_viewer.py
```

For recorded IMU/camera handoff data, the Holosoma `ch_robot` MuJoCo replay can
use root translation from the recording so the robot moves across the floor
instead of stepping in place:

```bash
python demos/demo_mujoco_ch_robot_replay.py ../data/smplh_capture_3.jsonl --base-motion root_xy_forward
```

Use `--base-motion root_xy` for raw root trajectory replay, `fixed` for the
older in-place behavior, or `root_xyz` if the source has usable vertical root
motion.

The direct ESP32 IMU live bridge is still orientation-only: it can retarget
joint rotations live, but it needs a separate root-position source before it
can translate the MuJoCo freejoint across the floor. The ZMQ mock-live path is
the live-style path for recorded joint-position streams.

### With hardware (live ESP32-S3 / BNO085 over UDP)

The ESP32 nodes and the receiver must be on the same Wi-Fi network (e.g. the
shared phone hotspot). Point each node's `JETSON_IP` at the receiver and use the
matching `--host`/`--port` below.

```bash
# Live skeleton from real IMU packets. --config picks the sensor set:
#   thighs = 3 IMUs, shanks = 5, full = 7
python demos/demo_partial_imu_live_viewer.py --host 0.0.0.0 --port 5005 --config full

# Direct live IMU retargeting on the ch_robot MuJoCo model.
# The window opens first; the robot stays still until calibration completes.
python demos/demo_mujoco_ch_robot_live.py --host 0.0.0.0 --port 5005 --config shanks --fps 50

# Text-only packet monitor: rate, age, jitter, drops, raw quaternions
python demos/demo_udp_quaternion_receiver.py --host 0.0.0.0 --port 5005

# Round-trip latency ping to one node (pass the ESP32 IP from Serial Monitor)
python demos/demo_udp_latency_ping.py 192.168.1.164 --port 5006
```

On macOS, launching with `python` automatically relaunches the script with the
`mjpython` executable from the active environment. Hold a neutral standing pose
while the required IMUs connect and 20 calibration samples are collected. The
robot starts following the live pose after `Live retargeting active` appears.

See [`demos/README.md`](demos/README.md) for a full description of every demo
and its flags.

---

## Running the Tests

```bash
python -m pytest -q
```

With conda, without activating the environment:

```bash
conda run --no-capture-output -n humanoid-sim python -m pytest -q
```

The suite runs on synthetic data only — no hardware or display needed.

---

## Repository Layout

| Folder | Workflow | Purpose |
|---|---|---|
| `hardware/` | Real device IO | BNO085 connection, device IDs, ESP32 firmware, live quaternion acquisition |
| `sensor/` | Sensor packet layer | Quaternion packet types, frame conventions, timestamps, validity, filtering |
| `calibration/` | Real-world alignment | Neutral pose, strap twist correction, functional calibration |
| `ik/` | Pose estimation | Convert calibrated segment orientations into joint estimates |
| `model/` | Body model | Segment definitions, dimensions, IMU placement assumptions |
| `simulator/` | Archived test harness | MuJoCo synthetic body and fake IMU quaternion generator |
| `evaluation/` | Accuracy checks | Compare estimates to known ground truth or labeled captures |
| `demos/` | Runnable workflows | Small scripts that connect modules for inspection |
| `data/` | Captures and outputs | Calibration captures, synthetic logs, plots, visualizations |
| `tests/` | Verification | Unit tests and consistency checks |
| `env/` | Environment setup | Conda environment definition |

---

## Hardware & Wearable Placement

Each ESP32-S3 reads a BNO085 over full UART and streams fused quaternion reports
to the receiver over UDP on the shared Wi-Fi network. Every node sends
independently, so one dropped sensor does not block the rest.

Default wearable placement for the lower-body setup:

- **pelvis:** front center, around the belt line
- **thigh:** lateral/outside thigh, around mid-thigh
- **shank:** lateral/outside lower leg, around mid-shank
- **foot:** top of the foot, around midfoot/laces

For quaternion-only IK, exact sensor *position* matters less than *orientation*.
Per-segment sensor axis directions (which way each IMU's X/Y/Z point when
mounted) are documented in [`model/README.md`](model/README.md) and
[`ik/imu_orientation.py`](ik/imu_orientation.py). For wiring, firmware, and
BNO085 transport details, see [`hardware/README.md`](hardware/README.md).

---

## Sensor Data Contract

The BNO085 provides fused orientation quaternions directly, so the input
contract is quaternion packets (not raw accel/gyro). The current wire format is
a 40-byte little-endian UDP payload with `IMUQ` magic and `wxyz` quaternion
order, parsed by `sensor.packet.parse_quaternion_packet`.

```text
magic[4]          "IMUQ"
version           uint8, currently 1
sensor_id         uint8     which physical ESP32-S3 board sent this
segment_id        uint8     where that board is mounted this session
quat_order        uint8, 1 means wxyz
sequence          uint32
sensor_time_us    uint32 from ESP32 micros()
qw qx qy qz       float32
accuracy          float32 from the BNO085 rotation-vector report
status            uint8 from BNO085 event status
report_type       uint8, 1 means rotation vector
reserved          uint16
```

Segment IDs:

```text
0 pelvis   1 left_thigh   2 left_shank   3 left_foot
4 right_thigh   5 right_shank   6 right_foot   255 unknown
```

Identity rules:

- `sensor_id` identifies the physical board; calibration binds primarily to
  `segment_id` (the body placement), while diagnostics also track `sensor_id`
  so a bad physical board can be found and swapped without confusing profiles.

---

## Calibration

Real sensors are not fixed to bones — they rotate on the strap, mount slightly
differently every session, and get disturbed by soft-tissue motion. Calibration
corrects this before IK.

Recommended first calibration sequence:

```text
1. stand neutral for 2-5 seconds
2. average each sensor quaternion as the neutral reference
3. align thigh/shank headings within each leg
4. do slow knee flexion or squat samples
5. estimate strap twist corrections from the knee hinge constraint
6. save a calibration profile
```

A calibration profile is expected to hold:

```text
CalibrationProfile:
    user_height_optional
    segment_lengths
    neutral_sensor_quaternions
    sensor_to_segment_rotations
    leg_heading_corrections
    knee_axis_estimates
    ankle_axis_estimates_optional
```

Note that quaternions alone do not determine bone lengths; segment lengths come
from manual measurement or user-height ratios.

---

## Development Direction

The guiding rule: the estimator should be designed for messy straps and real
people first, then tested with simulation as a convenience.

Next real work, in order:

```text
1. flash one ESP32-S3/BNO085 node and receive UDP packets on the laptop over the phone hotspot
2. flash all 7 nodes with unique sensor_id and segment_id settings
3. tune quaternion spike filtering with real packet traces
4. validate neutral standing calibration
5. add thigh/shank heading alignment
6. add functional knee-hinge calibration
7. route calibrated quaternions into IK
8. keep MuJoCo only as a fake-data generator and regression harness
```

**Implemented so far:** ESP32-S3/BNO085 UDP firmware scaffold, receiver-side
binary quaternion parser, latest-packet UDP buffer, quaternion
norm/stale/spike filtering, SLERP smoothing, neutral standing calibration, and
partial/full live skeleton viewers.

**Not implemented yet:** thigh/shank heading alignment, functional knee-hinge
calibration, magnetometer disturbance gating, and a constrained IK solver with
residual/confidence output (the current IK is forward kinematics from per-segment
orientations).
