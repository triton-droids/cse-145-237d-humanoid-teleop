# Human Pose Estimation with Intel RealSense D435 and MediaPipe

This folder contains a simple local pose estimation demo for Ubuntu using:

- Python
- Intel RealSense D435
- `pyrealsense2` for camera input
- MediaPipe Pose Landmarker Tasks API
- OpenCV for drawing and display

The main script is:

- `pose_realsense_simple.py`

## What The Demo Does

The script:

- opens the RealSense depth stream
- opens the RealSense color stream
- aligns depth to color using `rs.align(rs.stream.color)`
- runs MediaPipe Pose Landmarker on the RGB color frame
- draws skeleton lines and landmark circles with OpenCV
- reads aligned depth for a few landmarks
- converts landmark pixel + depth into camera-frame `X, Y, Z`
- overlays example `XYZ` labels for shoulders and wrists
- shows the live result in a local OpenCV window
- quits when you press `q`

## Files

- `pose_realsense_simple.py`: runnable demo
- `models/pose_landmarker.task`: local MediaPipe model file

## Environment

Tested for a local Ubuntu workflow with a Python virtual environment.

Python packages used:

- `mediapipe`
- `pyrealsense2`
- `opencv-python`
- `numpy`

## Setup

From the project directory:

```bash
cd /home/droids/Desktop/humanEstimation
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install mediapipe pyrealsense2 opencv-python numpy
```

## Download The Model

This demo expects the MediaPipe model at:

```text
/home/droids/Desktop/humanEstimation/models/pose_landmarker.task
```

Create the folder and download the official `full` model:

```bash
cd /home/droids/Desktop/humanEstimation
mkdir -p models
wget -O models/pose_landmarker.task \
  https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_full/float16/latest/pose_landmarker_full.task
```

## Run

```bash
cd /home/droids/Desktop/humanEstimation
source .venv/bin/activate
python pose_realsense_simple.py
```

Press `q` in the OpenCV window to quit.

## RealSense Camera Activation

There is no separate manual "activation" step in the code. The RealSense camera is activated when the script starts the pipeline.

The script does this during startup:

1. Creates a RealSense pipeline.
2. Tries to open the depth stream as `640x480`, `z16`, `30 FPS`.
3. Tries to open the color stream as `640x480`, `30 FPS`.
4. Tries multiple color formats until one works: `BGR8`, `RGB8`, `YUYV`, `MJPEG`.
5. Prints the selected format, for example:

```text
Started RealSense with color format: YUYV
```

6. Aligns depth to the color stream with:

```python
rs.align(rs.stream.color)
```

7. Converts the returned color frame into OpenCV BGR for drawing and into RGB for MediaPipe.

This fallback was added because on this machine the D435 did not accept a `BGR8` request directly and reported that `YUYV` was available.

## Depth Information

Yes, the script uses real depth from the D435.

It does the following:

- samples aligned depth near each labeled landmark
- uses `rs.rs2_deproject_pixel_to_point(...)`
- converts pixel + depth into camera-frame `X, Y, Z` in meters
- overlays example `XYZ` text for left shoulder, right shoulder, left wrist, and right wrist

Important note:

- MediaPipe also provides model-estimated 3D landmarks
- the `XYZ` labels in this demo come from the RealSense aligned depth stream, not only from MediaPipe inference

## Commands We Used To Debug RealSense Startup

If the camera is busy or the pipeline fails to start, these commands are useful.

List video devices:

```bash
ls -l /dev/video*
```

Check which process is using the camera:

```bash
fuser -v /dev/video*
```

Close Intel RealSense Viewer if it is holding the device:

```bash
pkill realsense-viewer
```

You can also inspect running processes:

```bash
ps -ef | rg 'realsense|python|opencv|cv2|camera|pose_realsense_simple'
```

## Common Problems

### 1. Camera Busy

Example error:

```text
RuntimeError: xioctl(VIDIOC_S_FMT) failed, errno=16 Last Error: Device or resource busy
```

Cause:

- another app already owns the RealSense camera
- `realsense-viewer` is a common cause

Fix:

```bash
pkill realsense-viewer
```

Then run the script again.

### 2. Unsupported Color Format

Example error we hit before adding fallback:

```text
Failed to resolve the request:
    Format: BGR8, width: 640, height: 480

Into:
    Formats:
      YUYV
```

Cause:

- the camera did not expose `BGR8` for the requested stream profile

Fix:

- the script now automatically tries `BGR8`, `RGB8`, `YUYV`, and `MJPEG`

### 3. Missing Model File

Example error:

```text
FileNotFoundError: Missing model file: /home/droids/Desktop/humanEstimation/models/pose_landmarker.task
```

Fix:

```bash
cd /home/droids/Desktop/humanEstimation
mkdir -p models
wget -O models/pose_landmarker.task \
  https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_full/float16/latest/pose_landmarker_full.task
```

### 4. Harmless Startup Warnings

You may see warnings like these:

- TensorFlow Lite feedback manager warnings
- `QFontDatabase: Cannot find font directory ...`

In our testing, these were not the reason the script failed to start. The important failures were the RealSense device conflict and unsupported color format request.

### 5. Device Disconnect During Streaming

Example error:

```text
RuntimeError: Device disconnected. Failed to reconnect: No device connected
```

Cause:

- the D435 disconnected after the pipeline started
- common reasons are a loose cable, flaky USB connection, or insufficient USB bandwidth/power

Fix:

- reconnect the camera
- plug the D435 into a stable USB 3 port
- avoid hubs if possible
- rerun the script after reconnecting

The script now handles this case more cleanly and prints a short reconnect message instead of raising a second cleanup exception.

## Full Command List

If you want the entire flow in one place, these are the commands:

Create environment and install packages:

```bash
cd /home/droids/Desktop/humanEstimation
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install mediapipe pyrealsense2 opencv-python numpy
```

Download the model:

```bash
cd /home/droids/Desktop/humanEstimation
mkdir -p models
wget -O models/pose_landmarker.task \
  https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_full/float16/latest/pose_landmarker_full.task
```

Run the demo:

```bash
cd /home/droids/Desktop/humanEstimation
source .venv/bin/activate
python pose_realsense_simple.py
```

If the camera is busy:

```bash
fuser -v /dev/video*
pkill realsense-viewer
python pose_realsense_simple.py
```

## Notes

- Use a USB 3 port for the D435 when possible.
- If startup still fails, reconnect the camera and try again.
- If needed, the next debugging step is lowering resolution or printing active RealSense stream profiles.
