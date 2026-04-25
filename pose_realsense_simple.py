from pathlib import Path
import time

import cv2
import mediapipe as mp
import numpy as np
import pyrealsense2 as rs


# MediaPipe Tasks API aliases.
BaseOptions = mp.tasks.BaseOptions
PoseLandmarker = mp.tasks.vision.PoseLandmarker
PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
RunningMode = mp.tasks.vision.RunningMode

MODEL_PATH = Path(__file__).resolve().parent / "models" / "pose_landmarker.task"
WINDOW_NAME = "RealSense Pose"
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_FPS = 30

COLOR_FORMAT_CANDIDATES = [
    ("BGR8", rs.format.bgr8),
    ("RGB8", rs.format.rgb8),
    ("YUYV", rs.format.yuyv),
    ("MJPEG", rs.format.mjpeg),
]

# MediaPipe Pose connections, kept local so we do not depend on the old solutions API.
POSE_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 7),
    (0, 4), (4, 5), (5, 6), (6, 8),
    (9, 10),
    (11, 12),
    (11, 13), (13, 15), (15, 17), (15, 19), (15, 21), (17, 19),
    (12, 14), (14, 16), (16, 18), (16, 20), (16, 22), (18, 20),
    (11, 23), (12, 24), (23, 24),
    (23, 25), (24, 26), (25, 27), (26, 28),
    (27, 29), (28, 30), (29, 31), (30, 32),
    (27, 31), (28, 32),
]

LABEL_LANDMARKS = {
    11: "L shoulder",
    12: "R shoulder",
    15: "L wrist",
    16: "R wrist",
}


def landmark_pixel(landmark, width, height):
    """Convert a normalized landmark into an image pixel."""
    px = int(np.clip(landmark.x * width, 0, width - 1))
    py = int(np.clip(landmark.y * height, 0, height - 1))
    return px, py


def landmark_visible(landmark, threshold=0.3):
    """Skip landmarks that are present but clearly unreliable."""
    return getattr(landmark, "visibility", 1.0) >= threshold


def sample_depth_m(depth_frame, px, py, width, height):
    """Use a small median filter so depth is less noisy at joint pixels."""
    samples = []
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            x = int(np.clip(px + dx, 0, width - 1))
            y = int(np.clip(py + dy, 0, height - 1))
            depth_m = depth_frame.get_distance(x, y)
            if depth_m > 0:
                samples.append(depth_m)
    return float(np.median(samples)) if samples else 0.0


def landmark_xyz(depth_frame, intrinsics, landmark, width, height):
    """Read aligned depth at the landmark and deproject to camera XYZ."""
    px, py = landmark_pixel(landmark, width, height)
    depth_m = sample_depth_m(depth_frame, px, py, width, height)
    if depth_m <= 0:
        return px, py, None
    xyz = rs.rs2_deproject_pixel_to_point(intrinsics, [float(px), float(py)], depth_m)
    return px, py, xyz


def draw_pose(image_bgr, landmarks, depth_frame, intrinsics):
    """Draw pose lines, joint circles, and a few example XYZ labels."""
    height, width = image_bgr.shape[:2]

    for start_idx, end_idx in POSE_CONNECTIONS:
        start = landmarks[start_idx]
        end = landmarks[end_idx]
        if not landmark_visible(start) or not landmark_visible(end):
            continue
        x1, y1 = landmark_pixel(start, width, height)
        x2, y2 = landmark_pixel(end, width, height)
        cv2.line(image_bgr, (x1, y1), (x2, y2), (0, 255, 255), 2, cv2.LINE_AA)

    for landmark in landmarks:
        if not landmark_visible(landmark):
            continue
        px, py = landmark_pixel(landmark, width, height)
        cv2.circle(image_bgr, (px, py), 4, (0, 180, 0), -1, cv2.LINE_AA)

    for landmark_idx, label in LABEL_LANDMARKS.items():
        landmark = landmarks[landmark_idx]
        if not landmark_visible(landmark):
            continue
        px, py, xyz = landmark_xyz(depth_frame, intrinsics, landmark, width, height)
        if xyz is None:
            text = f"{label}: no depth"
        else:
            text = f"{label}: {xyz[0]:+.2f}, {xyz[1]:+.2f}, {xyz[2]:+.2f} m"
        text_origin = (px + 8, max(py - 8, 20))
        cv2.putText(image_bgr, text, text_origin, cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(image_bgr, text, text_origin, cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)


def create_landmarker():
    """Create the MediaPipe Pose Landmarker in VIDEO mode."""
    options = PoseLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=str(MODEL_PATH)),
        running_mode=RunningMode.VIDEO,
        num_poses=1,
        min_pose_detection_confidence=0.5,
        min_pose_presence_confidence=0.5,
        min_tracking_confidence=0.5,
        output_segmentation_masks=False,
    )
    return PoseLandmarker.create_from_options(options)


def start_pipeline_with_fallback(pipeline):
    """Try a few common RealSense color formats until one works."""
    errors = []

    for color_name, color_format in COLOR_FORMAT_CANDIDATES:
        config = rs.config()
        config.enable_stream(rs.stream.depth, FRAME_WIDTH, FRAME_HEIGHT, rs.format.z16, FRAME_FPS)
        config.enable_stream(rs.stream.color, FRAME_WIDTH, FRAME_HEIGHT, color_format, FRAME_FPS)

        try:
            pipeline.start(config)
            print(f"Started RealSense with color format: {color_name}")
            return color_format
        except RuntimeError as exc:
            errors.append(f"{color_name}: {exc}")

    joined_errors = "\n".join(errors)
    raise RuntimeError(
        "Could not start the RealSense pipeline with any tested color format.\n"
        "Try closing other camera apps, reconnecting the D435 to USB 3, or lowering the resolution.\n"
        f"Startup errors:\n{joined_errors}"
    )


def color_frame_to_bgr(color_frame, color_format):
    """Convert the active RealSense color format into OpenCV BGR."""
    frame_data = np.asanyarray(color_frame.get_data())

    if color_format == rs.format.bgr8:
        return frame_data
    if color_format == rs.format.rgb8:
        return cv2.cvtColor(frame_data, cv2.COLOR_RGB2BGR)
    if color_format == rs.format.yuyv:
        return cv2.cvtColor(frame_data, cv2.COLOR_YUV2BGR_YUY2)
    if color_format == rs.format.mjpeg:
        decoded = cv2.imdecode(np.frombuffer(color_frame.get_data(), dtype=np.uint8), cv2.IMREAD_COLOR)
        if decoded is None:
            raise RuntimeError("Failed to decode MJPEG color frame.")
        return decoded

    raise RuntimeError(f"Unsupported color format returned by RealSense: {color_format}")


def is_disconnect_error(exc):
    """Detect the common RealSense disconnect errors we want to handle gracefully."""
    message = str(exc).lower()
    return "device disconnected" in message or "no device connected" in message


def safe_stop_pipeline(pipeline):
    """Stop the RealSense pipeline without masking the real failure on shutdown."""
    try:
        pipeline.stop()
    except RuntimeError as exc:
        if "cannot be called before start" not in str(exc).lower():
            print(f"Warning: RealSense pipeline stop failed: {exc}")


def main():
    if not MODEL_PATH.is_file():
        raise FileNotFoundError(f"Missing model file: {MODEL_PATH}")

    landmarker = create_landmarker()

    # Start color + depth on the D435.
    pipeline = rs.pipeline()
    pipeline_started = False
    color_format = None

    # Align depth into color camera space so pixels match.
    align = rs.align(rs.stream.color)
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    try:
        color_format = start_pipeline_with_fallback(pipeline)
        pipeline_started = True

        while True:
            # Get the next synchronized RealSense frames.
            try:
                frames = pipeline.wait_for_frames()
            except RuntimeError as exc:
                if is_disconnect_error(exc):
                    print("\nRealSense device disconnected. Reconnect the D435 and run the script again.")
                    pipeline_started = False
                    break
                raise

            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert the color frame to RGB for MediaPipe Tasks.
            color_bgr = color_frame_to_bgr(color_frame, color_format)
            color_rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=color_rgb)

            # VIDEO mode needs a steadily increasing timestamp in milliseconds.
            timestamp_ms = time.monotonic_ns() // 1_000_000
            result = landmarker.detect_for_video(mp_image, timestamp_ms)

            if result.pose_landmarks:
                intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
                draw_pose(color_bgr, result.pose_landmarks[0], depth_frame, intrinsics)
            else:
                cv2.putText(color_bgr, "No pose detected", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)

            cv2.imshow(WINDOW_NAME, color_bgr)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        landmarker.close()
        if pipeline_started:
            safe_stop_pipeline(pipeline)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
