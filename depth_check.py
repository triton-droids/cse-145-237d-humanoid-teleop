import time

import cv2
import numpy as np
import pyrealsense2 as rs

WINDOW_NAME = "RealSense Depth Check"
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_FPS = 30

DEPTH_PROFILE_CANDIDATES = [
    (640, 480, 30),
    (848, 480, 30),
    (640, 360, 30),
    (424, 240, 30),
]

COLOR_FORMAT_CANDIDATES = [
    ("BGR8", rs.format.bgr8),
    ("RGB8", rs.format.rgb8),
    ("YUYV", rs.format.yuyv),
    ("MJPEG", rs.format.mjpeg),
]


def start_pipeline_with_fallback(pipeline):
    errors = []

    for width, height, fps in DEPTH_PROFILE_CANDIDATES:
        for color_name, color_format in COLOR_FORMAT_CANDIDATES:
            config = rs.config()
            config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
            config.enable_stream(rs.stream.color, width, height, color_format, fps)

            try:
                pipeline.start(config)
                print(
                    f"Started RealSense depth {width}x{height}@{fps} + color {color_name}"
                )
                return color_format
            except RuntimeError as exc:
                errors.append(
                    f"{width}x{height}@{fps}, {color_name}: {exc}"
                )

    joined_errors = "\n".join(errors)
    raise RuntimeError(
        "Could not start the RealSense pipeline with any tested depth/color profile.\n"
        "Try closing other camera apps, reconnecting the D435 to USB 3, or lowering the resolution.\n"
        f"Startup errors:\n{joined_errors}"
    )


def main():
    pipeline = rs.pipeline()
    pipeline_started = False
    color_format = None

    align = rs.align(rs.stream.color)
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    try:
        color_format = start_pipeline_with_fallback(pipeline)
        pipeline_started = True
        last_print = 0.0

        while True:
            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)
            depth_frame = aligned.get_depth_frame()
            color_frame = aligned.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(depth_frame.get_data())

            if color_format == rs.format.bgr8:
                color_image = np.asanyarray(color_frame.get_data())
            elif color_format == rs.format.rgb8:
                color_image = cv2.cvtColor(
                    np.asanyarray(color_frame.get_data()), cv2.COLOR_RGB2BGR
                )
            elif color_format == rs.format.yuyv:
                color_image = cv2.cvtColor(
                    np.asanyarray(color_frame.get_data()), cv2.COLOR_YUV2BGR_YUY2
                )
            elif color_format == rs.format.mjpeg:
                decoded = cv2.imdecode(
                    np.frombuffer(color_frame.get_data(), dtype=np.uint8),
                    cv2.IMREAD_COLOR,
                )
                if decoded is None:
                    raise RuntimeError("Failed to decode MJPEG color frame.")
                color_image = decoded
            else:
                raise RuntimeError(
                    f"Unsupported color format returned by RealSense: {color_format}"
                )

            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
            )

            height, width = depth_image.shape[:2]
            cx, cy = width // 2, height // 2
            distance_m = depth_frame.get_distance(cx, cy)

            now = time.time()
            if now - last_print > 0.5:
                print(f"Center depth: {distance_m:.3f} m")
                last_print = now

            cv2.circle(color_image, (cx, cy), 5, (0, 255, 0), -1)
            cv2.putText(
                color_image,
                f"{distance_m:.3f} m",
                (cx + 10, cy - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

            stacked = np.hstack((color_image, depth_colormap))
            cv2.imshow(WINDOW_NAME, stacked)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
    finally:
        if pipeline_started:
            pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
