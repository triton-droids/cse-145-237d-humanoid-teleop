"""Live-view lower-body aggregation with a synthetic motion source."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from ik.lower_body_aggregation import LowerBodySkeleton, aggregate_lower_body_skeleton  # noqa: E402
from sensor.packet import SegmentId  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interval-ms", type=int, default=40)
    parser.add_argument("--frames", type=int, default=0, help="Stop after this many frames; 0 runs until the window closes.")
    parser.add_argument("--no-show", action="store_true", help="Run without opening a window; useful for smoke tests.")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("data/visualizations/live_lower_body_aggregation_last_frame.png"),
        help="Path for the final frame image when --no-show is used.",
    )
    return parser.parse_args()


def synthetic_segment_orientations(frame_index: int) -> dict[SegmentId, Rotation]:
    phase = frame_index * 0.08
    pelvis = Rotation.from_euler(
        "xyz",
        [1.5 * np.sin(phase * 0.5), -2.0, 4.0 * np.sin(phase * 0.35)],
        degrees=True,
    )

    left_swing = np.sin(phase)
    right_swing = np.sin(phase + np.pi)

    left_hip = Rotation.from_euler("xyz", [5.0, 14.0 + 12.0 * left_swing, -2.0], degrees=True)
    left_knee = Rotation.from_euler("xyz", [0.0, 24.0 + 24.0 * max(0.0, left_swing), 0.0], degrees=True)
    left_ankle = Rotation.from_euler("xyz", [1.0, -8.0 - 5.0 * left_swing, 3.0], degrees=True)

    right_hip = Rotation.from_euler("xyz", [3.0, 14.0 + 12.0 * right_swing, 2.0], degrees=True)
    right_knee = Rotation.from_euler("xyz", [0.0, 24.0 + 24.0 * max(0.0, right_swing), 0.0], degrees=True)
    right_ankle = Rotation.from_euler("xyz", [-1.0, -8.0 - 5.0 * right_swing, -2.0], degrees=True)

    return {
        SegmentId.PELVIS: pelvis,
        SegmentId.LEFT_THIGH: pelvis * left_hip,
        SegmentId.LEFT_SHANK: pelvis * left_hip * left_knee,
        SegmentId.LEFT_FOOT: pelvis * left_hip * left_knee * left_ankle,
        SegmentId.RIGHT_THIGH: pelvis * right_hip,
        SegmentId.RIGHT_SHANK: pelvis * right_hip * right_knee,
        SegmentId.RIGHT_FOOT: pelvis * right_hip * right_knee * right_ankle,
    }


class SkeletonLivePlot:
    def __init__(self) -> None:
        self.figure = plt.figure(figsize=(8, 7))
        self.axis = self.figure.add_subplot(111, projection="3d")
        self.lines = {
            "pelvis": self.axis.plot([], [], [], color="#222222", linewidth=6)[0],
            "left_leg": self.axis.plot([], [], [], color="#2a9d8f", linewidth=6, marker="o", markersize=5)[0],
            "right_leg": self.axis.plot([], [], [], color="#e76f51", linewidth=6, marker="o", markersize=5)[0],
            "left_foot": self.axis.plot([], [], [], color="#2a9d8f", linewidth=4)[0],
            "right_foot": self.axis.plot([], [], [], color="#e76f51", linewidth=4)[0],
        }
        self.text = self.axis.text2D(0.03, 0.95, "", transform=self.axis.transAxes)
        self.axis.set_xlabel("x forward (m)")
        self.axis.set_ylabel("y left (m)")
        self.axis.set_zlabel("z up (m)")
        self.axis.set_title("Live lower-body aggregation")
        self.axis.view_init(elev=18, azim=-58)
        self.axis.set_xlim(-0.8, 0.35)
        self.axis.set_ylim(-0.55, 0.55)
        self.axis.set_zlim(0.0, 1.2)

    def update(self, frame_index: int, skeleton: LowerBodySkeleton) -> None:
        self._set_line("pelvis", np.vstack([skeleton.joints["left_hip"], skeleton.joints["right_hip"]]))
        for side in ("left", "right"):
            self._set_line(
                f"{side}_leg",
                np.vstack(
                    [
                        skeleton.joints[f"{side}_hip"],
                        skeleton.joints[f"{side}_knee"],
                        skeleton.joints[f"{side}_ankle"],
                        skeleton.joints[f"{side}_toe"],
                    ]
                ),
            )
            self._set_line(
                f"{side}_foot",
                np.vstack(
                    [
                        skeleton.joints[f"{side}_heel"],
                        skeleton.joints[f"{side}_ankle"],
                        skeleton.joints[f"{side}_toe"],
                    ]
                ),
            )

        left_knee_y = skeleton.joint_rotations["left"].knee.as_euler("xyz", degrees=True)[1]
        right_knee_y = skeleton.joint_rotations["right"].knee.as_euler("xyz", degrees=True)[1]
        self.text.set_text(
            f"frame {frame_index}\n"
            f"left knee y: {left_knee_y:5.1f} deg\n"
            f"right knee y: {right_knee_y:5.1f} deg"
        )

    def _set_line(self, name: str, points: np.ndarray) -> None:
        line = self.lines[name]
        line.set_data(points[:, 0], points[:, 1])
        line.set_3d_properties(points[:, 2])


def main() -> None:
    args = parse_args()
    plot = SkeletonLivePlot()

    frame_count = args.frames if args.frames > 0 else 10_000_000
    for frame_index in range(frame_count):
        skeleton = aggregate_lower_body_skeleton(synthetic_segment_orientations(frame_index))
        plot.update(frame_index, skeleton)
        plot.figure.canvas.draw_idle()

        if args.no_show:
            continue

        plt.pause(args.interval_ms / 1000.0)
        if not plt.fignum_exists(plot.figure.number):
            break

    if args.no_show:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        plot.figure.savefig(args.output, dpi=180)
        print(f"final frame saved: {args.output}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
