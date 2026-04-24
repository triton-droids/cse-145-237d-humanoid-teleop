"""Visualize the lower-body model and mounted IMU frames."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from model.lower_body import LowerBodyDimensions, build_lower_body_model  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("data/visualizations/lower_body_model.png"),
        help="Path for the saved visualization image",
    )
    return parser.parse_args()


def set_equal_axes(axis: plt.Axes, points: np.ndarray) -> None:
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    centers = 0.5 * (mins + maxs)
    radius = 0.5 * float(np.max(maxs - mins))
    radius = max(radius, 0.3)

    axis.set_xlim(centers[0] - radius, centers[0] + radius)
    axis.set_ylim(centers[1] - radius, centers[1] + radius)
    axis.set_zlim(centers[2] - radius, centers[2] + radius)


def main() -> None:
    args = parse_args()
    model = build_lower_body_model()
    dims = LowerBodyDimensions()

    args.output.parent.mkdir(parents=True, exist_ok=True)

    figure = plt.figure(figsize=(9, 7))
    axis = figure.add_subplot(111, projection="3d")

    left_hip = model.hip_centers["left"]
    right_hip = model.hip_centers["right"]
    axis.plot(
        [left_hip[0], right_hip[0]],
        [left_hip[1], right_hip[1]],
        [left_hip[2], right_hip[2]],
        color="#444444",
        linewidth=6,
        label="pelvis",
    )

    segment_colors = {
        "left": "#2a9d8f",
        "right": "#e76f51",
    }

    points = [left_hip, right_hip]

    for name, segment in model.segments.items():
        side = "left" if name.startswith("left") else "right"
        origin = segment.origin
        distal = segment.distal_point
        points.extend([origin, distal])

        axis.plot(
            [origin[0], distal[0]],
            [origin[1], distal[1]],
            [origin[2], distal[2]],
            color=segment_colors[side],
            linewidth=5 if name.endswith("foot") else 7,
        )
        axis.text(
            distal[0],
            distal[1],
            distal[2],
            name.replace("_", " "),
            fontsize=9,
        )

    axis.scatter(
        [model.pelvis_center[0]],
        [model.pelvis_center[1]],
        [model.pelvis_center[2]],
        color="#111111",
        s=36,
    )

    axis_length = 0.08
    axis_colors = {
        "x": "#d62828",
        "y": "#1d3557",
        "z": "#2a9d8f",
    }

    for name, mount in model.imu_mounts.items():
        position = mount.position
        rotation = mount.sensor_rotation_world
        points.append(position)

        axis.scatter(
            [position[0]],
            [position[1]],
            [position[2]],
            color="#ffcc00",
            edgecolors="#222222",
            s=70,
        )
        axis.text(position[0], position[1], position[2] + 0.02, f"{name} imu", fontsize=8)

        sensor_axes = {
            "x": rotation.apply([axis_length, 0.0, 0.0]),
            "y": rotation.apply([0.0, axis_length, 0.0]),
            "z": rotation.apply([0.0, 0.0, axis_length]),
        }
        for axis_name, vector in sensor_axes.items():
            axis.quiver(
                position[0],
                position[1],
                position[2],
                vector[0],
                vector[1],
                vector[2],
                color=axis_colors[axis_name],
                linewidth=2,
                arrow_length_ratio=0.18,
            )

    world_origin = np.array([-0.25, -0.25, 0.72], dtype=float)
    axis.quiver(*world_origin, 0.10, 0.0, 0.0, color=axis_colors["x"], linewidth=2)
    axis.quiver(*world_origin, 0.0, 0.10, 0.0, color=axis_colors["y"], linewidth=2)
    axis.quiver(*world_origin, 0.0, 0.0, 0.10, color=axis_colors["z"], linewidth=2)
    axis.text(*(world_origin + np.array([0.11, 0.0, 0.0])), "world +X", fontsize=8)
    axis.text(*(world_origin + np.array([0.0, 0.11, 0.0])), "world +Y", fontsize=8)
    axis.text(*(world_origin + np.array([0.0, 0.0, 0.11])), "world +Z", fontsize=8)

    point_array = np.vstack(points)
    set_equal_axes(axis, point_array)

    axis.set_xlabel("x forward (m)")
    axis.set_ylabel("y left (m)")
    axis.set_zlabel("z up (m)")
    axis.set_title("Lower-body model with IMU mounts")
    axis.view_init(elev=20, azim=-62)
    figure.tight_layout()
    figure.savefig(args.output, dpi=180)
    plt.close(figure)

    print(f"visualization saved: {args.output}")
    print(f"hip spacing: {dims.hip_spacing:.2f} m")
    print(f"thigh length: {dims.thigh_length:.2f} m")
    print(f"shank length: {dims.shank_length:.2f} m")
    print(f"foot total length: {dims.foot_total_length:.2f} m")
    print(f"foot heel length: {dims.foot_heel_length:.2f} m")
    print(f"foot toe length: {dims.foot_toe_length:.2f} m")


if __name__ == "__main__":
    main()
