"""Run a minimal lower-body inverse kinematics example."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import matplotlib.pyplot as plt

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from ik.planar_leg import FootTarget, LegSegmentLengths, forward_kinematics, solve_leg_ik


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--x", type=float, default=0.18, help="Foot-tip x target in meters")
    parser.add_argument("--z", type=float, default=-0.82, help="Foot-tip z target in meters")
    parser.add_argument(
        "--angle",
        type=float,
        default=0.12,
        help="Foot segment angle in radians from vertical down",
    )
    parser.add_argument(
        "--knee-direction",
        type=int,
        choices=(-1, 1),
        default=1,
        help="IK branch for the knee bend",
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Save a plot of the solved leg pose",
    )
    parser.add_argument(
        "--plot-path",
        type=Path,
        default=Path("data/ik_demo/planar_leg_ik.png"),
        help="Output path used with --plot",
    )
    return parser.parse_args()


def plot_solution(
    target: FootTarget,
    solution_path: Path,
    lengths: LegSegmentLengths,
    knee_direction: int,
) -> None:
    solution = solve_leg_ik(target, lengths, knee_direction=knee_direction)
    pose = forward_kinematics(solution.angles, lengths)

    xs = [pose[name][0] for name in ("hip", "knee", "ankle", "foot")]
    zs = [pose[name][1] for name in ("hip", "knee", "ankle", "foot")]

    solution_path.parent.mkdir(parents=True, exist_ok=True)

    fig, axis = plt.subplots(figsize=(5, 5))
    axis.plot(xs, zs, marker="o", linewidth=3, label="solved leg")
    axis.scatter([target.x], [target.z], marker="x", s=100, label="target foot")
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("x forward (m)")
    axis.set_ylabel("z up (m)")
    axis.set_title("Planar lower-body inverse kinematics")
    axis.grid(True, alpha=0.3)
    axis.legend()
    fig.tight_layout()
    fig.savefig(solution_path, dpi=160)
    plt.close(fig)


def main() -> None:
    args = parse_args()
    lengths = LegSegmentLengths()
    target = FootTarget(x=args.x, z=args.z, angle=args.angle)
    solution = solve_leg_ik(
        target,
        lengths,
        knee_direction=args.knee_direction,
    )

    print("Inverse kinematics target")
    print(f"  foot position: x={target.x:.3f} m, z={target.z:.3f} m")
    print(f"  foot angle: {target.angle:.3f} rad")
    print()
    print("Solved joint angles")
    for joint, value in solution.angles.as_degrees().items():
        print(f"  {joint}: {value:8.3f} deg")
    print()
    print(f"reachable: {solution.reachable}")
    print(f"position error: {solution.position_error:.6f} m")
    print(f"orientation error: {solution.orientation_error:.6f} rad")

    if args.plot:
        plot_solution(target, args.plot_path, lengths, args.knee_direction)
        print(f"plot saved: {args.plot_path}")


if __name__ == "__main__":
    main()
