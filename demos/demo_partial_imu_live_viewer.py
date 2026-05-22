"""Live lower-body skeleton from a partial IMU set (default: pelvis + 2 thighs).

Usage (from project root):
  conda run --no-capture-output -p .\.conda python demos\demo_partial_imu_live_viewer.py --host 0.0.0.0 --port 5005
  conda run --no-capture-output -p .\.conda python demos\demo_partial_imu_live_viewer.py --host 0.0.0.0 --port 5005 --config shanks
  conda run --no-capture-output -p .\.conda python demos\demo_partial_imu_live_viewer.py --host 0.0.0.0 --port 5005 --config full
"""

from __future__ import annotations

import argparse
import threading
import time
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from calibration.neutral import CalibrationProfile, NeutralCalibrationAccumulator  # noqa: E402
from ik.imu_orientation import default_lower_limb_mounts, front_pelvis_mount  # noqa: E402
from ik.lower_body_aggregation import aggregate_lower_body_skeleton  # noqa: E402
from sensor.filtering import QuaternionPacketFilter  # noqa: E402
from sensor.packet import SegmentId  # noqa: E402
from sensor.udp_receiver import LatestPacketBuffer, receive_quaternion_packets  # noqa: E402
from simulator.mujoco_lower_body import lower_body_points_from_skeleton  # noqa: E402


PARTIAL_CONFIGS: dict[str, tuple[SegmentId, ...]] = {
    "thighs": (
        SegmentId.PELVIS,
        SegmentId.LEFT_THIGH,
        SegmentId.RIGHT_THIGH,
    ),
    "shanks": (
        SegmentId.PELVIS,
        SegmentId.LEFT_THIGH,
        SegmentId.RIGHT_THIGH,
        SegmentId.LEFT_SHANK,
        SegmentId.RIGHT_SHANK,
    ),
    "full": (
        SegmentId.PELVIS,
        SegmentId.LEFT_THIGH,
        SegmentId.LEFT_SHANK,
        SegmentId.LEFT_FOOT,
        SegmentId.RIGHT_THIGH,
        SegmentId.RIGHT_SHANK,
        SegmentId.RIGHT_FOOT,
    ),
}

_LEFT = default_lower_limb_mounts("left")
_RIGHT = default_lower_limb_mounts("right")

# Fixed sensor-to-segment mount rotations per segment
_SEGMENT_MOUNTS: dict[SegmentId, Rotation] = {
    SegmentId.PELVIS:       front_pelvis_mount(),
    SegmentId.LEFT_THIGH:   _LEFT["thigh"],
    SegmentId.LEFT_SHANK:   _LEFT["shank"],
    SegmentId.LEFT_FOOT:    _LEFT["foot"],
    SegmentId.RIGHT_THIGH:  _RIGHT["thigh"],
    SegmentId.RIGHT_SHANK:  _RIGHT["shank"],
    SegmentId.RIGHT_FOOT:   _RIGHT["foot"],
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument(
        "--config",
        choices=list(PARTIAL_CONFIGS.keys()),
        default="thighs",
        help="IMU set to use: thighs=3 IMUs, shanks=5, full=7 (default: thighs)",
    )
    parser.add_argument(
        "--max-age-ms", type=float, default=150.0,
        help="max packet age before a segment is treated as stale (ms)",
    )
    parser.add_argument(
        "--min-samples", type=int, default=20,
        help="still-standing samples per segment needed to finish calibration",
    )
    return parser.parse_args()


def _calibrate(
    buffer: LatestPacketBuffer,
    required_segments: tuple[SegmentId, ...],
    min_samples: int,
) -> CalibrationProfile:
    """Block until enough neutral-pose samples have been collected and averaged."""
    acc = NeutralCalibrationAccumulator(
        required_segments=required_segments,
        min_samples_per_segment=min_samples,
    )
    last_seqs: dict[SegmentId, int] = {}
    last_print = 0.0

    print(f"\nCalibration — stand still in neutral pose ({min_samples} samples per sensor needed)")
    while not acc.ready():
        for seg_id in required_segments:
            pkt = buffer.packets.get(seg_id)
            if pkt is None or last_seqs.get(seg_id) == pkt.sequence:
                continue
            last_seqs[seg_id] = pkt.sequence
            acc.add_packet(pkt)

        now = time.monotonic()
        if now - last_print >= 0.25:
            counts = acc.sample_counts()
            bar = "  " + " | ".join(
                f"{s.name.lower()}: {counts.get(s, 0)}/{min_samples}"
                for s in required_segments
            )
            print(bar, end="\r", flush=True)
            last_print = now
        time.sleep(0.01)

    print("\n  Calibration complete.\n")
    return acc.build_profile()


def _calibrated_segment_orientations(
    filtered: dict[SegmentId, Rotation],
    buffer: LatestPacketBuffer,
    profile: CalibrationProfile,
    required_segments: tuple[SegmentId, ...],
    max_age_s: float,
) -> dict[SegmentId, Rotation] | None:
    """Return calibrated world-from-segment orientations, or None if any segment is stale.

    Calibration is performed in segment space so that joint angles are zero at
    the neutral pose regardless of each sensor's absolute heading.
    """
    now = time.monotonic()
    seg_orients: dict[SegmentId, Rotation] = {}
    for seg_id in required_segments:
        pkt = buffer.packets.get(seg_id)
        if pkt is None or (now - pkt.receive_time_s) > max_age_s:
            return None
        raw = filtered.get(seg_id)
        if raw is None:
            return None
        mount = _SEGMENT_MOUNTS[seg_id]
        # Calibrate in segment space: neutral_seg^{-1} * live_seg
        # neutral_seg = neutral_sensor * mount^{-1}
        # live_seg    = raw            * mount^{-1}
        # result      = mount * neutral_sensor^{-1} * raw * mount^{-1}
        neutral_seg = profile.neutral_orientations[seg_id] * mount.inv()
        live_seg = raw * mount.inv()
        seg_orients[seg_id] = neutral_seg.inv() * live_seg
    return seg_orients


def main() -> None:
    args = parse_args()
    required_segments = PARTIAL_CONFIGS[args.config]
    max_age_s = args.max_age_ms / 1000.0

    seg_names = ", ".join(s.name.lower() for s in required_segments)
    print(f"Partial IMU live viewer")
    print(f"  config : {args.config} ({len(required_segments)} sensors: {seg_names})")
    print(f"  address: {args.host}:{args.port}")

    # --- background UDP receiver + per-segment spike filter ---
    buffer = LatestPacketBuffer()
    pkt_filter = QuaternionPacketFilter()
    filtered: dict[SegmentId, Rotation] = {}
    filter_lock = threading.Lock()
    stop_event = threading.Event()

    def _receive_loop() -> None:
        for pkt in receive_quaternion_packets(
            host=args.host, port=args.port, timeout_s=0.25
        ):
            if stop_event.is_set():
                break
            buffer.update(pkt)
            result = pkt_filter.update(pkt)
            if result.rotation is not None:
                with filter_lock:
                    filtered[pkt.segment_id] = result.rotation

    recv_thread = threading.Thread(target=_receive_loop, daemon=True)
    recv_thread.start()

    # wait for all required segments to arrive, with periodic status
    print("\nWaiting for IMU packets...")
    last_wait_print = 0.0
    while not all(s in buffer.packets for s in required_segments):
        now = time.monotonic()
        if now - last_wait_print >= 2.0:
            missing = [s.name.lower() for s in required_segments if s not in buffer.packets]
            print(f"  still waiting for: {', '.join(missing)}")
            last_wait_print = now
        time.sleep(0.1)
    print(f"All {len(required_segments)} sensors connected.\n")

    # --- open the window now so the user has visual feedback ---
    plt.ion()
    fig = plt.figure(figsize=(7, 9))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlabel("x forward (m)")
    ax.set_ylabel("y left (m)")
    ax.set_zlabel("z up (m)")
    ax.set_xlim(-0.4, 0.4)
    ax.set_ylim(-0.4, 0.4)
    ax.set_zlim(-0.05, 1.15)
    ax.set_box_aspect((0.8, 0.8, 1.2))
    ax.set_proj_type("ortho")
    ax.view_init(elev=18, azim=-62)

    _L, _R = "#2a9d8f", "#e76f51"
    _LE, _RE = "#8ec8c2", "#f0b49a"

    def _mk(color: str, label: str = "", ls: str = "solid"):
        (ln,) = ax.plot(
            [], [], [],
            color=color, linewidth=3, marker="o", markersize=5,
            label=label, linestyle=ls,
        )
        return ln

    lines = {
        "pelvis":          _mk("#555555", "pelvis"),
        "left_thigh":      _mk(_L, "left"),
        "left_shank":      _mk(_L),
        "left_foot":       _mk(_L),
        "right_thigh":     _mk(_R, "right"),
        "right_shank":     _mk(_R),
        "right_foot":      _mk(_R),
        "left_thigh_est":  _mk(_LE, "estimated", "dashed"),
        "left_shank_est":  _mk(_LE, ls="dashed"),
        "left_foot_est":   _mk(_LE, ls="dashed"),
        "right_thigh_est": _mk(_RE, ls="dashed"),
        "right_shank_est": _mk(_RE, ls="dashed"),
        "right_foot_est":  _mk(_RE, ls="dashed"),
    }
    ax.legend(loc="upper left", fontsize=8)

    status_text = ax.text2D(
        0.02, 0.02, "", transform=ax.transAxes,
        va="bottom", fontsize=8, family="monospace",
    )

    _SEG_KEYS = (
        "left_thigh", "left_shank", "left_foot",
        "right_thigh", "right_shank", "right_foot",
    )

    def _set(key: str, coords: np.ndarray) -> None:
        ln = lines[key]
        ln.set_data(coords[:, 0], coords[:, 1])
        ln.set_3d_properties(coords[:, 2])
        ln.set_visible(True)

    live_title = f"Live pose — {args.config} ({len(required_segments)} IMUs)  |  R = recalibrate"

    # show window immediately so the user knows it's working
    ax.set_title("Waiting to calibrate — check the terminal")
    plt.pause(0.01)

    input("Stand in neutral position, feet shoulder-width apart, then press Enter to calibrate... ")
    ax.set_title("Calibrating — stand still...")
    fig.canvas.draw_idle()
    plt.pause(0.01)

    profile = _calibrate(buffer, required_segments, args.min_samples)
    ax.set_title(live_title)

    state = {"recalibrate": False, "profile": profile}

    def on_key(event) -> None:
        if event.key == "r":
            state["recalibrate"] = True

    fig.canvas.mpl_connect("key_press_event", on_key)

    print("Visualization running.\n")
    print("  Solid lines  = measured segments")
    print("  Dashed lines = estimated (neutral assumed)")
    print("  Press R in the plot window to recalibrate\n")

    while plt.fignum_exists(fig.number):
        if state["recalibrate"]:
            state["recalibrate"] = False
            ax.set_title("Recalibrating — stand still in neutral position...")
            fig.canvas.draw_idle()
            plt.pause(0.01)
            print("\nRecalibration — stand still in neutral position")
            state["profile"] = _calibrate(buffer, required_segments, args.min_samples)
            ax.set_title(live_title)

        with filter_lock:
            filtered_snapshot = dict(filtered)

        seg_orients = _calibrated_segment_orientations(
            filtered_snapshot, buffer, state["profile"], required_segments, max_age_s
        )

        if seg_orients is not None:
            skeleton = aggregate_lower_body_skeleton(seg_orients)
            measured, estimated = lower_body_points_from_skeleton(skeleton)

            if "pelvis" in measured:
                _set("pelvis", measured["pelvis"])

            for key in _SEG_KEYS:
                if key in measured:
                    _set(key, measured[key])
                    lines[f"{key}_est"].set_visible(False)
                else:
                    _set(f"{key}_est", estimated[key])
                    lines[key].set_visible(False)

            now = time.monotonic()
            age_info = "  ".join(
                f"{s.name.lower()}: {1000*(now - buffer.packets[s].receive_time_s):.0f}ms"
                for s in required_segments
            )
            status_text.set_text(age_info)
        else:
            status_text.set_text("waiting for fresh packets…")

        fig.canvas.draw_idle()
        plt.pause(0.04)  # ~25 Hz; also pumps the Tk/Qt event loop

    stop_event.set()
    print("Window closed.")


if __name__ == "__main__":
    main()
