"""Launch the MuJoCo viewer for the lower-body inspection model."""

# Usage (from project root):
#   conda run -p .\.conda python demos\demo_mujoco_lower_body_viewer.py
#   conda run -p .\.conda python demos\demo_mujoco_lower_body_viewer.py --animate --preset walking

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time
import threading
import tkinter as tk
from tkinter import ttk

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import mujoco
import mujoco.viewer
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from simulator.mujoco_lower_body import (  # noqa: E402
    build_lower_body_mjcf,
    clamp_ctrl_to_actuator_ranges,
    format_pose_readout_from_imus,
    lower_body_points_from_skeleton,
    lower_body_skeleton_from_imus,
    pose_overlay_columns_from_imus,
    pose_presets,
    preset_qpos,
    virtual_imu_orientations_from_mujoco,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--animate",
        action="store_true",
        help="Apply a slow periodic motion while the viewer is open",
    )
    parser.add_argument(
        "--preset",
        choices=tuple(pose_presets().keys()),
        default="standing",
        help="Initial pose preset",
    )
    parser.add_argument(
        "--no-control-window",
        action="store_true",
        help="Do not open the separate control window",
    )
    parser.add_argument(
        "--no-estimator-window",
        action="store_true",
        help="Do not open the separate live estimator window",
    )
    return parser.parse_args()


def animated_ctrl(ctrl0: np.ndarray, t: float) -> np.ndarray:
    ctrl = ctrl0.copy()
    phase = np.sin(1.1 * t)
    phase_opposite = np.sin(1.1 * t + np.pi)

    ctrl[1] = ctrl0[1] - 0.18 * phase
    ctrl[3] = ctrl0[3] + 0.28 * max(phase, 0.0)
    ctrl[5] = ctrl0[5] + 0.08 * max(phase, 0.0)
    ctrl[6] = ctrl0[6] + 0.03 * phase

    ctrl[8] = ctrl0[8] - 0.18 * phase_opposite
    ctrl[10] = ctrl0[10] + 0.28 * max(phase_opposite, 0.0)
    ctrl[12] = ctrl0[12] + 0.08 * max(phase_opposite, 0.0)
    ctrl[13] = ctrl0[13] - 0.03 * phase_opposite

    return ctrl


def joint_label(name: str) -> str:
    label = name.replace("_ctrl", "").replace("_", " ")
    label = label.replace("left ", "L ").replace("right ", "R ")
    return label


def main() -> None:
    args = parse_args()
    model = mujoco.MjModel.from_xml_string(build_lower_body_mjcf())
    data = mujoco.MjData(model)
    state_lock = threading.Lock()

    preset = preset_qpos(args.preset)
    state = {
        "ctrl_home": preset.copy(),
        "animate": args.animate,
        "preset_name": args.preset,
    }
    data.qpos[:] = preset
    data.ctrl[:] = preset
    mujoco.mj_forward(model, data)

    manual_controls = {
        "q": ("left hip flex", 1, +5.0),
        "a": ("left hip flex", 1, -5.0),
        "w": ("left knee", 3, +5.0),
        "s": ("left knee", 3, -5.0),
        "e": ("left ankle pitch", 5, +5.0),
        "d": ("left ankle pitch", 5, -5.0),
        "u": ("right hip flex", 8, +5.0),
        "j": ("right hip flex", 8, -5.0),
        "i": ("right knee", 10, +5.0),
        "k": ("right knee", 10, -5.0),
        "o": ("right ankle pitch", 12, +5.0),
        "l": ("right ankle pitch", 12, -5.0),
    }

    def store_ctrl_target(ctrl: np.ndarray, preset_name: str | None = None) -> None:
        ctrl = clamp_ctrl_to_actuator_ranges(model, ctrl)
        with state_lock:
            state["ctrl_home"] = ctrl.copy()
            if preset_name is not None:
                state["preset_name"] = preset_name

    def adjust_ctrl_target(index: int, degrees_delta: float, label: str) -> None:
        with state_lock:
            ctrl = state["ctrl_home"].copy()
            ctrl[index] += np.deg2rad(degrees_delta)
        store_ctrl_target(ctrl, preset_name="custom")
        print(f"{label}: {np.rad2deg(clamp_ctrl_to_actuator_ranges(model, ctrl)[index]):.1f} deg")

    def key_callback(keycode: int) -> None:
        try:
            key = chr(keycode).lower()
        except ValueError:
            return

        if key == "1":
            store_ctrl_target(preset_qpos("standing"), preset_name="standing")
            print("preset: standing")
            return
        if key == "2":
            store_ctrl_target(preset_qpos("squat"), preset_name="squat")
            print("preset: squat")
            return
        if key == "3":
            store_ctrl_target(preset_qpos("step"), preset_name="step")
            print("preset: step")
            return
        if key == " ":
            with state_lock:
                state["animate"] = not state["animate"]
                animate = state["animate"]
            print(f"animate: {animate}")
            return
        if key in manual_controls:
            label, index, degrees_delta = manual_controls[key]
            adjust_ctrl_target(index, degrees_delta, label)

    control_root: tk.Tk | None = None
    estimator_root: tk.Toplevel | None = None
    slider_vars: list[tk.DoubleVar] = []
    animate_var: tk.BooleanVar | None = None
    preset_var: tk.StringVar | None = None
    estimator_text: tk.Text | None = None
    estimator_canvas: FigureCanvasTkAgg | None = None
    estimator_axes = {}
    estimator_lines: dict[str, object] = {}
    suppress_slider_callbacks = {"value": False}
    ui_pause_until = {"time": 0.0}
    last_estimator_refresh = {"time": 0.0}
    estimator_refresh_interval = 0.08
    ui_drag_pause_seconds = 0.22

    def pause_ui_redraws() -> None:
        ui_pause_until["time"] = time.time() + ui_drag_pause_seconds

    def ui_redraw_paused() -> bool:
        return time.time() < ui_pause_until["time"]

    def bind_drag_pause(window: tk.Misc) -> None:
        def on_configure(_event: tk.Event) -> None:
            pause_ui_redraws()

        window.bind("<Configure>", on_configure, add="+")

    def on_slider_change(index: int, raw_value: str) -> None:
        if suppress_slider_callbacks["value"]:
            return
        degrees_value = float(raw_value)
        with state_lock:
            ctrl = state["ctrl_home"].copy()
        ctrl[index] = np.deg2rad(degrees_value)
        store_ctrl_target(ctrl, preset_name="custom")
        if preset_var is not None:
            preset_var.set("custom")

    def sync_slider_vars() -> None:
        with state_lock:
            ctrl = state["ctrl_home"].copy()
            animate_value = state["animate"]
            preset_name = state["preset_name"]
        suppress_slider_callbacks["value"] = True
        for index, var in enumerate(slider_vars):
            target_deg = float(np.rad2deg(ctrl[index]))
            if abs(var.get() - target_deg) > 0.25:
                var.set(target_deg)
        if animate_var is not None:
            animate_var.set(animate_value)
        if preset_var is not None:
            preset_var.set(preset_name)
        suppress_slider_callbacks["value"] = False

    def build_control_window() -> tk.Tk:
        nonlocal animate_var, preset_var
        root = tk.Tk()
        root.title("Lower-body controls")
        root.geometry("620x640")
        bind_drag_pause(root)
        animate_var = tk.BooleanVar(root, value=args.animate)
        preset_var = tk.StringVar(root, value=args.preset)

        header = ttk.Frame(root, padding=(10, 10, 10, 6))
        header.pack(fill="x")
        ttk.Label(
            header,
            text="Lower-body actuator controls",
            font=("Segoe UI", 14, "bold"),
        ).pack(anchor="w")
        ttk.Label(
            header,
            text="These sliders drive the MuJoCo actuators directly.",
        ).pack(anchor="w", pady=(2, 0))

        preset_frame = ttk.Frame(root, padding=(10, 0, 10, 8))
        preset_frame.pack(fill="x")
        ttk.Label(preset_frame, text="Preset").pack(side="left")
        ttk.Label(preset_frame, textvariable=preset_var, width=10).pack(side="left", padx=(8, 12))

        for preset_name in ("standing", "squat", "step"):
            ttk.Button(
                preset_frame,
                text=preset_name.title(),
                command=lambda name=preset_name: store_ctrl_target(preset_qpos(name), preset_name=name),
            ).pack(side="left", padx=4)

        ttk.Checkbutton(
            preset_frame,
            text="Animate",
            variable=animate_var,
            command=lambda: _toggle_animation_from_checkbox(),
        ).pack(side="right")

        body = ttk.Frame(root, padding=(10, 0, 10, 10))
        body.pack(fill="both", expand=True)
        left_frame = ttk.LabelFrame(body, text="Left leg", padding=8)
        left_frame.pack(side="left", fill="both", expand=True, padx=(0, 6))
        right_frame = ttk.LabelFrame(body, text="Right leg", padding=8)
        right_frame.pack(side="left", fill="both", expand=True, padx=(6, 0))

        slider_specs = (
            (left_frame, 0, "L hip roll", 0),
            (left_frame, 1, "L hip pitch", 1),
            (left_frame, 2, "L hip yaw", 2),
            (left_frame, 3, "L knee pitch", 3),
            (left_frame, 4, "L ankle roll", 4),
            (left_frame, 5, "L ankle pitch", 5),
            (left_frame, 6, "L ankle yaw", 6),
            (right_frame, 7, "R hip roll", 7),
            (right_frame, 8, "R hip pitch", 8),
            (right_frame, 9, "R hip yaw", 9),
            (right_frame, 10, "R knee pitch", 10),
            (right_frame, 11, "R ankle roll", 11),
            (right_frame, 12, "R ankle pitch", 12),
            (right_frame, 13, "R ankle yaw", 13),
        )

        slider_vars.extend(tk.DoubleVar(value=float(np.rad2deg(value))) for value in preset)

        for parent, _, label_text, index in slider_specs:
            row = ttk.Frame(parent)
            row.pack(fill="x", pady=3)
            ttk.Label(row, text=label_text, width=14).pack(side="left")
            actuator_range = model.actuator_ctrlrange[index]
            from_deg = float(np.rad2deg(actuator_range[0]))
            to_deg = float(np.rad2deg(actuator_range[1]))
            scale = tk.Scale(
                row,
                variable=slider_vars[index],
                from_=from_deg,
                to=to_deg,
                resolution=1.0,
                orient="horizontal",
                length=180,
                showvalue=True,
                command=lambda value, idx=index: on_slider_change(idx, value),
            )
            scale.pack(side="left", fill="x", expand=True, padx=(8, 0))

        footer = ttk.Frame(root, padding=(10, 0, 10, 10))
        footer.pack(fill="x")
        ttk.Label(
            footer,
            text="Viewer hotkeys still work: 1/2/3, Space, q/a w/s e/d, u/j i/k o/l",
        ).pack(anchor="w")

        def on_close() -> None:
            if viewer_ref["handle"] is not None:
                viewer_ref["handle"].close()
            root.destroy()

        root.protocol("WM_DELETE_WINDOW", on_close)
        return root

    def build_estimator_window(parent: tk.Tk) -> tk.Toplevel:
        nonlocal estimator_text, estimator_canvas, estimator_axes, estimator_lines
        window = tk.Toplevel(parent)
        window.title("Live estimator")
        window.geometry("900x700")
        bind_drag_pause(window)

        frame = ttk.Frame(window, padding=10)
        frame.pack(fill="both", expand=True)
        ttk.Label(
            frame,
            text="Real-time pose estimation view",
            font=("Segoe UI", 13, "bold"),
        ).pack(anchor="w")
        ttk.Label(
            frame,
            text="Live 3D pose view from the current synthetic IMU orientations.",
        ).pack(anchor="w", pady=(2, 8))

        figure = Figure(figsize=(8.8, 5.4), dpi=100)
        pose_axis = figure.add_subplot(111, projection="3d")
        pose_axis.set_title("Estimated lower-body pose")
        pose_axis.set_xlabel("x forward (m)")
        pose_axis.set_ylabel("y left (m)")
        pose_axis.set_zlabel("z up (m)")
        pose_axis.set_xlim(-0.35, 0.35)
        pose_axis.set_ylim(-0.35, 0.35)
        pose_axis.set_zlim(-0.05, 1.15)
        pose_axis.set_box_aspect((0.70, 0.70, 1.20))
        pose_axis.set_proj_type("ortho")
        pose_axis.view_init(elev=18, azim=-62)
        estimator_axes = {"pose3d": pose_axis}

        def init_line(axis, color: str, label: str, linestyle: str = "solid"):
            (line,) = axis.plot(
                [], [], [], color=color, linewidth=3, marker="o",
                label=label, linestyle=linestyle,
            )
            return line

        _L = "#2a9d8f"
        _R = "#e76f51"
        _LE = "#8ec8c2"  # lighter teal for estimated left segments
        _RE = "#f0b49a"  # lighter orange for estimated right segments
        estimator_lines = {
            "pelvis":       init_line(pose_axis, "#333333", "pelvis"),
            "left_thigh":   init_line(pose_axis, _L, "left"),
            "left_shank":   init_line(pose_axis, _L, ""),
            "left_foot":    init_line(pose_axis, _L, ""),
            "right_thigh":  init_line(pose_axis, _R, "right"),
            "right_shank":  init_line(pose_axis, _R, ""),
            "right_foot":   init_line(pose_axis, _R, ""),
            "left_thigh_est":  init_line(pose_axis, _LE, "", linestyle="dashed"),
            "left_shank_est":  init_line(pose_axis, _LE, "", linestyle="dashed"),
            "left_foot_est":   init_line(pose_axis, _LE, "", linestyle="dashed"),
            "right_thigh_est": init_line(pose_axis, _RE, "", linestyle="dashed"),
            "right_shank_est": init_line(pose_axis, _RE, "", linestyle="dashed"),
            "right_foot_est":  init_line(pose_axis, _RE, "", linestyle="dashed"),
        }

        pose_axis.legend(loc="upper left", fontsize=9)
        estimator_canvas = FigureCanvasTkAgg(figure, master=frame)
        estimator_canvas.get_tk_widget().pack(fill="both", expand=True, pady=(0, 8))

        text = tk.Text(
            frame,
            wrap="word",
            font=("Consolas", 11),
            background="#f7f7f7",
            foreground="#111111",
            relief="solid",
            borderwidth=1,
            height=10,
        )
        text.pack(fill="x", expand=False)
        text.configure(state="disabled")

        def on_close() -> None:
            window.withdraw()

        window.protocol("WM_DELETE_WINDOW", on_close)
        estimator_text = text
        return window

    def update_estimator_pose_view(imu: dict) -> None:
        if estimator_canvas is None:
            return

        skeleton = lower_body_skeleton_from_imus(imu)
        measured, estimated = lower_body_points_from_skeleton(skeleton)

        _seg_keys = (
            "left_thigh", "left_shank", "left_foot",
            "right_thigh", "right_shank", "right_foot",
        )

        def _set(line_key: str, coords: np.ndarray) -> None:
            line = estimator_lines[line_key]
            line.set_data(coords[:, 0], coords[:, 1])
            line.set_3d_properties(coords[:, 2])
            line.set_visible(True)

        pelvis_pts = measured.get("pelvis")
        if pelvis_pts is not None:
            _set("pelvis", pelvis_pts)

        for key in _seg_keys:
            if key in measured:
                _set(key, measured[key])
                estimator_lines[f"{key}_est"].set_visible(False)
            else:
                _set(f"{key}_est", estimated[key])
                estimator_lines[key].set_visible(False)

        estimator_canvas.draw_idle()

    def refresh_estimator_ui(
        imu,
        *,
        preset_name: str,
        animate: bool,
    ) -> None:
        if estimator_root is None or not estimator_root.winfo_exists():
            return
        if estimator_text is None:
            return
        if ui_redraw_paused():
            return

        now = time.time()
        if now - last_estimator_refresh["time"] < estimator_refresh_interval:
            return

        readout = format_pose_readout_from_imus(
            imu,
            preset_name=preset_name,
            animate=animate,
        )
        update_estimator_pose_view(imu)
        estimator_text.configure(state="normal")
        estimator_text.delete("1.0", "end")
        estimator_text.insert("1.0", readout)
        estimator_text.configure(state="disabled")
        last_estimator_refresh["time"] = now

    def _toggle_animation_from_checkbox() -> None:
        with state_lock:
            state["animate"] = bool(animate_var.get())

    print("MuJoCo lower-body viewer")
    print("  Mouse drag / scroll: orbit and zoom")
    print("  separate control window provides real sliders")
    print("  separate estimator window shows the live solver/readout")
    print("  1/2/3: standing / squat / step presets")
    print("  Space: toggle slow animation")
    print("  q/a w/s e/d: left hip, knee, ankle pitch")
    print("  u/j i/k o/l: right hip, knee, ankle pitch")
    if state["animate"]:
        print("  running in slow inspection animation mode")

    viewer_ref: dict[str, mujoco.viewer.Handle | None] = {"handle": None}

    with mujoco.viewer.launch_passive(
        model,
        data,
        key_callback=key_callback,
        show_left_ui=False,
        show_right_ui=False,
    ) as viewer:
        viewer_ref["handle"] = viewer
        with viewer.lock():
            viewer.opt.frame = mujoco.mjtFrame.mjFRAME_SITE
        start = time.time()
        last_overlay = 0.0

        if not args.no_control_window:
            control_root = build_control_window()
            if not args.no_estimator_window:
                estimator_root = build_estimator_window(control_root)

        def tick() -> None:
            nonlocal last_overlay
            if not viewer.is_running():
                if control_root is not None and control_root.winfo_exists():
                    control_root.destroy()
                return

            with state_lock:
                ctrl_home = state["ctrl_home"].copy()
                animate = bool(state["animate"])
                preset_name = str(state["preset_name"])

            if animate:
                target_qpos = clamp_ctrl_to_actuator_ranges(
                    model,
                    animated_ctrl(ctrl_home, time.time() - start),
                )
            else:
                target_qpos = clamp_ctrl_to_actuator_ranges(model, ctrl_home)

            data.ctrl[:] = target_qpos
            data.qpos[:] = data.qpos + 0.16 * (target_qpos - data.qpos)
            data.qvel[:] = 0.0
            mujoco.mj_forward(model, data)
            imu = virtual_imu_orientations_from_mujoco(model, data)

            with viewer.lock():
                viewer.opt.frame = mujoco.mjtFrame.mjFRAME_SITE
                now = time.time()
                if now - last_overlay > 0.1:
                    left_text, right_text = pose_overlay_columns_from_imus(
                        imu,
                        preset_name=preset_name,
                        animate=animate,
                    )
                    viewer.set_texts(
                        [
                            (
                                mujoco.mjtFontScale.mjFONTSCALE_150,
                                mujoco.mjtGridPos.mjGRID_TOPLEFT,
                                left_text,
                                "",
                            ),
                            (
                                mujoco.mjtFontScale.mjFONTSCALE_150,
                                mujoco.mjtGridPos.mjGRID_TOPRIGHT,
                                right_text,
                                "",
                            ),
                        ]
                    )
                    last_overlay = now
            viewer.sync()

            if control_root is not None and control_root.winfo_exists():
                if not ui_redraw_paused():
                    sync_slider_vars()
                refresh_estimator_ui(
                    imu,
                    preset_name=preset_name,
                    animate=animate,
                )
                control_root.after(10, tick)

        if control_root is not None:
            control_root.after(10, tick)
            control_root.mainloop()
        else:
            while viewer.is_running():
                tick()
                time.sleep(0.01)


if __name__ == "__main__":
    main()
