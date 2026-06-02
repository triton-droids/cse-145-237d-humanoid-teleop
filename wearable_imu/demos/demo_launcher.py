"""Clickable launcher for the demo workflows.

Usage (from project root):
  conda run --no-capture-output -n humanoid-sim python demos\demo_launcher.py
  python demos/demo_launcher.py
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import os
from pathlib import Path
import queue
import shlex
import subprocess
import sys
import threading
import tkinter as tk
from tkinter import messagebox, ttk


PROJECT_ROOT = Path(__file__).resolve().parents[1]
ANSI_CLEAR_HOME = "\033[2J\033[H"
DEFAULT_CAMERA_CLIP = "../data/smplh_capture_3.jsonl"
DEFAULT_HUMAN_CLIP = "../data/human_joint_clip_20260601_231345.npz"
DEFAULT_ZMQ_ENDPOINT = "tcp://127.0.0.1:5556"
DEFAULT_UDP_HOST = "0.0.0.0"
DEFAULT_IMU_PORT = "5005"


def _pick_font(root: tk.Misc, candidates: tuple[str, ...], fallback: str) -> str:
    """Return the first installed font family, else a Tk-guaranteed fallback."""
    try:
        import tkinter.font as tkfont

        available = {name.lower() for name in tkfont.families(root)}
    except Exception:
        return fallback
    for name in candidates:
        if name.lower() in available:
            return name
    return fallback


# Cross-platform font families. The originals were macOS-only (Avenir Next /
# Menlo), so on Linux/Windows Tk silently fell back to a tiny default font,
# which looked low-resolution. These are resolved at runtime to whatever is
# actually installed.
_UI_FONT_CANDIDATES = (
    "Avenir Next", "Segoe UI", "Cantarell", "Ubuntu", "Noto Sans",
    "DejaVu Sans", "Helvetica Neue", "Arial", "Helvetica",
)
_MONO_FONT_CANDIDATES = (
    "Menlo", "Cascadia Mono", "Consolas", "Ubuntu Mono", "Noto Sans Mono",
    "DejaVu Sans Mono", "Liberation Mono", "Courier New", "Courier",
)


@dataclass(frozen=True)
class OptionGroup:
    """A mutually-exclusive choice exposed as radio buttons on the right panel.

    The selected value is appended to the command as ``flag value`` (e.g.
    ``--config full``), so one demo entry can cover several variants without
    cluttering the demo list.
    """

    key: str
    label: str
    flag: str
    choices: tuple[str, ...]
    default: str

    def args_for(self, value: str) -> tuple[str, ...]:
        return (self.flag, value)


@dataclass(frozen=True)
class Field:
    """A free-text / numeric argument exposed as a labeled entry box.

    Emits ``flag value`` only when the box is non-empty, so leaving it blank
    falls back to the script's own default. An empty ``flag`` makes this a
    positional argument (just the value, no flag).
    """

    key: str
    label: str
    flag: str
    default: str = ""
    placeholder: str = ""

    def args_for(self, value: str) -> tuple[str, ...]:
        value = value.strip()
        if not value:
            return ()
        return (value,) if not self.flag else (self.flag, value)


@dataclass(frozen=True)
class Toggle:
    """A boolean ``store_true`` flag exposed as a checkbox.

    Emits the flag when checked, nothing when unchecked.
    """

    key: str
    label: str
    flag: str
    default: bool = False

    def args_for(self, checked: bool) -> tuple[str, ...]:
        return (self.flag,) if checked else ()


@dataclass(frozen=True)
class DemoSpec:
    key: str
    title: str
    script: Path
    description: str
    default_args: tuple[str, ...] = ()
    notes: str = ""
    needs_args: bool = False
    options: tuple[OptionGroup, ...] = ()
    fields: tuple[Field, ...] = ()
    toggles: tuple[Toggle, ...] = ()

    def command(self, extra_args: tuple[str, ...] = ()) -> list[str]:
        return [sys.executable, "-u", str(self.script), *self.default_args, *extra_args]


DEMOS: tuple[DemoSpec, ...] = (
    DemoSpec(
        key="imu-orientation-ik",
        title="IMU Orientation IK",
        script=PROJECT_ROOT / "demos" / "demo_imu_orientation_ik.py",
        description="Recover relative hip, knee, and ankle rotations from synthetic mounted IMU orientations.",
    ),
    DemoSpec(
        key="inverse-kinematics",
        title="Planar Inverse Kinematics",
        script=PROJECT_ROOT / "demos" / "demo_inverse_kinematics.py",
        description="Solve a toy lower-body leg IK target and save a plot.",
        default_args=("--plot",),
    ),
    DemoSpec(
        key="lower-body-aggregation",
        title="Lower-Body Aggregation",
        script=PROJECT_ROOT / "demos" / "demo_lower_body_aggregation.py",
        description="Aggregate one static seven-segment pose and print the skeleton joints and joint rotations.",
    ),
    DemoSpec(
        key="live-lower-body-aggregation",
        title="Live Lower-Body Aggregation",
        script=PROJECT_ROOT / "demos" / "demo_live_lower_body_aggregation.py",
        description="Animate a synthetic walking-like lower-body skeleton with Matplotlib.",
        notes="Close the Matplotlib window or click Stop to end it.",
    ),
    DemoSpec(
        key="visualize-lower-body-model",
        title="Visualize Lower-Body Model",
        script=PROJECT_ROOT / "demos" / "demo_visualize_lower_body_model.py",
        description="Render the lower-body model and IMU mounts to data/visualizations/lower_body_model.png.",
    ),
    DemoSpec(
        key="mujoco-lower-body-viewer",
        title="MuJoCo Lower-Body Viewer",
        script=PROJECT_ROOT / "demos" / "demo_mujoco_lower_body_viewer.py",
        description="Open the MuJoCo lower-body viewer with control and estimator windows.",
        notes="Requires the MuJoCo Python package and display support.",
    ),
    DemoSpec(
        key="partial-imu-live-viewer",
        title="Partial IMU Viewer + Recorder",
        script=PROJECT_ROOT / "demos" / "demo_partial_imu_live_viewer.py",
        description="Live lower-body skeleton from real IMU packets, with calibration and ML clip recording controls.",
        notes="Pick the IMU set on the right, set recording options below, then click Calibrate and Record in the plot window.",
        options=(
            OptionGroup(
                key="config",
                label="IMU set",
                flag="--config",
                choices=("thighs", "shanks", "full"),
                default="thighs",
            ),
        ),
        fields=(
            Field("host", "Host", "--host", default=DEFAULT_UDP_HOST, placeholder="0.0.0.0"),
            Field("port", "Port", "--port", default=DEFAULT_IMU_PORT, placeholder="5005"),
            Field("record_fps", "Record FPS", "--record-fps", default="50", placeholder="50"),
            Field("draw_fps", "Draw FPS", "--draw-fps", default="10", placeholder="10"),
            Field("record_duration_s", "Record seconds", "--record-duration-s", default="10", placeholder="10"),
            Field("record_output", "Output .npz", "--record-output", default="data/recordings/live_human_joint_clip.npz", placeholder="data/recordings/live_human_joint_clip.npz"),
        ),
    ),
    DemoSpec(
        key="live-ch-robot-retarget",
        title="Live ch_robot Retarget",
        script=PROJECT_ROOT / "demos" / "demo_live_retarget.py",
        description="Headless Plan A bridge: live IMU packets to ch_robot qpos/qvel JSON over stdout or UDP.",
        notes="Use stdout for terminal inspection, udp for a downstream sim or policy process.",
        options=(
            OptionGroup(
                key="config",
                label="IMU set",
                flag="--config",
                choices=("thighs", "shanks", "full"),
                default="shanks",
            ),
            OptionGroup(
                key="output",
                label="Output",
                flag="--output",
                choices=("stdout", "udp", "none"),
                default="stdout",
            ),
            OptionGroup(
                key="yaw_mode",
                label="Base yaw",
                flag="--yaw-mode",
                choices=("keep", "strip"),
                default="keep",
            ),
        ),
        fields=(
            Field("host", "Host", "--host", default=DEFAULT_UDP_HOST, placeholder="0.0.0.0"),
            Field("port", "Port", "--port", default=DEFAULT_IMU_PORT, placeholder="5005"),
            Field("fps", "Output FPS", "--fps", default="100", placeholder="100"),
            Field("target_host", "Target host", "--target-host", default="127.0.0.1", placeholder="127.0.0.1"),
            Field("target_port", "Target port", "--target-port", default="6010", placeholder="6010"),
        ),
        toggles=(
            Toggle("no_calibration", "Skip calibration", "--no-calibration"),
        ),
    ),
    DemoSpec(
        key="replay-ch-robot-retarget",
        title="Replay ch_robot Retarget",
        script=PROJECT_ROOT / "demos" / "demo_replay_ch_robot_retarget.py",
        description="Replay a recorded human_joint_clip .npz or camera .jsonl through Plan A and visualize ch_robot joint angles.",
        notes="Enter the recorded clip path below. Use Save .npz to export qpos/qvel.",
        options=(
            OptionGroup(
                key="frame_key",
                label="Frame key",
                flag="--frame-key",
                choices=("joint_pos_origin", "joint_pos_w"),
                default="joint_pos_origin",
            ),
            OptionGroup(
                key="yaw_mode",
                label="Base yaw",
                flag="--yaw-mode",
                choices=("keep", "strip"),
                default="keep",
            ),
            OptionGroup(
                key="base_motion",
                label="Base motion",
                flag="--base-motion",
                choices=("root_xy", "fixed", "root_xyz"),
                default="root_xy",
            ),
        ),
        fields=(
            Field("clip", "Clip path", "", default=DEFAULT_CAMERA_CLIP, placeholder="../data/smplh_capture_3.jsonl"),
            Field("save_output", "Save .npz", "--save-output", placeholder="../data/ch_robot_replay_qpos.npz"),
            Field("speed", "Speed", "--speed", default="1.0", placeholder="1.0"),
        ),
        toggles=(
            Toggle("no_show", "No window", "--no-show"),
        ),
        needs_args=True,
    ),
    DemoSpec(
        key="mujoco-ch-robot-replay",
        title="MuJoCo ch_robot Replay",
        script=PROJECT_ROOT / "demos" / "demo_mujoco_ch_robot_replay.py",
        description="Replay a recorded IMU handoff clip or qpos replay on the real Holosoma ch_robot MuJoCo model.",
        notes="First run extracts ch_robot XML/meshes from retargeting_holosoma into .cache/ch_robot_model.",
        options=(
            OptionGroup(
                key="frame_key",
                label="Frame key",
                flag="--frame-key",
                choices=("joint_pos_origin", "joint_pos_w"),
                default="joint_pos_origin",
            ),
            OptionGroup(
                key="yaw_mode",
                label="Base yaw",
                flag="--yaw-mode",
                choices=("keep", "strip"),
                default="keep",
            ),
            OptionGroup(
                key="base_motion",
                label="Base motion",
                flag="--base-motion",
                choices=("root_xy", "fixed", "root_xyz"),
                default="root_xy",
            ),
        ),
        fields=(
            Field("input", "Input path", "", default=DEFAULT_CAMERA_CLIP, placeholder="../data/smplh_capture_3.jsonl"),
            Field("speed", "Speed", "--speed", default="1.0", placeholder="1.0"),
        ),
        toggles=(
            Toggle("no_show", "No window", "--no-show"),
            Toggle("refresh_model", "Refresh model", "--refresh-model"),
        ),
        needs_args=True,
    ),
    DemoSpec(
        key="compare-human-ch-robot",
        title="Compare Human + ch_robot",
        script=PROJECT_ROOT / "demos" / "demo_compare_human_clip_ch_robot.py",
        description="Open the raw human .npz skeleton player and retargeted ch_robot MuJoCo replay together.",
        notes="Use a recorded human_joint_clip .npz. Closing either window stops both child demos.",
        options=(
            OptionGroup(
                key="frame_key",
                label="Frame key",
                flag="--frame-key",
                choices=("joint_pos_origin", "joint_pos_w"),
                default="joint_pos_origin",
            ),
            OptionGroup(
                key="yaw_mode",
                label="Base yaw",
                flag="--yaw-mode",
                choices=("keep", "strip"),
                default="keep",
            ),
            OptionGroup(
                key="base_motion",
                label="Base motion",
                flag="--base-motion",
                choices=("root_xy", "fixed", "root_xyz"),
                default="root_xy",
            ),
        ),
        fields=(
            Field("clip", "Clip .npz", "", default=DEFAULT_HUMAN_CLIP, placeholder="../data/human_joint_clip_20260601_231345.npz"),
            Field("speed", "Speed", "--speed", default="1.0", placeholder="1.0"),
        ),
        toggles=(
            Toggle("human_origin", "Human pelvis-relative", "--human-origin"),
            Toggle("no_show", "No window", "--no-show"),
            Toggle("refresh_model", "Refresh model", "--refresh-model"),
            Toggle("no_loop", "No robot loop", "--no-loop"),
        ),
        needs_args=True,
    ),
    DemoSpec(
        key="zmq-human-joint-publisher",
        title="ZMQ Human Joint Publisher",
        script=PROJECT_ROOT / "demos" / "demo_zmq_human_joint_publisher.py",
        description="Publish a recorded human_joint_clip .npz or camera .jsonl as mock-live 9-joint frames over ZeroMQ.",
        notes="Run this first, then run the MuJoCo ZMQ subscriber.",
        options=(
            OptionGroup(
                key="frame_key",
                label="Frame key",
                flag="--frame-key",
                choices=("joint_pos_origin", "joint_pos_w"),
                default="joint_pos_origin",
            ),
        ),
        fields=(
            Field("clip", "Clip path", "", default=DEFAULT_CAMERA_CLIP, placeholder="../data/smplh_capture_3.jsonl"),
            Field("endpoint", "Endpoint", "--endpoint", default=DEFAULT_ZMQ_ENDPOINT, placeholder="tcp://127.0.0.1:5556"),
            Field("fps", "Publish FPS", "--fps", default="50", placeholder="50"),
            Field("max_frames", "Max frames", "--max-frames", default="0", placeholder="0"),
        ),
        toggles=(
            Toggle("no_loop", "No loop", "--no-loop"),
        ),
        needs_args=True,
    ),
    DemoSpec(
        key="mujoco-ch-robot-zmq",
        title="MuJoCo ch_robot ZMQ",
        script=PROJECT_ROOT / "demos" / "demo_mujoco_ch_robot_zmq.py",
        description="Subscribe to mock-live ZMQ human joints and drive the real ch_robot MuJoCo model online.",
        notes="Start the ZMQ Human Joint Publisher first.",
        options=(
            OptionGroup(
                key="yaw_mode",
                label="Base yaw",
                flag="--yaw-mode",
                choices=("keep", "strip"),
                default="keep",
            ),
            OptionGroup(
                key="base_motion",
                label="Base motion",
                flag="--base-motion",
                choices=("root_xy", "fixed", "root_xyz"),
                default="root_xy",
            ),
        ),
        fields=(
            Field("endpoint", "Endpoint", "--endpoint", default=DEFAULT_ZMQ_ENDPOINT, placeholder="tcp://127.0.0.1:5556"),
            Field("max_frames", "Max frames", "--max-frames", default="0", placeholder="0"),
        ),
        toggles=(
            Toggle("no_show", "No window", "--no-show"),
            Toggle("refresh_model", "Refresh model", "--refresh-model"),
        ),
    ),
    DemoSpec(
        key="play-human-joint-clip",
        title="Play Human Joint Clip",
        script=PROJECT_ROOT / "demos" / "demo_play_human_joint_clip.py",
        description="Play back a recorded .npz joint clip as an animated 3D skeleton.",
        notes="Enter the clip path below. Origin shows pelvis-relative joints; Speed multiplies playback rate.",
        fields=(
            Field("clip", "Clip .npz", "", default=DEFAULT_HUMAN_CLIP, placeholder="data/recordings/live_human_joint_clip.npz"),
            Field("speed", "Speed", "--speed", default="1.0", placeholder="1.0"),
        ),
        toggles=(
            Toggle("origin", "Pelvis-relative (--origin)", "--origin"),
        ),
        needs_args=True,
    ),
    DemoSpec(
        key="udp-quaternion-receiver",
        title="UDP Quaternion Receiver",
        script=PROJECT_ROOT / "demos" / "demo_udp_quaternion_receiver.py",
        description="Monitor live ESP32/BNO085 quaternion packet status over UDP.",
        notes="Requires ESP32/BNO085 nodes streaming UDP packets.",
        fields=(
            Field("host", "Host", "--host", default=DEFAULT_UDP_HOST, placeholder="0.0.0.0"),
            Field("port", "Port", "--port", default=DEFAULT_IMU_PORT, placeholder="5005"),
        ),
    ),
    DemoSpec(
        key="udp-latency-ping",
        title="UDP Latency Ping",
        script=PROJECT_ROOT / "demos" / "demo_udp_latency_ping.py",
        description="Measure UDP round-trip latency to one ESP32 node.",
        notes="Enter the ESP32 IP address (from Serial Monitor) below.",
        fields=(
            Field("esp32_ip", "ESP32 IP", "", default="192.168.4.20", placeholder="192.168.4.20"),
            Field("port", "Port", "--port", default="5006", placeholder="5006"),
        ),
        needs_args=True,
    ),
)


class DemoLauncher(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("Humanoid Teleop Demo Launcher")

        # --- HiDPI / font setup --------------------------------------------
        # Tk does not auto-detect HiDPI on Linux, so on scaled displays the UI
        # renders at 1x and looks tiny/blurry. Derive a scale factor from the
        # real screen DPI and apply it to Tk's scaling plus our geometry/fonts.
        try:
            dpi = self.winfo_fpixels("1i")  # pixels per inch
        except tk.TclError:
            dpi = 96.0
        scale = dpi / 96.0
        # Some desktops (GNOME fractional scaling, Wayland) keep the X DPI at 96
        # and expose the scale via env vars instead, which leaves Tk tiny. Honor
        # those, and allow a manual override, so the launcher matches the rest of
        # the desktop. Set HUMANOID_UI_SCALE=1.5 (etc.) to force a value.
        for env_var in ("HUMANOID_UI_SCALE", "GDK_SCALE", "QT_SCALE_FACTOR"):
            raw = os.environ.get(env_var)
            if raw:
                try:
                    scale = max(scale, float(raw))
                    break
                except ValueError:
                    pass
        self.ui_scale = max(1.0, min(scale, 3.0))
        # Tk 'scaling' is points->pixels; 1 pt = 1/72 in. Setting it makes Tk
        # size point-based fonts/widgets for the effective DPI.
        self.tk.call("tk", "scaling", self.ui_scale * 96.0 / 72.0)

        self.ui_font = _pick_font(self, _UI_FONT_CANDIDATES, "TkDefaultFont")
        self.mono_font = _pick_font(self, _MONO_FONT_CANDIDATES, "TkFixedFont")

        # Tk 'scaling' already enlarges point-sized fonts for the real DPI, so
        # font point sizes stay as-is. _px scales explicit *pixel* quantities
        # (padding, geometry, wraplengths) that Tk would otherwise leave at 1x.
        def _px(pixels: float) -> int:
            return max(1, int(round(pixels * self.ui_scale)))

        self._px = _px
        self.geometry(f"{_px(1040)}x{_px(720)}")
        self.minsize(_px(900), _px(600))

        self.output_queue: queue.Queue[str] = queue.Queue()
        self.process: subprocess.Popen[str] | None = None
        self.selected_demo = tk.StringVar(value=DEMOS[0].key)
        self.extra_args = tk.StringVar(value="")
        self.status = tk.StringVar(value="Ready")
        self.live_frame_buffer: str | None = None

        self._configure_styles()
        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(80, self._drain_output_queue)
        self.after(300, self._poll_process)

    def _configure_styles(self) -> None:
        self.configure(background="#f3f4f1")
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        ui = self.ui_font
        mono = self.mono_font
        style.configure(".", font=(ui, 12), background="#f3f4f1", foreground="#1f2421")
        style.configure("Root.TFrame", background="#f3f4f1")
        style.configure("Panel.TFrame", background="#ffffff", relief="flat")
        style.configure("Sidebar.TFrame", background="#e8ebe4")
        style.configure("Header.TLabel", font=(ui, 24, "bold"), background="#f3f4f1", foreground="#1d2620")
        style.configure("Subtle.TLabel", background="#ffffff", foreground="#5f6860")
        style.configure("PanelTitle.TLabel", font=(ui, 18, "bold"), background="#ffffff", foreground="#1d2620")
        style.configure("Section.TLabel", font=(ui, 12, "bold"), background="#ffffff", foreground="#384139")
        style.configure("SidebarTitle.TLabel", font=(ui, 13, "bold"), background="#e8ebe4", foreground="#1d2620")
        style.configure("Status.TLabel", background="#dce8d9", foreground="#1f5b35", padding=(12, 5))
        style.configure("Command.TLabel", background="#eef1ec", foreground="#28322b", padding=(10, 8), font=(mono, 11))
        style.configure("TEntry", fieldbackground="#ffffff", bordercolor="#cbd2c8", lightcolor="#cbd2c8", darkcolor="#cbd2c8")
        style.configure("Primary.TButton", font=(ui, 12, "bold"), background="#23614a", foreground="#ffffff", padding=(14, 8))
        style.map("Primary.TButton", background=[("active", "#1d513d"), ("pressed", "#163d2f")])
        style.configure("Secondary.TButton", background="#e4e8e1", foreground="#243027", padding=(14, 8))
        style.map("Secondary.TButton", background=[("active", "#d8dfd4"), ("pressed", "#cbd5c7")])
        style.configure("Danger.TButton", background="#ead9d3", foreground="#733221", padding=(14, 8))
        style.map("Danger.TButton", background=[("active", "#dfc9c0"), ("pressed", "#d0b2a8")])
        # Segmented-control look for per-demo option choices (Toolbutton radios).
        style.configure("Option.Toolbutton", font=(ui, 11), padding=(14, 6), background="#e4e8e1", foreground="#243027")
        style.map(
            "Option.Toolbutton",
            background=[("selected", "#23614a"), ("active", "#d8dfd4")],
            foreground=[("selected", "#ffffff")],
        )

    def _build_ui(self) -> None:
        root = ttk.Frame(self, padding=18, style="Root.TFrame")
        root.pack(fill="both", expand=True)

        header = ttk.Frame(root, style="Root.TFrame")
        header.pack(fill="x", pady=(0, 16))
        ttk.Label(
            header,
            text="Humanoid Teleop",
            style="Header.TLabel",
        ).pack(side="left")
        ttk.Label(header, textvariable=self.status, style="Status.TLabel").pack(side="right")

        body = ttk.Frame(root, style="Root.TFrame")
        body.pack(fill="both", expand=True)
        body.columnconfigure(0, weight=0, minsize=280)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)

        sidebar = ttk.Frame(body, padding=14, style="Sidebar.TFrame")
        sidebar.grid(row=0, column=0, sticky="nsew", padx=(0, 14))
        ttk.Label(sidebar, text="Demos", style="SidebarTitle.TLabel").pack(anchor="w", pady=(0, 10))

        self.demo_listbox = tk.Listbox(
            sidebar,
            activestyle="none",
            background="#f8faf6",
            borderwidth=0,
            exportselection=False,
            foreground="#263029",
            highlightbackground="#d5dbd1",
            highlightcolor="#23614a",
            highlightthickness=1,
            relief="flat",
            selectbackground="#23614a",
            selectforeground="#ffffff",
            font=(self.ui_font, 12),
        )
        self.demo_listbox.pack(fill="both", expand=True)
        for demo in DEMOS:
            self.demo_listbox.insert("end", demo.title)
        self.demo_listbox.selection_set(0)
        self.demo_listbox.bind("<<ListboxSelect>>", self._on_demo_selected)

        controls = ttk.Frame(body, style="Root.TFrame")
        controls.grid(row=0, column=1, sticky="nsew")
        controls.columnconfigure(0, weight=1)
        controls.rowconfigure(3, weight=1)

        details = ttk.Frame(controls, padding=16, style="Panel.TFrame")
        details.grid(row=0, column=0, sticky="ew")
        details.columnconfigure(0, weight=1)

        self.title_label = ttk.Label(details, style="PanelTitle.TLabel")
        self.title_label.grid(row=0, column=0, sticky="w")
        self.description_label = ttk.Label(details, wraplength=self._px(650), justify="left", style="Subtle.TLabel")
        self.description_label.grid(row=1, column=0, sticky="ew", pady=(6, 0))
        self.notes_label = ttk.Label(details, wraplength=self._px(650), justify="left", style="Subtle.TLabel")
        self.notes_label.grid(row=2, column=0, sticky="ew", pady=(6, 0))

        # Per-demo option groups (radio buttons) are rebuilt on demo selection.
        self.options_frame = ttk.Frame(details, style="Panel.TFrame")
        self.options_frame.grid(row=3, column=0, sticky="ew", pady=(14, 0))
        self.option_vars: dict[str, tk.StringVar] = {}

        # Per-demo entry fields + checkboxes, also rebuilt on selection.
        self.fields_frame = ttk.Frame(details, style="Panel.TFrame")
        self.fields_frame.grid(row=4, column=0, sticky="ew", pady=(10, 0))
        self.field_vars: dict[str, tk.StringVar] = {}
        self.toggle_vars: dict[str, tk.BooleanVar] = {}
        self._placeholder_active: dict[str, bool] = {}

        args_frame = ttk.Frame(details, style="Panel.TFrame")
        args_frame.grid(row=5, column=0, sticky="ew", pady=(14, 0))
        args_frame.columnconfigure(1, weight=1)
        ttk.Label(args_frame, text="Extra args", style="Section.TLabel").grid(row=0, column=0, sticky="w", padx=(0, 10))
        args_entry = ttk.Entry(args_frame, textvariable=self.extra_args)
        args_entry.grid(row=0, column=1, sticky="ew")
        args_entry.bind("<KeyRelease>", lambda _event: self._refresh_command_label())

        button_frame = ttk.Frame(details, style="Panel.TFrame")
        button_frame.grid(row=6, column=0, sticky="ew", pady=(14, 0))
        ttk.Button(button_frame, text="Run Demo", command=self._run_selected_demo, style="Primary.TButton").pack(side="left")
        ttk.Button(button_frame, text="Stop", command=self._stop_process, style="Danger.TButton").pack(side="left", padx=(8, 0))
        ttk.Button(button_frame, text="Clear Output", command=self._clear_output, style="Secondary.TButton").pack(side="left", padx=(8, 0))

        command_frame = ttk.Frame(controls, padding=(0, 12, 0, 0), style="Root.TFrame")
        command_frame.grid(row=1, column=0, sticky="ew")
        ttk.Label(command_frame, text="Command", style="Section.TLabel").pack(anchor="w", pady=(0, 6))
        self.command_label = ttk.Label(command_frame, wraplength=self._px(720), justify="left", style="Command.TLabel")
        self.command_label.pack(anchor="w", fill="x")

        output_frame = ttk.Frame(controls, padding=(0, 12, 0, 0), style="Root.TFrame")
        output_frame.grid(row=3, column=0, sticky="nsew")
        ttk.Label(output_frame, text="Output", style="Section.TLabel").pack(anchor="w", pady=(0, 6))
        terminal = ttk.Frame(output_frame, padding=1)
        terminal.pack(fill="both", expand=True)
        self.output = tk.Text(
            terminal,
            wrap="word",
            height=20,
            font=(self.mono_font, 11),
            background="#111814",
            foreground="#dbe7de",
            insertbackground="#dbe7de",
            borderwidth=0,
            padx=12,
            pady=10,
            relief="flat",
        )
        self.output.tag_configure("command", foreground="#8fd6a4")
        self.output.tag_configure("error", foreground="#f1a08f")
        self.output.tag_configure("muted", foreground="#93a197")
        scrollbar = ttk.Scrollbar(terminal, orient="vertical", command=self.output.yview)
        self.output.configure(yscrollcommand=scrollbar.set)
        self.output.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        self._sync_selected_demo()

    def _on_demo_selected(self, _event: tk.Event) -> None:
        selection = self.demo_listbox.curselection()
        if not selection:
            return
        self.selected_demo.set(DEMOS[selection[0]].key)
        self._sync_selected_demo()

    def _sync_selected_demo(self) -> None:
        demo = self._demo_by_key(self.selected_demo.get())
        self.title_label.configure(text=demo.title)
        self.description_label.configure(text=demo.description)
        self.notes_label.configure(text=demo.notes)
        self._rebuild_options(demo)
        self._refresh_command_label()

    def _rebuild_options(self, demo: DemoSpec) -> None:
        """Render the selected demo's option groups, fields, and toggles."""
        # --- radio-button option groups ---
        for child in self.options_frame.winfo_children():
            child.destroy()
        self.option_vars = {}
        if not demo.options:
            self.options_frame.grid_remove()
        else:
            self.options_frame.grid()
            for row, group in enumerate(demo.options):
                ttk.Label(self.options_frame, text=group.label, style="Section.TLabel").grid(
                    row=row, column=0, sticky="w", padx=(0, 12), pady=(0, 4)
                )
                choices = ttk.Frame(self.options_frame, style="Panel.TFrame")
                choices.grid(row=row, column=1, sticky="w", pady=(0, 4))
                var = tk.StringVar(value=group.default)
                self.option_vars[group.key] = var
                for choice in group.choices:
                    ttk.Radiobutton(
                        choices,
                        text=choice,
                        value=choice,
                        variable=var,
                        style="Option.Toolbutton",
                        command=self._refresh_command_label,
                    ).pack(side="left", padx=(0, 6))

        # --- entry fields + checkboxes ---
        for child in self.fields_frame.winfo_children():
            child.destroy()
        self.field_vars = {}
        self.toggle_vars = {}
        if not demo.fields and not demo.toggles:
            self.fields_frame.grid_remove()
            return
        self.fields_frame.grid()
        self.fields_frame.columnconfigure(1, weight=1)

        row = 0
        for field in demo.fields:
            ttk.Label(self.fields_frame, text=field.label, style="Section.TLabel").grid(
                row=row, column=0, sticky="w", padx=(0, 12), pady=3
            )
            var = tk.StringVar(value=field.default)
            self.field_vars[field.key] = var
            entry = ttk.Entry(self.fields_frame, textvariable=var)
            entry.grid(row=row, column=1, sticky="ew", pady=3)
            self._placeholder_active[field.key] = False
            var.trace_add("write", lambda *_a: self._refresh_command_label())
            if field.placeholder and not field.default:
                self._add_placeholder(entry, field.key, var, field.placeholder)
            row += 1

        if demo.toggles:
            toggles = ttk.Frame(self.fields_frame, style="Panel.TFrame")
            toggles.grid(row=row, column=0, columnspan=2, sticky="w", pady=(4, 0))
            for toggle in demo.toggles:
                tvar = tk.BooleanVar(value=toggle.default)
                self.toggle_vars[toggle.key] = tvar
                ttk.Checkbutton(
                    toggles,
                    text=toggle.label,
                    variable=tvar,
                    command=self._refresh_command_label,
                ).pack(side="left", padx=(0, 12))

    def _add_placeholder(self, entry: ttk.Entry, key: str, var: tk.StringVar, text: str) -> None:
        """Show greyed placeholder text while the field is empty and unfocused."""
        def show() -> None:
            if not var.get():
                self._placeholder_active[key] = True
                entry.configure(foreground="#9aa39b")
                var.set(text)

        def hide(_evt=None) -> None:
            if self._placeholder_active.get(key):
                self._placeholder_active[key] = False
                entry.configure(foreground="#1f2421")
                var.set("")

        def restore(_evt=None) -> None:
            if not var.get():
                show()

        entry.bind("<FocusIn>", hide)
        entry.bind("<FocusOut>", restore)
        show()

    def _option_args(self, demo: DemoSpec) -> tuple[str, ...]:
        args: list[str] = []
        # Positional/value fields first (positionals must precede flags cleanly).
        for field in demo.fields:
            if self._placeholder_active.get(field.key):
                continue  # showing placeholder text, treat as empty
            var = self.field_vars.get(field.key)
            if var is not None:
                args.extend(field.args_for(var.get()))
        for group in demo.options:
            var = self.option_vars.get(group.key)
            if var is not None:
                args.extend(group.args_for(var.get()))
        for toggle in demo.toggles:
            tvar = self.toggle_vars.get(toggle.key)
            if tvar is not None:
                args.extend(toggle.args_for(tvar.get()))
        return tuple(args)

    def _refresh_command_label(self) -> None:
        demo = self._demo_by_key(self.selected_demo.get())
        try:
            extra = self._split_extra_args()
        except ValueError as exc:
            self.command_label.configure(text=f"Invalid extra args: {exc}")
            return
        command = demo.command(self._option_args(demo) + extra)
        relative_command = []
        for index, part in enumerate(command):
            if index == 0:
                relative_command.append("python")
            elif part == "-u":
                relative_command.append(part)
            elif part.startswith(str(PROJECT_ROOT)):
                relative_command.append(str(Path(part).relative_to(PROJECT_ROOT)))
            else:
                relative_command.append(part)
        self.command_label.configure(text=" ".join(shlex.quote(part) for part in relative_command))

    def _run_selected_demo(self) -> None:
        if self.process is not None and self.process.poll() is None:
            messagebox.showinfo("Demo already running", "Stop the current demo before starting another one.")
            return

        demo = self._demo_by_key(self.selected_demo.get())
        try:
            extra = self._split_extra_args()
        except ValueError as exc:
            messagebox.showerror("Invalid extra args", str(exc))
            return
        option_args = self._option_args(demo)
        if demo.needs_args and not option_args and not extra:
            messagebox.showerror("Missing argument", f"{demo.title} needs a value. {demo.notes}")
            return

        command = demo.command(option_args + extra)
        self.live_frame_buffer = None
        self._append_output(f"\n$ {' '.join(shlex.quote(part) for part in command)}\n", tag="command")
        try:
            self.process = subprocess.Popen(
                command,
                cwd=PROJECT_ROOT,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
        except OSError as exc:
            self._append_output(f"Failed to start demo: {exc}\n", tag="error")
            self.status.set("Failed to start")
            return

        self.status.set(f"Running: {demo.title}")
        threading.Thread(target=self._read_process_output, daemon=True).start()

    def _read_process_output(self) -> None:
        if self.process is None or self.process.stdout is None:
            return
        for line in self.process.stdout:
            self.output_queue.put(line)

    def _drain_output_queue(self) -> None:
        while True:
            try:
                line = self.output_queue.get_nowait()
            except queue.Empty:
                break
            self._append_output(line)
        self.after(80, self._drain_output_queue)

    def _poll_process(self) -> None:
        if self.process is not None:
            returncode = self.process.poll()
            if returncode is not None:
                tag = "muted" if returncode == 0 else "error"
                self._append_output(f"\nProcess exited with code {returncode}\n", tag=tag)
                self.process = None
                self.status.set("Ready")
        self._refresh_command_label()
        self.after(300, self._poll_process)

    def _stop_process(self) -> None:
        if self.process is None or self.process.poll() is not None:
            self.status.set("Ready")
            return
        self.process.terminate()
        self.status.set("Stopping...")
        self.after(1200, self._kill_if_still_running)

    def _kill_if_still_running(self) -> None:
        if self.process is not None and self.process.poll() is None:
            self.process.kill()

    def _clear_output(self) -> None:
        self.live_frame_buffer = None
        self.output.delete("1.0", "end")

    def _append_output(self, text: str, tag: str | None = None) -> None:
        if tag is None and (ANSI_CLEAR_HOME in text or self.live_frame_buffer is not None):
            self._append_live_frame_output(text)
            return
        if tag is None and "\r" in text:
            self._append_carriage_return_output(text)
            return

        if tag is None:
            self.output.insert("end", text)
        else:
            self.output.insert("end", text, tag)
        self.output.see("end")

    def _append_live_frame_output(self, text: str) -> None:
        parts = text.split(ANSI_CLEAR_HOME)
        if len(parts) > 1:
            prefix = parts[0]
            if self.live_frame_buffer is None and prefix:
                self.output.insert("end", prefix)
            self.live_frame_buffer = parts[-1]
        elif self.live_frame_buffer is not None:
            self.live_frame_buffer += text
        else:
            self.output.insert("end", text)
            self.output.see("end")
            return

        if self.live_frame_buffer is not None and self._live_frame_complete(self.live_frame_buffer):
            self.output.delete("1.0", "end")
            self.output.insert("end", self.live_frame_buffer)
            self.output.see("end")

    @staticmethod
    def _live_frame_complete(text: str) -> bool:
        return "complete fresh frame:" in text and text.endswith("\n")

    def _append_carriage_return_output(self, text: str) -> None:
        chunks = text.split("\r")
        for index, chunk in enumerate(chunks):
            if index > 0:
                self.output.delete("insert linestart", "insert lineend")
            if chunk:
                self.output.insert("insert", chunk)
        self.output.see("end")

    def _on_close(self) -> None:
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
        self.destroy()

    def _split_extra_args(self) -> tuple[str, ...]:
        raw = self.extra_args.get().strip()
        if not raw:
            return ()
        return tuple(shlex.split(raw))

    @staticmethod
    def _demo_by_key(key: str) -> DemoSpec:
        for demo in DEMOS:
            if demo.key == key:
                return demo
        raise KeyError(key)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list", action="store_true", help="Print available demo keys and exit")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.list:
        for demo in DEMOS:
            default_args = " ".join(demo.default_args)
            suffix = f" [{default_args}]" if default_args else ""
            print(f"{demo.key}: {demo.script.relative_to(PROJECT_ROOT)}{suffix}")
            for group in demo.options:
                choices = "|".join(group.choices)
                print(f"    {group.flag} {{{choices}}} (default: {group.default})")
            for field in demo.fields:
                name = field.flag or f"<{field.key}>"
                hint = f" (e.g. {field.placeholder})" if field.placeholder else ""
                print(f"    {name} VALUE{hint}")
            for toggle in demo.toggles:
                print(f"    {toggle.flag} (checkbox)")
        return
    DemoLauncher().mainloop()


if __name__ == "__main__":
    main()
