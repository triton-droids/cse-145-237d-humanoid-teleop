"""Clickable launcher for the demo workflows.

Usage (from project root):
  conda run --no-capture-output -n humanoid-sim python demos\demo_launcher.py
  python demos/demo_launcher.py
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
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
class DemoSpec:
    key: str
    title: str
    script: Path
    description: str
    default_args: tuple[str, ...] = ()
    notes: str = ""
    needs_args: bool = False
    options: tuple[OptionGroup, ...] = ()

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
        notes=(
            "Pick the IMU set on the right. Click Calibrate, then Record in the plot window. "
            "Use Extra args for --record-duration-s, --record-fps, or --record-output."
        ),
        options=(
            OptionGroup(
                key="config",
                label="IMU set",
                flag="--config",
                choices=("thighs", "shanks", "full"),
                default="thighs",
            ),
        ),
    ),
    DemoSpec(
        key="udp-quaternion-receiver",
        title="UDP Quaternion Receiver",
        script=PROJECT_ROOT / "demos" / "demo_udp_quaternion_receiver.py",
        description="Monitor live ESP32/BNO085 quaternion packet status over UDP.",
        notes="Requires ESP32/BNO085 nodes streaming UDP packets.",
    ),
    DemoSpec(
        key="udp-latency-ping",
        title="UDP Latency Ping",
        script=PROJECT_ROOT / "demos" / "demo_udp_latency_ping.py",
        description="Measure UDP round-trip latency to one ESP32 node.",
        notes="Enter the ESP32 IP address in Extra args before running, for example: 192.168.4.20",
        needs_args=True,
    ),
)


class DemoLauncher(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("Humanoid Teleop Demo Launcher")
        self.geometry("1040x720")
        self.minsize(900, 600)

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

        style.configure(".", font=("Avenir Next", 12), background="#f3f4f1", foreground="#1f2421")
        style.configure("Root.TFrame", background="#f3f4f1")
        style.configure("Panel.TFrame", background="#ffffff", relief="flat")
        style.configure("Sidebar.TFrame", background="#e8ebe4")
        style.configure("Header.TLabel", font=("Avenir Next", 24, "bold"), background="#f3f4f1", foreground="#1d2620")
        style.configure("Subtle.TLabel", background="#ffffff", foreground="#5f6860")
        style.configure("PanelTitle.TLabel", font=("Avenir Next", 18, "bold"), background="#ffffff", foreground="#1d2620")
        style.configure("Section.TLabel", font=("Avenir Next", 12, "bold"), background="#ffffff", foreground="#384139")
        style.configure("SidebarTitle.TLabel", font=("Avenir Next", 13, "bold"), background="#e8ebe4", foreground="#1d2620")
        style.configure("Status.TLabel", background="#dce8d9", foreground="#1f5b35", padding=(12, 5))
        style.configure("Command.TLabel", background="#eef1ec", foreground="#28322b", padding=(10, 8), font=("Menlo", 11))
        style.configure("TEntry", fieldbackground="#ffffff", bordercolor="#cbd2c8", lightcolor="#cbd2c8", darkcolor="#cbd2c8")
        style.configure("Primary.TButton", font=("Avenir Next", 12, "bold"), background="#23614a", foreground="#ffffff", padding=(14, 8))
        style.map("Primary.TButton", background=[("active", "#1d513d"), ("pressed", "#163d2f")])
        style.configure("Secondary.TButton", background="#e4e8e1", foreground="#243027", padding=(14, 8))
        style.map("Secondary.TButton", background=[("active", "#d8dfd4"), ("pressed", "#cbd5c7")])
        style.configure("Danger.TButton", background="#ead9d3", foreground="#733221", padding=(14, 8))
        style.map("Danger.TButton", background=[("active", "#dfc9c0"), ("pressed", "#d0b2a8")])
        # Segmented-control look for per-demo option choices (Toolbutton radios).
        style.configure("Option.Toolbutton", font=("Avenir Next", 11), padding=(14, 6), background="#e4e8e1", foreground="#243027")
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
            font=("Avenir Next", 12),
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
        self.description_label = ttk.Label(details, wraplength=650, justify="left", style="Subtle.TLabel")
        self.description_label.grid(row=1, column=0, sticky="ew", pady=(6, 0))
        self.notes_label = ttk.Label(details, wraplength=650, justify="left", style="Subtle.TLabel")
        self.notes_label.grid(row=2, column=0, sticky="ew", pady=(6, 0))

        # Per-demo option groups (radio buttons) are rebuilt on demo selection.
        self.options_frame = ttk.Frame(details, style="Panel.TFrame")
        self.options_frame.grid(row=3, column=0, sticky="ew", pady=(14, 0))
        self.option_vars: dict[str, tk.StringVar] = {}

        args_frame = ttk.Frame(details, style="Panel.TFrame")
        args_frame.grid(row=4, column=0, sticky="ew", pady=(14, 0))
        args_frame.columnconfigure(1, weight=1)
        ttk.Label(args_frame, text="Extra args", style="Section.TLabel").grid(row=0, column=0, sticky="w", padx=(0, 10))
        args_entry = ttk.Entry(args_frame, textvariable=self.extra_args)
        args_entry.grid(row=0, column=1, sticky="ew")
        args_entry.bind("<KeyRelease>", lambda _event: self._refresh_command_label())

        button_frame = ttk.Frame(details, style="Panel.TFrame")
        button_frame.grid(row=5, column=0, sticky="ew", pady=(14, 0))
        ttk.Button(button_frame, text="Run Demo", command=self._run_selected_demo, style="Primary.TButton").pack(side="left")
        ttk.Button(button_frame, text="Stop", command=self._stop_process, style="Danger.TButton").pack(side="left", padx=(8, 0))
        ttk.Button(button_frame, text="Clear Output", command=self._clear_output, style="Secondary.TButton").pack(side="left", padx=(8, 0))

        command_frame = ttk.Frame(controls, padding=(0, 12, 0, 0), style="Root.TFrame")
        command_frame.grid(row=1, column=0, sticky="ew")
        ttk.Label(command_frame, text="Command", style="Section.TLabel").pack(anchor="w", pady=(0, 6))
        self.command_label = ttk.Label(command_frame, wraplength=720, justify="left", style="Command.TLabel")
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
            font=("Menlo", 11),
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
        """Render the selected demo's option groups as segmented radio buttons."""
        for child in self.options_frame.winfo_children():
            child.destroy()
        self.option_vars = {}

        if not demo.options:
            self.options_frame.grid_remove()
            return
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

    def _option_args(self, demo: DemoSpec) -> tuple[str, ...]:
        args: list[str] = []
        for group in demo.options:
            var = self.option_vars.get(group.key)
            if var is not None:
                args.extend(group.args_for(var.get()))
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
        if demo.needs_args and not extra:
            messagebox.showerror("Missing argument", f"{demo.title} needs extra args. {demo.notes}")
            return

        command = demo.command(self._option_args(demo) + extra)
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
        return
    DemoLauncher().mainloop()


if __name__ == "__main__":
    main()
