"""Easy-to-use Tkinter + Matplotlib GUI for LinuxCNC PID & Feedforward Auto-Tuning."""

from __future__ import annotations

import csv
import json
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Dict, List, Optional
import numpy as np

import matplotlib

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

from .gcode_gen import generate_tuning_gcode
from .hal_backend import AXIS_TO_JOINT, LinuxCNCBackend
from .kpi_analyzer import analyze_trace
from .models import (
    KPIResult,
    MotionConfig,
    MotionPhase,
    PIDParams,
    TuningAttempt,
    TuningSuggestion,
)
from .optimizer import PIDAutoOptimizer


PARAM_FIELDS = [
    ("P", "Pgain (Stiffness)"),
    ("I", "Igain (Steady-State)"),
    ("D", "Dgain (Damping)"),
    ("FF0", "FF0 (Position FF)"),
    ("FF1", "FF1 (Velocity FF)"),
    ("FF2", "FF2 (Accel FF)"),
    ("deadband", "Deadband (mm)"),
    ("bias", "Bias (Offset)"),
    ("maxerrorI", "MaxErrorI (mm)"),
]


class PIDAutoTunerGUI:
    """Main application window for LinuxCNC PID + Feedforward Auto-Tuner."""

    def __init__(self, root: tk.Tk, force_sim: bool = False) -> None:
        self.root = root
        self.root.title(
            "LinuxCNC PID & Feedforward Auto-Tuner (Velocity Servo + Linear Scale)"
        )
        self.root.geometry("1460x900")
        self.root.minsize(1200, 760)

        self.backend = LinuxCNCBackend(force_sim=force_sim)
        self.optimizer = PIDAutoOptimizer()

        # Session state: history of all attempts across the session
        self.attempts: List[TuningAttempt] = []
        self.current_attempt: Optional[TuningAttempt] = None
        self.comparison_attempt: Optional[TuningAttempt] = None

        # Tk variables
        self.axis_var = tk.StringVar(value="X")
        self.sim_mode_var = tk.BooleanVar(value=not self.backend.is_live_linuxcnc)
        self.start_pos_var = tk.DoubleVar(value=0.0)
        self.travel_var = tk.DoubleVar(value=30.0)
        self.feed_var = tk.DoubleVar(value=1500.0)
        self.accel_var = tk.DoubleVar(value=250.0)
        self.dwell_var = tk.DoubleVar(value=0.35)
        self.status_var = tk.StringVar(
            value="Ready. Select an Axis, adjust Start Pos / Travel (mm) to stay within limits, and click '1. Run Test'."
        )

        self.current_param_vars: Dict[str, tk.StringVar] = {}
        self.suggested_param_vars: Dict[str, tk.StringVar] = {}
        self.delta_param_vars: Dict[str, tk.StringVar] = {}
        for key, _ in PARAM_FIELDS:
            self.current_param_vars[key] = tk.StringVar(value="0.0")
            self.suggested_param_vars[key] = tk.StringVar(value="—")
            self.delta_param_vars[key] = tk.StringVar(value="")

        self.kpi_vars: Dict[str, tk.StringVar] = {}

        self._configure_styles()
        self._build_layout()
        self._on_axis_changed()

    def _configure_styles(self) -> None:
        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except Exception:
            pass
        style.configure("TFrame", background="#f4f6f9")
        style.configure("TLabelframe", background="#f4f6f9")
        style.configure(
            "TLabelframe.Label",
            background="#f4f6f9",
            foreground="#1e293b",
            font=("Segoe UI", 10, "bold"),
        )
        style.configure("TLabel", background="#f4f6f9", foreground="#1e293b")
        style.configure(
            "Primary.TButton",
            font=("Segoe UI", 10, "bold"),
            padding=(10, 6),
        )
        style.configure(
            "Accent.TButton",
            font=("Segoe UI", 10, "bold"),
            padding=(10, 6),
        )
        style.configure("Treeview.Heading", font=("Segoe UI", 9, "bold"))
        style.configure("Treeview", rowheight=24, font=("Segoe UI", 9))

    def _build_layout(self) -> None:
        # 1. Top Control Toolbar
        top_frame = ttk.LabelFrame(
            self.root, text="Step 1: Axis, Motion Profile & Test Execution", padding=8
        )
        top_frame.pack(side=tk.TOP, fill=tk.X, padx=8, pady=(6, 4))

        # Axis selector
        ttk.Label(top_frame, text="Axis:", font=("Segoe UI", 10, "bold")).pack(
            side=tk.LEFT, padx=(4, 4)
        )
        axis_cb = ttk.Combobox(
            top_frame,
            textvariable=self.axis_var,
            values=["X", "Y", "Z", "A"],
            width=4,
            state="readonly",
            font=("Segoe UI", 10, "bold"),
        )
        axis_cb.pack(side=tk.LEFT, padx=(0, 8))
        axis_cb.bind("<<ComboboxSelected>>", lambda _e: self._on_axis_changed())

        # Start position + Read Current Position button
        ttk.Label(top_frame, text="Start (mm):").pack(side=tk.LEFT, padx=(2, 2))
        start_entry = ttk.Entry(top_frame, textvariable=self.start_pos_var, width=6)
        start_entry.pack(side=tk.LEFT, padx=(0, 2))
        start_entry.bind("<KeyRelease>", lambda _e: self._update_gcode_preview())

        ttk.Button(
            top_frame,
            text="📍 Read Pos",
            width=9,
            command=self.read_current_axis_pos,
        ).pack(side=tk.LEFT, padx=(0, 8))

        # Travel distance + Flip Direction button
        ttk.Label(top_frame, text="Travel (mm):").pack(side=tk.LEFT, padx=(2, 2))
        travel_entry = ttk.Entry(top_frame, textvariable=self.travel_var, width=6)
        travel_entry.pack(side=tk.LEFT, padx=(0, 2))
        travel_entry.bind("<KeyRelease>", lambda _e: self._update_gcode_preview())

        ttk.Button(
            top_frame,
            text="↔ Flip",
            width=6,
            command=self.flip_travel_direction,
        ).pack(side=tk.LEFT, padx=(0, 8))

        # Feed, Accel, Dwell
        for label_txt, var, width in [
            ("Feed (mm/min):", self.feed_var, 6),
            ("Accel (mm/s²):", self.accel_var, 5),
            ("Dwell (s):", self.dwell_var, 4),
        ]:
            ttk.Label(top_frame, text=label_txt).pack(side=tk.LEFT, padx=(2, 2))
            ent = ttk.Entry(top_frame, textvariable=var, width=width)
            ent.pack(side=tk.LEFT, padx=(0, 6))
            ent.bind("<KeyRelease>", lambda _e: self._update_gcode_preview())

        ttk.Separator(top_frame, orient=tk.VERTICAL).pack(
            side=tk.LEFT, fill=tk.Y, padx=8
        )

        # Primary Workflow Buttons
        btn_run = tk.Button(
            top_frame,
            text="▶ 1. Run Test (G-Code + Record)",
            bg="#2563eb",
            fg="white",
            activebackground="#1d4ed8",
            activeforeground="white",
            font=("Segoe UI", 10, "bold"),
            padx=12,
            pady=4,
            relief=tk.FLAT,
            command=self.run_single_test,
        )
        btn_run.pack(side=tk.LEFT, padx=4)

        btn_apply_run = tk.Button(
            top_frame,
            text="⚡ 2. Apply Suggested & Re-Test",
            bg="#059669",
            fg="white",
            activebackground="#047857",
            activeforeground="white",
            font=("Segoe UI", 10, "bold"),
            padx=12,
            pady=4,
            relief=tk.FLAT,
            command=self.apply_suggested_and_retest,
        )
        btn_apply_run.pack(side=tk.LEFT, padx=4)

        btn_auto = tk.Button(
            top_frame,
            text="🤖 Auto-Tune (5 Attempts)",
            bg="#7c3aed",
            fg="white",
            activebackground="#6d28d9",
            activeforeground="white",
            font=("Segoe UI", 10, "bold"),
            padx=10,
            pady=4,
            relief=tk.FLAT,
            command=lambda: self.run_auto_sequence(steps=5),
        )
        btn_auto.pack(side=tk.LEFT, padx=4)

        btn_stop = tk.Button(
            top_frame,
            text="🛑 STOP",
            bg="#dc2626",
            fg="white",
            activebackground="#b91c1c",
            activeforeground="white",
            font=("Segoe UI", 10, "bold"),
            padx=8,
            pady=4,
            relief=tk.FLAT,
            command=self.backend.abort_motion,
        )
        btn_stop.pack(side=tk.LEFT, padx=4)

        # Right side: Simulation toggle & INI file buttons
        sim_chk = ttk.Checkbutton(
            top_frame,
            text="Simulation Mode",
            variable=self.sim_mode_var,
            command=self._on_sim_mode_toggled,
        )
        sim_chk.pack(side=tk.RIGHT, padx=6)

        # 2. Main Split Area (Middle: Left Parameters/KPIs, Right Oscilloscope/Evolution Plots)
        main_paned = ttk.PanedWindow(self.root, orient=tk.VERTICAL)
        main_paned.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=8, pady=2)

        upper_pane = ttk.Frame(main_paned)
        lower_pane = ttk.LabelFrame(
            main_paned,
            text="Succession of Attempts — Evolution Log (Click any row to Overlay on Scope or Restore)",
            padding=6,
        )
        main_paned.add(upper_pane, weight=4)
        main_paned.add(lower_pane, weight=2)

        # Left column inside upper_pane: PID Parameters + KPIs + Rationale
        left_col = ttk.Frame(upper_pane, width=450)
        left_col.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 6))

        self._build_params_panel(left_col)
        self._build_kpi_summary_panel(left_col)
        self._build_rationale_panel(left_col)

        # Right column inside upper_pane: Notebook with Scope, Evolution Charts, G-Code
        right_col = ttk.Frame(upper_pane)
        right_col.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self._build_plots_notebook(right_col)

        # Bottom pane: Attempt History Table
        self._build_history_panel(lower_pane)

        # Status Bar
        status_bar = ttk.Frame(self.root, padding=(8, 3))
        status_bar.pack(side=tk.BOTTOM, fill=tk.X)
        ttk.Label(
            status_bar,
            textvariable=self.status_var,
            font=("Segoe UI", 9, "italic"),
        ).pack(side=tk.LEFT)

    def _build_params_panel(self, parent: ttk.Frame) -> None:
        p_frame = ttk.LabelFrame(
            parent,
            text="Step 2: PID & Feedforward Values (Current vs Suggested)",
            padding=8,
        )
        p_frame.pack(side=tk.TOP, fill=tk.X, pady=(0, 6))

        headers = ["Parameter", "Active Value", "Suggested", "Delta"]
        for col, h in enumerate(headers):
            ttk.Label(
                p_frame, text=h, font=("Segoe UI", 9, "bold")
            ).grid(row=0, column=col, sticky="w", padx=4, pady=(0, 4))

        for row_idx, (key, label) in enumerate(PARAM_FIELDS, start=1):
            ttk.Label(p_frame, text=label).grid(
                row=row_idx, column=0, sticky="w", padx=4, pady=1
            )
            entry = ttk.Entry(
                p_frame, textvariable=self.current_param_vars[key], width=11
            )
            entry.grid(row=row_idx, column=1, sticky="w", padx=4, pady=1)

            sug_lbl = ttk.Label(
                p_frame,
                textvariable=self.suggested_param_vars[key],
                font=("Consolas", 9, "bold"),
                foreground="#047857",
                width=11,
            )
            sug_lbl.grid(row=row_idx, column=2, sticky="w", padx=4, pady=1)

            delta_lbl = ttk.Label(
                p_frame,
                textvariable=self.delta_param_vars[key],
                font=("Consolas", 8),
                foreground="#475569",
                width=12,
            )
            delta_lbl.grid(row=row_idx, column=3, sticky="w", padx=4, pady=1)

        btn_row = ttk.Frame(p_frame)
        btn_row.grid(
            row=len(PARAM_FIELDS) + 1,
            column=0,
            columnspan=4,
            sticky="ew",
            pady=(8, 0),
        )
        ttk.Button(
            btn_row,
            text="✓ Copy Suggested -> Active",
            command=self.copy_suggested_to_active,
        ).pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(
            btn_row,
            text="Load .INI",
            command=self.load_from_ini_dialog,
        ).pack(side=tk.LEFT, padx=4)
        ttk.Button(
            btn_row,
            text="💾 Save Active to .INI",
            command=self.save_to_ini_dialog,
        ).pack(side=tk.LEFT, padx=4)

    def _build_kpi_summary_panel(self, parent: ttk.Frame) -> None:
        k_frame = ttk.LabelFrame(
            parent, text="Measured Oscilloscope KPIs (Latest Attempt)", padding=8
        )
        k_frame.pack(side=tk.TOP, fill=tk.X, pady=(0, 6))

        kpi_items = [
            ("ferror_rms", "RMS ferror:"),
            ("ferror_max", "Max Peak |ferror|:"),
            ("cruise_error_mean", "Cruise Lag (FF1):"),
            ("accel_peak_error", "Accel Spike (FF2):"),
            ("overshoot_mm", "Overshoot (D/P):"),
            ("settling_time_ms", "Settling Time:"),
            ("standstill_hunting_p2p", "Standstill Hunting:"),
            ("output_chatter_rms", "Drive Chatter RMS:"),
            ("cruise_asymmetry", "+/- Asymmetry (Bias):"),
            ("overall_score", "Overall Quality Score:"),
        ]

        for idx, (k_key, k_label) in enumerate(kpi_items):
            r = idx // 2
            c = (idx % 2) * 2
            self.kpi_vars[k_key] = tk.StringVar(value="—")
            ttk.Label(k_frame, text=k_label, font=("Segoe UI", 8)).grid(
                row=r, column=c, sticky="w", padx=(2, 2), pady=1
            )
            ttk.Label(
                k_frame,
                textvariable=self.kpi_vars[k_key],
                font=("Consolas", 9, "bold"),
                foreground="#0f172a",
            ).grid(row=r, column=c + 1, sticky="w", padx=(0, 10), pady=1)

    def _build_rationale_panel(self, parent: ttk.Frame) -> None:
        r_frame = ttk.LabelFrame(
            parent, text="Engineering Diagnosis & Tuning Recommendation", padding=6
        )
        r_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.rationale_text = tk.Text(
            r_frame,
            height=6,
            width=46,
            wrap=tk.WORD,
            font=("Segoe UI", 9),
            bg="#ffffff",
            fg="#1e293b",
            relief=tk.SOLID,
            bd=1,
        )
        self.rationale_text.pack(fill=tk.BOTH, expand=True)
        self.rationale_text.insert(
            tk.END,
            "Click '▶ 1. Run Test (G-Code + Record)' to execute the reference move, "
            "capture HAL oscilloscope signals, and compute optimal PID + FF parameters.",
        )
        self.rationale_text.config(state=tk.DISABLED)

    def _build_plots_notebook(self, parent: ttk.Frame) -> None:
        self.notebook = ttk.Notebook(parent)
        self.notebook.pack(fill=tk.BOTH, expand=True)

        # Tab 1: Integrated LinuxCNC Oscilloscope (Halscope view)
        scope_tab = ttk.Frame(self.notebook)
        self.notebook.add(scope_tab, text="📈 Integrated Oscilloscope (Halscope View)")

        self.scope_fig = Figure(figsize=(8.5, 5.0), dpi=100, facecolor="#ffffff")
        self.ax_ferror = self.scope_fig.add_subplot(211)
        self.ax_vel = self.scope_fig.add_subplot(212, sharex=self.ax_ferror)
        self.scope_fig.subplots_adjust(
            left=0.09, right=0.92, top=0.92, bottom=0.10, hspace=0.22
        )

        self.scope_canvas = FigureCanvasTkAgg(self.scope_fig, master=scope_tab)
        self.scope_canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        scope_toolbar = NavigationToolbar2Tk(self.scope_canvas, scope_tab)
        scope_toolbar.update()

        # Tab 2: Evolution Across Attempts (Succession of Attempts)
        evol_tab = ttk.Frame(self.notebook)
        self.notebook.add(
            evol_tab, text="📊 Evolution Across Attempts (KPIs & Parameters)"
        )

        self.evol_fig = Figure(figsize=(8.5, 5.0), dpi=100, facecolor="#ffffff")
        self.ax_evol_kpi = self.evol_fig.add_subplot(211)
        self.ax_evol_param = self.evol_fig.add_subplot(212, sharex=self.ax_evol_kpi)
        self.evol_fig.subplots_adjust(
            left=0.09, right=0.95, top=0.92, bottom=0.10, hspace=0.28
        )

        self.evol_canvas = FigureCanvasTkAgg(self.evol_fig, master=evol_tab)
        self.evol_canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # Tab 3: Generated G-Code Preview
        gcode_tab = ttk.Frame(self.notebook, padding=8)
        self.notebook.add(gcode_tab, text="📝 Test G-Code Program")
        self.gcode_text = tk.Text(
            gcode_tab, font=("Consolas", 10), bg="#0f172a", fg="#e2e8f0"
        )
        self.gcode_text.pack(fill=tk.BOTH, expand=True)

        self._draw_empty_plots()

    def _build_history_panel(self, parent: ttk.LabelFrame) -> None:
        columns = (
            "attempt",
            "time",
            "axis",
            "stage",
            "P",
            "I",
            "D",
            "FF1",
            "FF2",
            "deadband",
            "bias",
            "rms_um",
            "max_um",
            "cruise_um",
            "accel_um",
            "overshoot",
            "settling_ms",
            "chatter",
            "score",
        )
        tree_container = ttk.Frame(parent)
        tree_container.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.history_tree = ttk.Treeview(
            tree_container, columns=columns, show="headings", selectmode="browse"
        )
        col_specs = [
            ("attempt", "#", 40),
            ("time", "Time", 65),
            ("axis", "Axis", 42),
            ("stage", "Tuning Stage", 210),
            ("P", "P", 62),
            ("I", "I", 55),
            ("D", "D", 62),
            ("FF1", "FF1", 72),
            ("FF2", "FF2", 72),
            ("deadband", "Deadband", 70),
            ("bias", "Bias", 60),
            ("rms_um", "RMS (µm)", 72),
            ("max_um", "Peak (µm)", 74),
            ("cruise_um", "Cruise (µm)", 80),
            ("accel_um", "Accel (µm)", 76),
            ("overshoot", "Overshoot", 74),
            ("settling_ms", "Settle (ms)", 76),
            ("chatter", "Chatter", 64),
            ("score", "Score", 66),
        ]
        for col_id, heading, width in col_specs:
            self.history_tree.heading(col_id, text=heading)
            anchor = "w" if col_id == "stage" else "center"
            self.history_tree.column(col_id, width=width, anchor=anchor)

        y_scroll = ttk.Scrollbar(
            tree_container, orient=tk.VERTICAL, command=self.history_tree.yview
        )
        self.history_tree.configure(yscrollcommand=y_scroll.set)
        self.history_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        y_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        self.history_tree.bind("<<TreeviewSelect>>", self._on_history_row_selected)

        # History action bar
        hist_btns = ttk.Frame(parent)
        hist_btns.pack(side=tk.BOTTOM, fill=tk.X, pady=(4, 0))

        ttk.Button(
            hist_btns,
            text="🔍 Overlay Selected Attempt on Oscilloscope",
            command=self.overlay_selected_attempt,
        ).pack(side=tk.LEFT, padx=(0, 6))

        ttk.Button(
            hist_btns,
            text="⏪ Restore Selected Attempt Parameters",
            command=self.restore_selected_attempt_params,
        ).pack(side=tk.LEFT, padx=6)

        ttk.Button(
            hist_btns,
            text="🏆 Load Best Attempt So Far",
            command=self.load_best_attempt_params,
        ).pack(side=tk.LEFT, padx=6)

        ttk.Button(
            hist_btns,
            text="🗑 Clear History",
            command=self.clear_history,
        ).pack(side=tk.RIGHT, padx=4)

        ttk.Button(
            hist_btns,
            text="📤 Export History (CSV / JSON)",
            command=self.export_history_dialog,
        ).pack(side=tk.RIGHT, padx=6)

    # ------------------------------------------------------------------
    # State Helpers
    # ------------------------------------------------------------------
    def _on_sim_mode_toggled(self) -> None:
        self.backend.force_sim = bool(self.sim_mode_var.get())
        mode_str = (
            "Simulation Mode (Velocity Servo + Linear Scale Physics Model)"
            if self.sim_mode_var.get()
            else "Live LinuxCNC HAL Mode"
        )
        self.status_var.set(f"Switched to {mode_str}.")

    def _on_axis_changed(self) -> None:
        axis = self.axis_var.get().upper()
        params = self.backend.read_pid_params(axis)
        self._set_active_params_ui(params)
        if self.backend.is_live_linuxcnc:
            cur_pos = self.backend.read_current_position(axis)
            self.start_pos_var.set(cur_pos)
        for key, _ in PARAM_FIELDS:
            self.suggested_param_vars[key].set("—")
            self.delta_param_vars[key].set("")
        self._update_gcode_preview()
        min_lim, max_lim = self.backend.read_axis_limits(axis)
        self.status_var.set(
            f"Selected Axis {axis} (Limits: [{min_lim:.1f}, {max_lim:.1f} mm]). "
            "Adjust Start/Travel (mm) and click '▶ 1. Run Test'."
        )

    def read_current_axis_pos(self) -> None:
        """Read current live axis coordinate from LinuxCNC into Start (mm)."""
        axis = self.axis_var.get().upper()
        pos = self.backend.read_current_position(axis)
        self.start_pos_var.set(pos)
        self._update_gcode_preview()
        self.status_var.set(f"Set Start (mm) to current Axis {axis} position: {pos:.3f} mm.")

    def flip_travel_direction(self) -> None:
        """Invert the sign of Travel (mm) so the move goes in the opposite direction."""
        try:
            cur_travel = float(self.travel_var.get())
            self.travel_var.set(-cur_travel)
            self._update_gcode_preview()
            start = float(self.start_pos_var.get())
            self.status_var.set(
                f"Flipped travel direction: {start:.1f} mm -> {start - cur_travel:.1f} mm -> {start:.1f} mm."
            )
        except ValueError:
            pass

    def _get_motion_config(self) -> MotionConfig:
        axis = self.axis_var.get().upper()
        try:
            start_p = float(self.start_pos_var.get())
        except ValueError:
            start_p = 0.0
        try:
            travel_d = float(self.travel_var.get())
        except ValueError:
            travel_d = 20.0
        return MotionConfig(
            axis_letter=axis,
            joint_index=AXIS_TO_JOINT.get(axis, 0),
            start_pos=start_p,
            travel_distance=travel_d,
            cruise_feedrate=float(self.feed_var.get()),
            max_acceleration=float(self.accel_var.get()),
            dwell_time=float(self.dwell_var.get()),
        )

    def _get_active_params_from_ui(self) -> PIDParams:
        vals: Dict[str, float] = {}
        for key, _ in PARAM_FIELDS:
            try:
                vals[key] = float(self.current_param_vars[key].get())
            except ValueError:
                vals[key] = 0.0
        return PIDParams.from_dict(vals)

    def _set_active_params_ui(self, params: PIDParams) -> None:
        p_dict = params.to_dict()
        for key, _ in PARAM_FIELDS:
            self.current_param_vars[key].set(f"{p_dict[key]:.6g}")

    def _set_suggested_params_ui(
        self, active: PIDParams, suggested: PIDParams
    ) -> None:
        a_dict = active.to_dict()
        s_dict = suggested.to_dict()
        for key, _ in PARAM_FIELDS:
            val_s = s_dict[key]
            val_a = a_dict[key]
            self.suggested_param_vars[key].set(f"{val_s:.6g}")
            diff = val_s - val_a
            if abs(diff) > 1e-7:
                self.delta_param_vars[key].set(f"({diff:+.4g})")
            else:
                self.delta_param_vars[key].set("(unchanged)")

    def _update_gcode_preview(self) -> None:
        cfg = self._get_motion_config()
        gcode = generate_tuning_gcode(cfg)
        self.gcode_text.config(state=tk.NORMAL)
        self.gcode_text.delete("1.0", tk.END)
        self.gcode_text.insert(tk.END, gcode)
        self.gcode_text.config(state=tk.DISABLED)

    # ------------------------------------------------------------------
    # Core Actions: Run Test, Analyze, Suggest, Record Attempt
    # ------------------------------------------------------------------
    def run_single_test(self) -> Optional[TuningAttempt]:
        """Execute G-code, record oscilloscope trace, analyze KPIs, and suggest next PID/FF values."""
        cfg = self._get_motion_config()

        # Verify soft limits before moving the machine
        is_safe, safe_travel, limit_msg = self.backend.validate_travel_within_limits(
            cfg.axis_letter, cfg.start_pos, cfg.travel_distance
        )
        if not is_safe:
            if safe_travel != 0.0 and messagebox.askyesno(
                "Soft Limit Protection",
                f"{limit_msg}\n\nAutomatically adjust Travel to {safe_travel:+.1f} mm and continue?",
            ):
                self.travel_var.set(safe_travel)
                cfg = self._get_motion_config()
            else:
                self.status_var.set(f"Aborted: {limit_msg}")
                return None

        active_params = self._get_active_params_from_ui()
        self._update_gcode_preview()
        gcode_str = generate_tuning_gcode(cfg)

        self.status_var.set(
            f"Running G-code test on Axis {cfg.axis_letter} and recording HAL signals..."
        )
        self.root.update_idletasks()

        try:
            trace = self.backend.run_test_and_record(cfg, active_params)
        except Exception as exc:
            messagebox.showerror("LinuxCNC Execution Error", str(exc))
            self.status_var.set(f"Error during test: {exc}")
            return None

        kpis = analyze_trace(trace, active_params)
        axis_history = [a for a in self.attempts if a.axis == cfg.axis_letter]
        suggestion = self.optimizer.suggest_next(
            active_params, kpis, cfg, history=axis_history
        )

        attempt_num = len(self.attempts) + 1
        attempt = TuningAttempt.create(
            attempt_id=attempt_num,
            axis=cfg.axis_letter,
            stage_name=suggestion.stage_name,
            params=active_params,
            kpis=kpis,
            suggestion=suggestion,
            trace=trace,
            gcode_used=gcode_str,
        )

        # Automatically set previous attempt on the same axis as overlay comparison if none selected
        if axis_history and self.comparison_attempt is None:
            self.comparison_attempt = axis_history[-1]
        elif len(axis_history) >= 1:
            self.comparison_attempt = axis_history[-1]

        self.attempts.append(attempt)
        self.current_attempt = attempt

        # Update UI panels
        self._set_suggested_params_ui(active_params, suggestion.suggested_params)
        self._update_kpi_panel(kpis)
        self._update_rationale_panel(attempt)
        self._insert_attempt_row(attempt)
        self._update_scope_plot()
        self._update_evolution_plot()

        self.status_var.set(
            f"Attempt #{attempt_num} complete — RMS ferror: {kpis.ferror_rms*1000:.1f} µm | "
            f"Peak: {kpis.ferror_max*1000:.1f} µm | Score: {kpis.overall_score:.1f}. "
            "Click '⚡ 2. Apply Suggested & Re-Test' to continue."
        )
        return attempt

    def copy_suggested_to_active(self) -> bool:
        """Copy the suggested PID/FF values into the active input fields and apply to HAL."""
        if not self.current_attempt:
            messagebox.showinfo(
                "No Suggestion Yet",
                "Please click '▶ 1. Run Test (G-Code + Record)' first to generate suggested values.",
            )
            return False

        suggested = self.current_attempt.suggestion.suggested_params
        self._set_active_params_ui(suggested)
        axis = self.axis_var.get().upper()
        _, msg = self.backend.write_pid_params(axis, suggested)
        self.status_var.set(f"Applied suggested parameters to Active ({msg}).")
        return True

    def apply_suggested_and_retest(self) -> None:
        """One-click workflow: apply suggested values and immediately run the next attempt."""
        if not self.current_attempt:
            self.run_single_test()
            return
        if self.copy_suggested_to_active():
            self.run_single_test()

    def run_auto_sequence(self, steps: int = 5) -> None:
        """Automatically run a succession of N test-and-update attempts to see full convergence."""
        for step_idx in range(steps):
            if step_idx > 0 or self.current_attempt is not None:
                self.copy_suggested_to_active()
            attempt = self.run_single_test()
            self.root.update()
            if attempt is None:
                break

    # ------------------------------------------------------------------
    # UI Panel Updaters
    # ------------------------------------------------------------------
    def _update_kpi_panel(self, kpis: KPIResult) -> None:
        self.kpi_vars["ferror_rms"].set(f"{kpis.ferror_rms*1000:.1f} µm")
        self.kpi_vars["ferror_max"].set(f"{kpis.ferror_max*1000:.1f} µm")
        self.kpi_vars["cruise_error_mean"].set(
            f"{kpis.cruise_error_mean*1000:+.1f} µm"
        )
        self.kpi_vars["accel_peak_error"].set(
            f"{kpis.accel_peak_error*1000:+.1f} µm"
        )
        self.kpi_vars["overshoot_mm"].set(
            f"{kpis.overshoot_mm*1000:.1f} µm ({kpis.overshoot_pct:.0f}%)"
        )
        self.kpi_vars["settling_time_ms"].set(f"{kpis.settling_time_ms:.0f} ms")
        self.kpi_vars["standstill_hunting_p2p"].set(
            f"{kpis.standstill_hunting_p2p*1000:.1f} µm"
        )
        self.kpi_vars["output_chatter_rms"].set(f"{kpis.output_chatter_rms:.3f}")
        self.kpi_vars["cruise_asymmetry"].set(
            f"{kpis.cruise_asymmetry*1000:+.1f} µm"
        )
        self.kpi_vars["overall_score"].set(f"{kpis.overall_score:.1f}")

    def _update_rationale_panel(self, attempt: TuningAttempt) -> None:
        sug = attempt.suggestion
        self.rationale_text.config(state=tk.NORMAL)
        self.rationale_text.delete("1.0", tk.END)

        lines = [f"Attempt #{attempt.attempt_id} — {sug.stage_name}\n"]
        if sug.warnings:
            for w in sug.warnings:
                lines.append(f"⚠ WARNING: {w}\n")
        for item in sug.rationale:
            lines.append(f"• {item}\n")

        if self.comparison_attempt and self.comparison_attempt.attempt_id != attempt.attempt_id:
            prev = self.comparison_attempt
            rms_prev = prev.kpis.ferror_rms * 1000.0
            rms_now = attempt.kpis.ferror_rms * 1000.0
            if rms_prev > 1e-6:
                impr = ((rms_prev - rms_now) / rms_prev) * 100.0
                lines.append(
                    f"\nComparison vs Attempt #{prev.attempt_id}: "
                    f"RMS ferror changed from {rms_prev:.1f} µm -> {rms_now:.1f} µm ({impr:+.1f}% improvement)."
                )

        self.rationale_text.insert(tk.END, "\n".join(lines))
        self.rationale_text.config(state=tk.DISABLED)

    def _insert_attempt_row(self, attempt: TuningAttempt) -> None:
        p = attempt.params
        k = attempt.kpis
        vals = (
            f"#{attempt.attempt_id}",
            attempt.timestamp,
            attempt.axis,
            attempt.stage_name,
            f"{p.P:.4g}",
            f"{p.I:.4g}",
            f"{p.D:.4g}",
            f"{p.FF1:.5g}",
            f"{p.FF2:.5g}",
            f"{p.deadband:.4g}",
            f"{p.bias:.4g}",
            f"{k.ferror_rms*1000:.1f}",
            f"{k.ferror_max*1000:.1f}",
            f"{k.cruise_error_mean*1000:+.1f}",
            f"{k.accel_peak_error*1000:+.1f}",
            f"{k.overshoot_pct:.0f}%",
            f"{k.settling_time_ms:.0f}",
            f"{k.output_chatter_rms:.3f}",
            f"{k.overall_score:.1f}",
        )
        item_id = self.history_tree.insert("", tk.END, iid=str(attempt.attempt_id), values=vals)
        self.history_tree.see(item_id)

    # ------------------------------------------------------------------
    # Matplotlib Oscilloscope & Evolution Plotting
    # ------------------------------------------------------------------
    def _draw_empty_plots(self) -> None:
        self.ax_ferror.clear()
        self.ax_vel.clear()
        self.ax_ferror.set_title(
            "LinuxCNC Integrated Oscilloscope — Following Error (joint.N.f-error)"
        )
        self.ax_ferror.set_ylabel("Following Error (mm)")
        self.ax_ferror.grid(True, linestyle="--", alpha=0.5)

        self.ax_vel.set_title("Commanded Velocity (mm/s) & PID Drive Output")
        self.ax_vel.set_xlabel("Time (s)")
        self.ax_vel.set_ylabel("Velocity (mm/s)")
        self.ax_vel.grid(True, linestyle="--", alpha=0.5)
        self.scope_canvas.draw_idle()

        self.ax_evol_kpi.clear()
        self.ax_evol_param.clear()
        self.ax_evol_kpi.set_title(
            "KPI Evolution Across Attempts (Lower Error = Better)"
        )
        self.ax_evol_kpi.set_ylabel("Error (µm)")
        self.ax_evol_kpi.grid(True, linestyle="--", alpha=0.5)

        self.ax_evol_param.set_title("Parameter Evolution Across Attempts")
        self.ax_evol_param.set_xlabel("Attempt #")
        self.ax_evol_param.set_ylabel("Gain Value")
        self.ax_evol_param.grid(True, linestyle="--", alpha=0.5)
        self.evol_canvas.draw_idle()

    def _update_scope_plot(self) -> None:
        if not self.current_attempt:
            return

        att = self.current_attempt
        tr = att.trace

        self.ax_ferror.clear()
        self.ax_vel.clear()

        # Shade motion phases on top plot for clear visual diagnosis
        phase_colors = {
            MotionPhase.ACCEL_POS: ("#fef08a", "Accel/Decel"),
            MotionPhase.DECEL_POS: ("#fef08a", "_nolegend_"),
            MotionPhase.ACCEL_NEG: ("#fef08a", "_nolegend_"),
            MotionPhase.DECEL_NEG: ("#fef08a", "_nolegend_"),
            MotionPhase.CRUISE_POS: ("#dcfce7", "Constant Cruise (FF1)"),
            MotionPhase.CRUISE_NEG: ("#dcfce7", "_nolegend_"),
            MotionPhase.SETTLING_POS: ("#e0e7ff", "Settling (P/D)"),
            MotionPhase.SETTLING_NEG: ("#e0e7ff", "_nolegend_"),
        }
        used_labels = set()
        if len(tr.phase) == len(tr.t) and len(tr.t) > 1:
            changes = np.where(np.diff(tr.phase) != 0)[0] + 1
            bounds = np.concatenate(([0], changes, [len(tr.t) - 1]))
            for i in range(len(bounds) - 1):
                s_idx, e_idx = int(bounds[i]), int(bounds[i + 1])
                ph = int(tr.phase[s_idx])
                if ph in phase_colors:
                    color, lbl = phase_colors[ph]
                    if lbl in used_labels:
                        lbl = "_nolegend_"
                    else:
                        used_labels.add(lbl)
                    self.ax_ferror.axvspan(
                        tr.t[s_idx], tr.t[e_idx], color=color, alpha=0.35, label=lbl
                    )

        # Plot comparison attempt overlay if available
        if (
            self.comparison_attempt is not None
            and self.comparison_attempt.attempt_id != att.attempt_id
        ):
            comp = self.comparison_attempt
            self.ax_ferror.plot(
                comp.trace.t,
                comp.trace.ferror * 1000.0,
                color="#94a3b8",
                linestyle="--",
                linewidth=1.3,
                label=f"Attempt #{comp.attempt_id} ferror (RMS={comp.kpis.ferror_rms*1000:.1f} µm)",
            )

        # Plot current attempt ferror in micrometers (µm)
        self.ax_ferror.plot(
            tr.t,
            tr.ferror * 1000.0,
            color="#dc2626",
            linewidth=1.6,
            label=f"Attempt #{att.attempt_id} ferror (RMS={att.kpis.ferror_rms*1000:.1f} µm, Peak={att.kpis.ferror_max*1000:.1f} µm)",
        )
        self.ax_ferror.axhline(0.0, color="#0f172a", linewidth=0.8, alpha=0.6)
        self.ax_ferror.set_title(
            f"Axis {att.axis} — Following Error `joint.{AXIS_TO_JOINT.get(att.axis, 0)}.f-error` (µm)"
        )
        self.ax_ferror.set_ylabel("Following Error (µm)")
        self.ax_ferror.legend(loc="upper right", fontsize=8)
        self.ax_ferror.grid(True, linestyle="--", alpha=0.4)

        # Bottom subplot: Commanded & Actual Velocity + PID Output
        self.ax_vel.plot(
            tr.t,
            tr.vel_cmd,
            color="#2563eb",
            linewidth=1.5,
            label="vel-cmd (mm/s)",
        )
        self.ax_vel.plot(
            tr.t,
            tr.pid_output * 5.0,
            color="#059669",
            linewidth=1.1,
            alpha=0.85,
            label="pid.output (x5 scaled)",
        )
        self.ax_vel.set_title("Commanded Velocity & PID Drive Command (`pid.N.output`)")
        self.ax_vel.set_xlabel("Time (s)")
        self.ax_vel.set_ylabel("mm/s | Output")
        self.ax_vel.legend(loc="upper right", fontsize=8)
        self.ax_vel.grid(True, linestyle="--", alpha=0.4)

        self.scope_canvas.draw_idle()

    def _update_evolution_plot(self) -> None:
        axis = self.axis_var.get().upper()
        axis_attempts = [a for a in self.attempts if a.axis == axis]
        if not axis_attempts:
            return

        ids = [a.attempt_id for a in axis_attempts]
        rms_vals = [a.kpis.ferror_rms * 1000.0 for a in axis_attempts]
        max_vals = [a.kpis.ferror_max * 1000.0 for a in axis_attempts]
        cruise_vals = [abs(a.kpis.cruise_error_mean) * 1000.0 for a in axis_attempts]
        accel_vals = [abs(a.kpis.accel_peak_error) * 1000.0 for a in axis_attempts]

        p_vals = [a.params.P for a in axis_attempts]
        d_vals = [a.params.D * 100.0 for a in axis_attempts]
        ff1_vals = [a.params.FF1 * 100.0 for a in axis_attempts]
        ff2_vals = [a.params.FF2 * 1000.0 for a in axis_attempts]

        self.ax_evol_kpi.clear()
        self.ax_evol_param.clear()

        self.ax_evol_kpi.plot(
            ids, max_vals, "o-", color="#dc2626", linewidth=1.8, label="Peak |ferror| (µm)"
        )
        self.ax_evol_kpi.plot(
            ids, rms_vals, "s-", color="#2563eb", linewidth=1.8, label="RMS ferror (µm)"
        )
        self.ax_evol_kpi.plot(
            ids, cruise_vals, "^--", color="#059669", linewidth=1.4, label="|Cruise Lag| (µm)"
        )
        self.ax_evol_kpi.plot(
            ids, accel_vals, "d--", color="#d97706", linewidth=1.4, label="|Accel Spike| (µm)"
        )
        self.ax_evol_kpi.set_title(
            f"Axis {axis} — Evolution of Following Error KPIs Across Attempts"
        )
        self.ax_evol_kpi.set_ylabel("Error (µm)")
        self.ax_evol_kpi.set_xticks(ids)
        self.ax_evol_kpi.legend(loc="upper right", fontsize=8)
        self.ax_evol_kpi.grid(True, linestyle="--", alpha=0.5)

        self.ax_evol_param.plot(
            ids, p_vals, "o-", color="#7c3aed", linewidth=1.8, label="Pgain"
        )
        self.ax_evol_param.plot(
            ids, ff1_vals, "s-", color="#059669", linewidth=1.6, label="FF1 (x100)"
        )
        self.ax_evol_param.plot(
            ids, ff2_vals, "^-", color="#d97706", linewidth=1.6, label="FF2 (x1000)"
        )
        self.ax_evol_param.plot(
            ids, d_vals, "d--", color="#0284c7", linewidth=1.4, label="Dgain (x100)"
        )
        self.ax_evol_param.set_title(
            f"Axis {axis} — Evolution of PID & Feedforward Parameters Across Attempts"
        )
        self.ax_evol_param.set_xlabel("Attempt #")
        self.ax_evol_param.set_ylabel("Scaled Parameter Value")
        self.ax_evol_param.set_xticks(ids)
        self.ax_evol_param.legend(loc="upper left", fontsize=8)
        self.ax_evol_param.grid(True, linestyle="--", alpha=0.5)

        self.evol_canvas.draw_idle()

    # ------------------------------------------------------------------
    # History Selection, Comparison Overlay, Restore, & Export
    # ------------------------------------------------------------------
    def _get_selected_history_attempt(self) -> Optional[TuningAttempt]:
        sel = self.history_tree.selection()
        if not sel:
            return None
        att_id = int(sel[0])
        for a in self.attempts:
            if a.attempt_id == att_id:
                return a
        return None

    def _on_history_row_selected(self, _event: object) -> None:
        att = self._get_selected_history_attempt()
        if att is None:
            return
        self.comparison_attempt = att
        self._update_scope_plot()
        if self.current_attempt:
            self._update_rationale_panel(self.current_attempt)
        self.status_var.set(
            f"Overlaying Attempt #{att.attempt_id} (dashed gray line) against latest attempt on Oscilloscope."
        )

    def overlay_selected_attempt(self) -> None:
        self._on_history_row_selected(None)

    def restore_selected_attempt_params(self) -> None:
        att = self._get_selected_history_attempt()
        if att is None:
            messagebox.showinfo(
                "Select an Attempt",
                "Click a row in the Attempt History table first to restore its parameters.",
            )
            return
        self.axis_var.set(att.axis)
        self._set_active_params_ui(att.params)
        self.backend.write_pid_params(att.axis, att.params)
        self.status_var.set(
            f"Restored active parameters from Attempt #{att.attempt_id} (RMS={att.kpis.ferror_rms*1000:.1f} µm)."
        )

    def load_best_attempt_params(self) -> None:
        axis = self.axis_var.get().upper()
        axis_attempts = [a for a in self.attempts if a.axis == axis]
        if not axis_attempts:
            messagebox.showinfo("No Attempts", f"No attempts recorded for Axis {axis} yet.")
            return
        best = min(axis_attempts, key=lambda a: a.kpis.overall_score)
        self._set_active_params_ui(best.params)
        self.backend.write_pid_params(axis, best.params)
        self.history_tree.selection_set(str(best.attempt_id))
        self.status_var.set(
            f"Loaded best attempt for Axis {axis}: Attempt #{best.attempt_id} "
            f"(Score={best.kpis.overall_score:.1f}, RMS={best.kpis.ferror_rms*1000:.1f} µm)."
        )

    def clear_history(self) -> None:
        if not self.attempts:
            return
        if messagebox.askyesno("Clear History", "Clear all recorded tuning attempts?"):
            self.attempts.clear()
            self.current_attempt = None
            self.comparison_attempt = None
            for item in self.history_tree.get_children():
                self.history_tree.delete(item)
            self._draw_empty_plots()
            self.status_var.set("Cleared session attempt history.")

    def export_history_dialog(self) -> None:
        if not self.attempts:
            messagebox.showinfo("No Data", "Run at least one test attempt before exporting.")
            return

        file_path = filedialog.asksaveasfilename(
            title="Export Tuning Attempts History",
            defaultextension=".csv",
            filetypes=[("CSV Table", "*.csv"), ("JSON Session", "*.json")],
        )
        if not file_path:
            return

        path = Path(file_path)
        if path.suffix.lower() == ".json":
            payload = [
                {
                    "attempt_id": a.attempt_id,
                    "timestamp": a.timestamp,
                    "axis": a.axis,
                    "stage_name": a.stage_name,
                    "params": a.params.to_dict(),
                    "kpis": a.kpis.to_dict(),
                    "suggested_params": a.suggestion.suggested_params.to_dict(),
                    "rationale": a.suggestion.rationale,
                }
                for a in self.attempts
            ]
            path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        else:
            with path.open("w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(
                    [
                        "attempt_id",
                        "timestamp",
                        "axis",
                        "stage",
                        "P",
                        "I",
                        "D",
                        "FF0",
                        "FF1",
                        "FF2",
                        "deadband",
                        "bias",
                        "maxerrorI",
                        "ferror_rms_mm",
                        "ferror_max_mm",
                        "cruise_error_mean_mm",
                        "accel_peak_error_mm",
                        "overshoot_pct",
                        "settling_time_ms",
                        "output_chatter_rms",
                        "overall_score",
                    ]
                )
                for a in self.attempts:
                    writer.writerow(
                        [
                            a.attempt_id,
                            a.timestamp,
                            a.axis,
                            a.stage_name,
                            a.params.P,
                            a.params.I,
                            a.params.D,
                            a.params.FF0,
                            a.params.FF1,
                            a.params.FF2,
                            a.params.deadband,
                            a.params.bias,
                            a.params.maxerrorI,
                            a.kpis.ferror_rms,
                            a.kpis.ferror_max,
                            a.kpis.cruise_error_mean,
                            a.kpis.accel_peak_error,
                            a.kpis.overshoot_pct,
                            a.kpis.settling_time_ms,
                            a.kpis.output_chatter_rms,
                            a.kpis.overall_score,
                        ]
                    )

        self.status_var.set(f"Exported {len(self.attempts)} attempts to {path.name}.")

    # ------------------------------------------------------------------
    # .INI Configuration Dialogs
    # ------------------------------------------------------------------
    def load_from_ini_dialog(self) -> None:
        initial = str(self.backend.ini_path.parent) if self.backend.ini_path else "."
        file_path = filedialog.askopenfilename(
            title="Select LinuxCNC .INI Configuration File",
            initialdir=initial,
            filetypes=[("LinuxCNC INI", "*.ini"), ("All Files", "*.*")],
        )
        if not file_path:
            return
        ini_path = Path(file_path)
        self.backend.ini_path = ini_path
        axis = self.axis_var.get().upper()
        params, msg = self.backend.load_params_from_ini(ini_path, axis)
        if params:
            self._set_active_params_ui(params)
            self.status_var.set(msg)
        else:
            messagebox.showwarning("INI Load Notice", msg)

    def save_to_ini_dialog(self) -> None:
        ini_path = self.backend.ini_path
        if ini_path is None or not ini_path.is_file():
            file_path = filedialog.asksaveasfilename(
                title="Save PID Parameters to LinuxCNC .INI File",
                defaultextension=".ini",
                filetypes=[("LinuxCNC INI", "*.ini"), ("All Files", "*.*")],
            )
            if not file_path:
                return
            ini_path = Path(file_path)
            if not ini_path.is_file():
                ini_path.write_text("# LinuxCNC Configuration\n", encoding="utf-8")
            self.backend.ini_path = ini_path

        axis = self.axis_var.get().upper()
        active_params = self._get_active_params_from_ui()
        ok, msg = self.backend.save_params_to_ini(ini_path, axis, active_params)
        if ok:
            messagebox.showinfo("Saved to .INI", msg)
            self.status_var.set(msg)
        else:
            messagebox.showerror("Save Error", msg)


def launch_gui(force_sim: bool = False) -> None:
    """Create the root Tk window and start the PID Auto-Tuner GUI."""
    root = tk.Tk()
    PIDAutoTunerGUI(root, force_sim=force_sim)
    root.mainloop()
