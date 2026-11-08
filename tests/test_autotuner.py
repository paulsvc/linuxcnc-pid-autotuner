"""Automated verification tests for the LinuxCNC PID & Feedforward Auto-Tuner."""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parent.parent
SRC_DIR = ROOT_DIR / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from lcnc_pid_tuner.hal_backend import LinuxCNCBackend
from lcnc_pid_tuner.kpi_analyzer import analyze_trace
from lcnc_pid_tuner.models import MotionConfig, PIDParams, TuningAttempt
from lcnc_pid_tuner.optimizer import PIDAutoOptimizer


class TestPIDAutoTuner(unittest.TestCase):
    def test_multi_attempt_convergence_x_axis(self) -> None:
        """Verify that 5 successive attempts significantly reduce ferror RMS and Peak on Axis X."""
        backend = LinuxCNCBackend(force_sim=True)
        optimizer = PIDAutoOptimizer()
        cfg = MotionConfig(
            axis_letter="X",
            joint_index=0,
            travel_distance=50.0,
            cruise_feedrate=1500.0,
            max_acceleration=250.0,
            dwell_time=0.35,
        )

        params = backend.read_pid_params("X")
        history = []

        for i in range(1, 7):
            trace = backend.run_test_and_record(cfg, params)
            kpis = analyze_trace(trace, params)
            suggestion = optimizer.suggest_next(params, kpis, cfg, history=history)
            att = TuningAttempt.create(
                attempt_id=i,
                axis="X",
                stage_name=suggestion.stage_name,
                params=params,
                kpis=kpis,
                suggestion=suggestion,
                trace=trace,
            )
            history.append(att)
            params = suggestion.suggested_params.copy()

        first = history[0]
        best = min(history, key=lambda a: a.kpis.overall_score)

        # Untuned initial attempt (FF1=0, P=12) has ~200+ um cruise error
        self.assertGreater(first.kpis.ferror_rms, 0.08)
        # Tuned attempt must reduce RMS following error by >85%
        self.assertLess(best.kpis.ferror_rms, first.kpis.ferror_rms * 0.15)
        self.assertLess(abs(best.kpis.cruise_error_mean), 0.005)
        self.assertAlmostEqual(best.params.FF1, 0.10, delta=0.02)

    def test_z_axis_gravity_bias_compensation(self) -> None:
        """Verify that Z-axis gravity asymmetry is detected and compensated via bias + FF1."""
        backend = LinuxCNCBackend(force_sim=True)
        optimizer = PIDAutoOptimizer()
        cfg = MotionConfig(
            axis_letter="Z",
            joint_index=2,
            travel_distance=40.0,
            cruise_feedrate=1200.0,
            max_acceleration=200.0,
            dwell_time=0.35,
        )

        params = backend.read_pid_params("Z")
        history = []
        for i in range(1, 6):
            trace = backend.run_test_and_record(cfg, params)
            kpis = analyze_trace(trace, params)
            suggestion = optimizer.suggest_next(params, kpis, cfg, history=history)
            att = TuningAttempt.create(
                attempt_id=i,
                axis="Z",
                stage_name=suggestion.stage_name,
                params=params,
                kpis=kpis,
                suggestion=suggestion,
                trace=trace,
            )
            history.append(att)
            params = suggestion.suggested_params.copy()

        # Gravity on Z is -0.35 in the simulator, so bias should become positive (> +0.15)
        self.assertGreater(history[-1].params.bias, 0.15)
        self.assertLess(history[-1].kpis.ferror_rms, history[0].kpis.ferror_rms * 0.25)

    def test_ini_load_and_save_with_backup(self) -> None:
        """Verify reading and updating a LinuxCNC .ini file with backup creation."""
        backend = LinuxCNCBackend(force_sim=True)
        sample_ini = """[EMC]
MACHINE = TestMill

[JOINT_0]
TYPE = LINEAR
MAX_VELOCITY = 50.0
MAX_ACCELERATION = 250.0
P = 15.0
I = 0.0
D = 0.0
FF0 = 0.0
FF1 = 0.05
FF2 = 0.0
DEADBAND = 0.001
BIAS = 0.0
"""
        with tempfile.TemporaryDirectory() as tmpdir:
            ini_path = Path(tmpdir) / "mill.ini"
            ini_path.write_text(sample_ini, encoding="utf-8")

            loaded, msg = backend.load_params_from_ini(ini_path, "X")
            self.assertIsNotNone(loaded)
            assert loaded is not None
            self.assertAlmostEqual(loaded.P, 15.0)
            self.assertAlmostEqual(loaded.FF1, 0.05)

            new_params = PIDParams(
                P=42.5,
                I=8.0,
                D=0.18,
                FF0=0.0,
                FF1=0.1002,
                FF2=0.00085,
                deadband=0.0015,
                bias=0.01,
            )
            ok, save_msg = backend.save_params_to_ini(ini_path, "X", new_params)
            self.assertTrue(ok, save_msg)

            # Check backup file was created
            backups = list(Path(tmpdir).glob("mill.ini.bak.*"))
            self.assertEqual(len(backups), 1)

            reloaded, _ = backend.load_params_from_ini(ini_path, "X")
            assert reloaded is not None
            self.assertAlmostEqual(reloaded.P, 42.5)
            self.assertAlmostEqual(reloaded.FF1, 0.1002)
            self.assertAlmostEqual(reloaded.FF2, 0.00085)

    def test_gui_workflow_and_attempt_history(self) -> None:
        """Verify GUI initialization, running 3 attempts, history logging, and best-attempt restore."""
        import tkinter as tk
        from lcnc_pid_tuner.gui import PIDAutoTunerGUI

        root = tk.Tk()
        root.withdraw()
        try:
            app = PIDAutoTunerGUI(root, force_sim=True)
            app.run_auto_sequence(steps=3)
            self.assertEqual(len(app.attempts), 3)
            self.assertLess(
                app.attempts[-1].kpis.ferror_rms,
                app.attempts[0].kpis.ferror_rms * 0.10,
            )
            app.load_best_attempt_params()
        finally:
            root.destroy()

    def test_soft_limit_protection_and_direction_flip(self) -> None:
        """Verify soft-limit validation prevents hitting limits and suggests safe flipped/clamped travel."""
        backend = LinuxCNCBackend(force_sim=True)
        backend.axis_limits["X"] = (-100.0, 100.0)

        # Safe move inside [-100, 100]
        ok, sug, _ = backend.validate_travel_within_limits("X", start_pos=20.0, travel_distance=40.0)
        self.assertTrue(ok)
        self.assertAlmostEqual(sug, 40.0)

        # Unsafe move (+50 from +70 = +120 > +100 limit), should suggest flipping to -50
        ok2, sug2, _ = backend.validate_travel_within_limits("X", start_pos=70.0, travel_distance=50.0)
        self.assertFalse(ok2)
        self.assertAlmostEqual(sug2, -50.0)

    def test_language_switching_fr_and_en(self) -> None:
        """Verify switching between English, French, and German updates UI labels, menus, and rationale."""
        import tkinter as tk
        from lcnc_pid_tuner.gui import PIDAutoTunerGUI

        root = tk.Tk()
        root.withdraw()
        try:
            app = PIDAutoTunerGUI(root, force_sim=True, default_lang="en")
            self.assertIn("Axis:", app.ui_str["axis_label"].get())
            app.run_single_test()

            # Switch to French dynamically
            app.set_language("fr")
            self.assertEqual(app.lang, "fr")
            self.assertIn("Axe", app.ui_str["axis_label"].get())
            self.assertIn("Raideur", app.ui_str["param_P"].get())
            rationale_fr = app.rationale_text.get("1.0", tk.END)
            self.assertIn("Essai #1", rationale_fr)

            # Switch to German dynamically
            app.set_language("de")
            self.assertEqual(app.lang, "de")
            self.assertIn("Achse", app.ui_str["axis_label"].get())
            self.assertIn("Steifigkeit", app.ui_str["param_P"].get())
            rationale_de = app.rationale_text.get("1.0", tk.END)
            self.assertIn("Versuch #1", rationale_de)

            # Switch back to English
            app.set_language("en")
            self.assertEqual(app.lang, "en")
            self.assertIn("Stiffness", app.ui_str["param_P"].get())
        finally:
            root.destroy()


if __name__ == "__main__":
    unittest.main()
