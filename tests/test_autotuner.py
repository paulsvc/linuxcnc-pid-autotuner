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

    def test_gcode_live_updates_on_edit_and_flip(self) -> None:
        """Verify editing travel, start_pos, feed, or flipping direction updates G-code preview safely."""
        import tkinter as tk
        from lcnc_pid_tuner.gui import PIDAutoTunerGUI

        root = tk.Tk()
        root.withdraw()
        try:
            app = PIDAutoTunerGUI(root, force_sim=True, default_lang="en")

            # Simulate typing intermediate '-' then '-15'
            app.travel_var.set("-")
            app.travel_var.set("-15")
            app.start_pos_var.set("10")
            app.feed_var.set("900")
            gcode = app.gcode_text.get("1.0", tk.END)
            self.assertIn("G0 X10.0000", gcode)
            self.assertIn("G1 X-5.0000 F900.0", gcode)

            # Flip direction -> +15 from 10 = 25
            app.flip_travel_direction()
            gcode_flipped = app.gcode_text.get("1.0", tk.END)
            self.assertIn("G1 X25.0000 F900.0", gcode_flipped)
        finally:
            root.destroy()

    def test_validator_bounds_and_step_limits(self) -> None:
        """Verify parameter validator rejects unsafe bounds, NaN, and large single-step jumps."""
        from lcnc_pid_tuner.validator import validate_pid_params

        current = PIDParams(P=12.0, I=0.0, D=0.0, FF1=0.08, FF2=0.0)

        # 1. Negative P rejected
        res_neg = validate_pid_params({"P": -5.0}, current=current)
        self.assertFalse(res_neg.valid)
        self.assertTrue(any("outside safe bounds" in e for e in res_neg.errors))

        # 2. NaN rejected
        res_nan = validate_pid_params({"FF1": float("nan")}, current=current)
        self.assertFalse(res_nan.valid)

        # 3. Excessive single-step jump (P: 12 -> 80 > 3x) rejected when strict_step_limit=True
        res_jump = validate_pid_params({"P": 80.0}, current=current, strict_step_limit=True)
        self.assertFalse(res_jump.valid)
        self.assertTrue(any("Step jump too large" in e for e in res_jump.errors))

        # 4. Safe update accepted with warning on non-zero FF0
        res_ok = validate_pid_params(
            {"Pgain": 20.0, "FF1": 0.10, "FF0": 0.05},
            current=current,
            strict_step_limit=True,
        )
        self.assertTrue(res_ok.valid)
        self.assertAlmostEqual(res_ok.validated_params.P, 20.0)
        self.assertAlmostEqual(res_ok.validated_params.FF1, 0.10)
        self.assertTrue(any("FF0" in w for w in res_ok.warnings))
        self.assertIn("P", res_ok.changes)

    def test_mcp_server_stdio_roundtrip(self) -> None:
        """Verify MCP JSON-RPC 2.0 initialize, tools/list, run_tuning_test, and validated AI apply."""
        import io
        import json
        from lcnc_pid_tuner.mcp_server import PIDTunerMCPServer

        requests = [
            {"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {}},
            {"jsonrpc": "2.0", "method": "notifications/initialized"},
            {"jsonrpc": "2.0", "id": 2, "method": "tools/list", "params": {}},
            {
                "jsonrpc": "2.0",
                "id": 3,
                "method": "tools/call",
                "params": {"name": "run_tuning_test", "arguments": {"axis": "X"}},
            },
            # Unsafe AI proposal (negative P) must be rejected
            {
                "jsonrpc": "2.0",
                "id": 4,
                "method": "tools/call",
                "params": {
                    "name": "validate_and_apply_params",
                    "arguments": {"axis": "X", "params": {"P": -10.0}},
                },
            },
            # Safe AI proposal must be validated, applied, and re-tested
            {
                "jsonrpc": "2.0",
                "id": 5,
                "method": "tools/call",
                "params": {
                    "name": "validate_and_apply_params",
                    "arguments": {
                        "axis": "X",
                        "params": {"P": 22.0, "FF1": 0.10, "FF2": 0.00085},
                        "apply": True,
                        "run_test_after": True,
                    },
                },
            },
            {
                "jsonrpc": "2.0",
                "id": 6,
                "method": "tools/call",
                "params": {"name": "get_tuning_history", "arguments": {"axis": "X"}},
            },
            {
                "jsonrpc": "2.0",
                "id": 7,
                "method": "tools/call",
                "params": {
                    "name": "get_attempt_trace",
                    "arguments": {"attempt_id": 1, "phase_filter": "Cruise", "max_points": 40},
                },
            },
        ]

        in_buf = io.StringIO("\n".join(json.dumps(r) for r in requests) + "\n")
        out_buf = io.StringIO()

        server = PIDTunerMCPServer(force_sim=True)
        server.run_stdio(in_stream=in_buf, out_stream=out_buf)

        responses = [json.loads(line) for line in out_buf.getvalue().strip().splitlines()]
        self.assertEqual(len(responses), 7)  # notification produces no response

        # Check tools/list
        tool_names = [t["name"] for t in responses[1]["result"]["tools"]]
        self.assertIn("validate_and_apply_params", tool_names)
        self.assertIn("get_attempt_trace", tool_names)

        # Check run_tuning_test result includes BOTH KPIs and trace_analysis
        test_payload = json.loads(responses[2]["result"]["content"][0]["text"])
        self.assertEqual(test_payload["attempt_id"], 1)
        self.assertIn("kpis_um", test_payload)
        self.assertIn("trace_analysis", test_payload)
        self.assertIn("Cruise (+)", test_payload["trace_analysis"]["phase_breakdown"])
        self.assertGreater(len(test_payload["trace_analysis"]["trace_samples"]), 20)

        # Check rejected unsafe AI params
        bad_payload = json.loads(responses[3]["result"]["content"][0]["text"])
        self.assertFalse(bad_payload["validation"]["valid"])
        self.assertFalse(bad_payload["applied"])

        # Check accepted AI params + immediate verification test
        good_payload = json.loads(responses[4]["result"]["content"][0]["text"])
        self.assertTrue(good_payload["validation"]["valid"])
        self.assertTrue(good_payload["applied"])
        self.assertIn("verification_test", good_payload)
        self.assertLess(
            good_payload["verification_test"]["kpis_um"]["ferror_rms_um"], 5.0
        )

        # Check history recorded both attempts
        hist_payload = json.loads(responses[5]["result"]["content"][0]["text"])
        self.assertEqual(hist_payload["count"], 2)

        # Check get_attempt_trace phase filter
        trace_payload = json.loads(responses[6]["result"]["content"][0]["text"])
        samples = trace_payload["trace_analysis"]["trace_samples"]
        self.assertGreater(len(samples), 0)
        self.assertTrue(all("Cruise" in s["phase"] for s in samples))

    def test_hardening_validator_simulator_and_mcp_errors(self) -> None:
        """Verify hardened validator step-jumps, maxoutput=0, G94, and MCP error flags."""
        from lcnc_pid_tuner.gcode_gen import generate_mdi_commands, generate_tuning_gcode
        from lcnc_pid_tuner.mcp_server import PIDTunerMCPServer
        from lcnc_pid_tuner.simulator import simulate_axis_run
        from lcnc_pid_tuner.validator import validate_pid_params

        # 1. Validator step-jump from 0 and consistency warnings
        cur_zero = PIDParams(P=0.0, I=0.0, D=0.0, FF1=0.0, FF2=0.0)
        res_jump0 = validate_pid_params({"P": 250.0}, current=cur_zero, strict_step_limit=True)
        self.assertFalse(res_jump0.valid)
        self.assertEqual(res_jump0.changes, {})

        res_warn = validate_pid_params({"P": 0.0, "I": 5.0, "maxerrorI": 0.0}, current=cur_zero)
        self.assertTrue(res_warn.valid)
        self.assertTrue(any("P=0" in w for w in res_warn.warnings))
        self.assertTrue(any("maxerrorI=0.0" in w for w in res_warn.warnings))

        # 2. G94 feedrate mode in generated G-code and MDI
        cfg = MotionConfig(axis_letter="X", travel_distance=20.0, cruise_feedrate=1200.0)
        self.assertIn("G94", generate_tuning_gcode(cfg))
        self.assertTrue(any("G94" in line for line in generate_mdi_commands(cfg)))

        # 3. Simulator maxoutput == 0.0 means unlimited output
        trace_unlim = simulate_axis_run(
            cfg, PIDParams(P=20.0, FF1=0.1, maxoutput=0.0)
        )
        self.assertGreater(max(abs(v) for v in trace_unlim.pid_output), 0.1)

        # 4. MCP error flags & JSON-RPC validation
        server = PIDTunerMCPServer(force_sim=True)
        bad_req = server.handle_request(["not", "a", "dict"])
        self.assertEqual(bad_req["error"]["code"], -32600)

        bad_params = server.handle_request(
            {"jsonrpc": "2.0", "id": 10, "method": "tools/call", "params": "oops"}
        )
        self.assertEqual(bad_params["error"]["code"], -32602)

        bad_axis = server.handle_request(
            {
                "jsonrpc": "2.0",
                "id": 11,
                "method": "tools/call",
                "params": {"name": "get_axis_state", "arguments": {"axis": "Q"}},
            }
        )
        self.assertEqual(bad_axis["error"]["code"], -32602)

        bad_apply = server.handle_request(
            {
                "jsonrpc": "2.0",
                "id": 12,
                "method": "tools/call",
                "params": {
                    "name": "validate_and_apply_params",
                    "arguments": {"axis": "X", "params": {"P": -5.0}},
                },
            }
        )
        self.assertTrue(bad_apply["result"].get("isError"))


if __name__ == "__main__":
    unittest.main()
