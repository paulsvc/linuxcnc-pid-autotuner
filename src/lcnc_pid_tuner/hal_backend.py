"""LinuxCNC HAL / NML interface and .INI configuration manager with Simulation fallback."""

from __future__ import annotations

import os
import re
import shutil
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import numpy as np

from .gcode_gen import generate_mdi_commands
from .kpi_analyzer import segment_motion_phases
from .models import MotionConfig, PIDParams, SampleTrace
from .simulator import simulate_axis_run

try:
    import linuxcnc  # type: ignore

    HAS_LINUXCNC = True
except ImportError:
    linuxcnc = None
    HAS_LINUXCNC = False


AXIS_TO_JOINT = {"X": 0, "Y": 1, "Z": 2, "A": 3, "B": 4, "C": 5}


class LinuxCNCBackend:
    """
    Unified hardware/simulation backend for reading/writing PID parameters,
    running tuning G-code moves, recording HAL signals, and saving to .INI files.
    """

    def __init__(self, force_sim: bool = False) -> None:
        self.force_sim = force_sim
        self._sim_params: Dict[str, PIDParams] = {
            "X": PIDParams(P=12.0, I=0.0, D=0.0, FF0=0.0, FF1=0.0, FF2=0.0, deadband=0.0005),
            "Y": PIDParams(P=12.0, I=0.0, D=0.0, FF0=0.0, FF1=0.0, FF2=0.0, deadband=0.0005),
            "Z": PIDParams(P=15.0, I=0.0, D=0.0, FF0=0.0, FF1=0.0, FF2=0.0, deadband=0.0005),
            "A": PIDParams(P=10.0, I=0.0, D=0.0, FF0=0.0, FF1=0.0, FF2=0.0, deadband=0.0005),
        }
        self._abort_flag = threading.Event()
        self.ini_path: Optional[Path] = self._auto_detect_ini()
        # Default soft limits per axis (min_mm, max_mm)
        self.axis_limits: Dict[str, Tuple[float, float]] = {
            "X": (-500.0, 500.0),
            "Y": (-500.0, 500.0),
            "Z": (-300.0, 100.0),
            "A": (-3600.0, 3600.0),
        }

    def read_current_position(self, axis_letter: str) -> float:
        """Read current commanded/actual position of the selected axis in mm."""
        axis = axis_letter.upper()
        j = AXIS_TO_JOINT.get(axis, 0)
        if not self.is_live_linuxcnc:
            return 0.0
        try:
            stat = linuxcnc.stat()
            stat.poll()
            return round(float(stat.joint[j]["input"]), 4)
        except Exception:
            return round(self._hal_get(f"joint.{j}.pos-fb", 0.0), 4)

    def read_axis_limits(self, axis_letter: str) -> Tuple[float, float]:
        """Read soft limits (min_limit, max_limit) from live LinuxCNC or cached INI."""
        axis = axis_letter.upper()
        j = AXIS_TO_JOINT.get(axis, 0)
        if self.is_live_linuxcnc:
            try:
                stat = linuxcnc.stat()
                stat.poll()
                min_lim = float(stat.joint[j]["min_position_limit"])
                max_lim = float(stat.joint[j]["max_position_limit"])
                if max_lim > min_lim:
                    self.axis_limits[axis] = (min_lim, max_lim)
            except Exception:
                pass
        return self.axis_limits.get(axis, (-500.0, 500.0))

    def validate_travel_within_limits(
        self, axis_letter: str, start_pos: float, travel_distance: float, margin: float = 2.0
    ) -> Tuple[bool, float, str]:
        """
        Verify that [start_pos, start_pos + travel_distance] stays inside soft limits
        with `margin` mm of clearance. Returns (is_safe, suggested_safe_travel, message).
        """
        min_lim, max_lim = self.read_axis_limits(axis_letter)
        safe_min = min_lim + margin
        safe_max = max_lim - margin
        end_pos = start_pos + travel_distance

        if safe_min <= start_pos <= safe_max and safe_min <= end_pos <= safe_max:
            return (
                True,
                travel_distance,
                f"Move [{start_pos:.1f} -> {end_pos:.1f} mm] is safely within limits [{min_lim:.1f}, {max_lim:.1f} mm].",
            )

        # Check if flipping direction makes it safe
        flipped_end = start_pos - travel_distance
        if safe_min <= start_pos <= safe_max and safe_min <= flipped_end <= safe_max:
            return (
                False,
                -travel_distance,
                f"Target {end_pos:.1f} mm exceeds limit [{min_lim:.1f}, {max_lim:.1f} mm]. "
                f"Flipping travel to {-travel_distance:+.1f} mm stays safe.",
            )

        # Otherwise clamp travel to maximum available room from start_pos
        room_pos = max(0.0, safe_max - start_pos)
        room_neg = max(0.0, start_pos - safe_min)
        if room_pos >= room_neg and room_pos >= 2.0:
            clamped = round(min(abs(travel_distance), room_pos), 2)
        elif room_neg >= 2.0:
            clamped = -round(min(abs(travel_distance), room_neg), 2)
        else:
            clamped = 0.0

        return (
            False,
            clamped,
            f"Target {end_pos:.1f} mm exceeds axis limits [{min_lim:.1f}, {max_lim:.1f} mm]. "
            f"Recommended safe travel from {start_pos:.1f} mm is {clamped:+.1f} mm.",
        )

    @property
    def is_live_linuxcnc(self) -> bool:
        """Return True if connected to a running LinuxCNC instance and not in sim mode."""
        if self.force_sim or not HAS_LINUXCNC:
            return False
        try:
            stat = linuxcnc.stat()
            stat.poll()
            return True
        except Exception:
            return False

    def _auto_detect_ini(self) -> Optional[Path]:
        """Detect active INI file from LinuxCNC environment variable if available."""
        env_ini = os.environ.get("INI_FILE_NAME")
        if env_ini and os.path.isfile(env_ini):
            return Path(env_ini)
        return None

    def _resolve_pid_prefix(self, axis_letter: str, joint_index: int) -> str:
        """
        Determine whether the HAL configuration names PID pins `pid.x.*`, `pid.X.*`,
        or `pid.0.*` (pncconf uses `pid.x`, traditional configs use `pid.0`).
        """
        candidates = [
            f"pid.{axis_letter.lower()}",
            f"pid.{joint_index}",
            f"pid.{axis_letter.upper()}",
        ]
        try:
            out = subprocess.check_output(
                ["halcmd", "show", "pin", "pid."],
                stderr=subprocess.DEVNULL,
                text=True,
                timeout=1.5,
            )
            for cand in candidates:
                if f"{cand}.Pgain" in out:
                    return cand
        except Exception:
            pass
        return f"pid.{joint_index}"

    def _hal_get(self, pin_or_param: str, default: float = 0.0) -> float:
        try:
            out = subprocess.check_output(
                ["halcmd", "-s", "getp", pin_or_param],
                stderr=subprocess.DEVNULL,
                text=True,
                timeout=1.0,
            ).strip()
            if out.lower() in ("true", "1"):
                return 1.0
            if out.lower() in ("false", "0"):
                return 0.0
            return float(out)
        except Exception:
            return default

    def _hal_set(self, pin_or_param: str, value: float) -> bool:
        try:
            subprocess.check_call(
                ["halcmd", "setp", pin_or_param, f"{value:.6g}"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=1.0,
            )
            return True
        except Exception:
            return False

    def read_pid_params(self, axis_letter: str) -> PIDParams:
        """Read current PID & FF parameters from live HAL (or simulation memory)."""
        axis = axis_letter.upper()
        if not self.is_live_linuxcnc:
            return self._sim_params.get(axis, PIDParams()).copy()

        j = AXIS_TO_JOINT.get(axis, 0)
        prefix = self._resolve_pid_prefix(axis, j)
        return PIDParams(
            P=self._hal_get(f"{prefix}.Pgain", 10.0),
            I=self._hal_get(f"{prefix}.Igain", 0.0),
            D=self._hal_get(f"{prefix}.Dgain", 0.0),
            FF0=self._hal_get(f"{prefix}.FF0", 0.0),
            FF1=self._hal_get(f"{prefix}.FF1", 0.0),
            FF2=self._hal_get(f"{prefix}.FF2", 0.0),
            deadband=self._hal_get(f"{prefix}.deadband", 0.001),
            bias=self._hal_get(f"{prefix}.bias", 0.0),
            maxerrorI=self._hal_get(f"{prefix}.maxerrorI", 0.05),
            maxoutput=self._hal_get(f"{prefix}.maxoutput", 10.0),
        )

    def write_pid_params(self, axis_letter: str, params: PIDParams) -> Tuple[bool, str]:
        """Apply PID & FF parameters immediately to HAL (or simulation memory)."""
        axis = axis_letter.upper()
        self._sim_params[axis] = params.copy()

        if not self.is_live_linuxcnc:
            return True, f"Applied parameters to Axis {axis} (Simulation Mode)."

        j = AXIS_TO_JOINT.get(axis, 0)
        prefix = self._resolve_pid_prefix(axis, j)
        mapping = {
            f"{prefix}.Pgain": params.P,
            f"{prefix}.Igain": params.I,
            f"{prefix}.Dgain": params.D,
            f"{prefix}.FF0": params.FF0,
            f"{prefix}.FF1": params.FF1,
            f"{prefix}.FF2": params.FF2,
            f"{prefix}.deadband": params.deadband,
            f"{prefix}.bias": params.bias,
            f"{prefix}.maxerrorI": params.maxerrorI,
            f"{prefix}.maxoutput": params.maxoutput,
        }
        ok_count = sum(1 for pin, val in mapping.items() if self._hal_set(pin, val))
        if ok_count == len(mapping):
            return True, f"Updated all {ok_count} HAL pins on {prefix}.*"
        return (
            ok_count > 0,
            f"Updated {ok_count}/{len(mapping)} HAL pins on {prefix}.*",
        )

    def abort_motion(self) -> None:
        """Immediately stop any running G-code test."""
        self._abort_flag.set()
        if self.is_live_linuxcnc:
            try:
                cmd = linuxcnc.command()
                cmd.abort()
            except Exception:
                pass

    def run_test_and_record(
        self, config: MotionConfig, params: PIDParams
    ) -> SampleTrace:
        """
        Apply `params`, execute the tuning G-code move for `config.axis_letter`,
        and capture the oscilloscope trace.
        """
        self._abort_flag.clear()
        self.write_pid_params(config.axis_letter, params)

        if not self.is_live_linuxcnc:
            return simulate_axis_run(config, params)

        return self._run_live_linuxcnc_capture(config, params)

    def _run_live_linuxcnc_capture(
        self, config: MotionConfig, params: PIDParams
    ) -> SampleTrace:
        """Execute MDI commands on live LinuxCNC while sampling HAL pins."""
        stat = linuxcnc.stat()
        cmd = linuxcnc.command()
        j = AXIS_TO_JOINT.get(config.axis_letter.upper(), config.joint_index)
        prefix = self._resolve_pid_prefix(config.axis_letter, j)

        stat.poll()
        if stat.task_state != linuxcnc.STATE_ON:
            raise RuntimeError(
                "LinuxCNC machine is not ON (STATE_ON). Please release E-Stop and turn Machine Power ON."
            )

        # Switch to MDI mode
        cmd.mode(linuxcnc.MODE_MDI)
        cmd.wait_complete(2.0)

        t_list: List[float] = []
        pos_cmd_list: List[float] = []
        pos_fb_list: List[float] = []
        vel_cmd_list: List[float] = []
        vel_fb_list: List[float] = []
        ferror_list: List[float] = []
        pid_out_list: List[float] = []
        sat_list: List[float] = []

        stop_sampling = threading.Event()
        t0 = time.perf_counter()

        def sampler_worker() -> None:
            dt_target = 1.0 / max(200.0, min(1000.0, config.sample_rate_hz))
            prev_t = 0.0
            prev_fb = 0.0
            while not stop_sampling.is_set() and not self._abort_flag.is_set():
                loop_start = time.perf_counter()
                now = loop_start - t0
                try:
                    stat.poll()
                    p_cmd = float(stat.joint[j]["output"])
                    p_fb = float(stat.joint[j]["input"])
                    f_err = float(stat.joint[j]["following_error"])
                    v_cmd = float(stat.joint[j]["velocity"])
                except Exception:
                    p_cmd = self._hal_get(f"joint.{j}.pos-cmd")
                    p_fb = self._hal_get(f"joint.{j}.pos-fb")
                    f_err = self._hal_get(f"joint.{j}.f-error")
                    v_cmd = self._hal_get(f"joint.{j}.vel-cmd")

                v_fb = (p_fb - prev_fb) / max(1e-4, now - prev_t) if t_list else 0.0
                prev_t = now
                prev_fb = p_fb

                u_out = self._hal_get(f"{prefix}.output", 0.0)
                sat = 1.0 if abs(u_out) >= 0.99 * params.maxoutput else 0.0

                t_list.append(now)
                pos_cmd_list.append(p_cmd)
                pos_fb_list.append(p_fb)
                vel_cmd_list.append(v_cmd)
                vel_fb_list.append(v_fb)
                ferror_list.append(f_err)
                pid_out_list.append(u_out)
                sat_list.append(sat)

                elapsed = time.perf_counter() - loop_start
                sleep_time = dt_target - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        worker = threading.Thread(target=sampler_worker, daemon=True)
        worker.start()

        try:
            for mdi_line in generate_mdi_commands(config):
                if self._abort_flag.is_set():
                    cmd.abort()
                    break
                cmd.mdi(mdi_line)
                while True:
                    if self._abort_flag.is_set():
                        cmd.abort()
                        break
                    stat.poll()
                    if stat.interp_state == linuxcnc.INTERP_IDLE and stat.inpos:
                        break
                    time.sleep(0.01)
        finally:
            stop_sampling.set()
            worker.join(timeout=1.5)

        t_arr = np.array(t_list, dtype=float)
        vel_cmd_arr = np.array(vel_cmd_list, dtype=float)
        acc_cmd_arr = (
            np.gradient(vel_cmd_arr, t_arr)
            if len(t_arr) > 2
            else np.zeros_like(t_arr)
        )

        trace = SampleTrace(
            t=t_arr,
            pos_cmd=np.array(pos_cmd_list, dtype=float),
            pos_fb=np.array(pos_fb_list, dtype=float),
            vel_cmd=vel_cmd_arr,
            vel_fb=np.array(vel_fb_list, dtype=float),
            acc_cmd=acc_cmd_arr,
            ferror=np.array(ferror_list, dtype=float),
            pid_output=np.array(pid_out_list, dtype=float),
            saturated=np.array(sat_list, dtype=float),
        )
        segment_motion_phases(trace)
        return trace

    # ------------------------------------------------------------------
    # .INI Configuration File Reader & Updater
    # ------------------------------------------------------------------
    def load_params_from_ini(
        self, ini_file: Path, axis_letter: str
    ) -> Tuple[Optional[PIDParams], str]:
        """Read PID & FF parameters from a LinuxCNC .ini file section ([JOINT_N] or [AXIS_X])."""
        if not ini_file.is_file():
            return None, f"INI file not found: {ini_file}"

        j = AXIS_TO_JOINT.get(axis_letter.upper(), 0)
        target_sections = [f"JOINT_{j}", f"AXIS_{axis_letter.upper()}", f"AXIS_{j}"]

        text = ini_file.read_text(encoding="utf-8", errors="ignore")
        current_sec = ""
        found_vals: Dict[str, float] = {}
        min_lim_val: Optional[float] = None
        max_lim_val: Optional[float] = None

        key_map = {
            "P": "P",
            "I": "I",
            "D": "D",
            "FF0": "FF0",
            "FF1": "FF1",
            "FF2": "FF2",
            "DEADBAND": "deadband",
            "BIAS": "bias",
            "MAX_ERROR_I": "maxerrorI",
            "MAXERRORI": "maxerrorI",
            "MAX_OUTPUT": "maxoutput",
        }

        for raw_line in text.splitlines():
            line = raw_line.split("#")[0].split(";")[0].strip()
            if line.startswith("[") and line.endswith("]"):
                current_sec = line[1:-1].strip().upper()
                continue
            if current_sec in target_sections and "=" in line:
                k, v = [p.strip() for p in line.split("=", 1)]
                k_up = k.upper()
                if k_up in key_map:
                    try:
                        found_vals[key_map[k_up]] = float(v)
                    except ValueError:
                        pass
                elif k_up == "MIN_LIMIT":
                    try:
                        min_lim_val = float(v)
                    except ValueError:
                        pass
                elif k_up == "MAX_LIMIT":
                    try:
                        max_lim_val = float(v)
                    except ValueError:
                        pass

        if min_lim_val is not None and max_lim_val is not None and max_lim_val > min_lim_val:
            self.axis_limits[axis_letter.upper()] = (min_lim_val, max_lim_val)

        if not found_vals:
            return None, f"No PID parameters found in sections {target_sections}"

        params = PIDParams.from_dict(found_vals)
        self._sim_params[axis_letter.upper()] = params.copy()
        return params, f"Loaded parameters for {axis_letter.upper()} from {ini_file.name}"

    def save_params_to_ini(
        self, ini_file: Path, axis_letter: str, params: PIDParams
    ) -> Tuple[bool, str]:
        """
        Safely update PID & FF parameters in a LinuxCNC .ini file after creating
        a timestamped `.bak` backup copy.
        """
        if not ini_file.is_file():
            return False, f"INI file does not exist: {ini_file}"

        backup_path = ini_file.with_suffix(
            f".ini.bak.{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        )
        shutil.copy2(ini_file, backup_path)

        j = AXIS_TO_JOINT.get(axis_letter.upper(), 0)
        target_sections = {f"JOINT_{j}", f"AXIS_{axis_letter.upper()}", f"AXIS_{j}"}

        ini_updates = {
            "P": f"{params.P:.6g}",
            "I": f"{params.I:.6g}",
            "D": f"{params.D:.6g}",
            "FF0": f"{params.FF0:.6g}",
            "FF1": f"{params.FF1:.6g}",
            "FF2": f"{params.FF2:.6g}",
            "DEADBAND": f"{params.deadband:.6g}",
            "BIAS": f"{params.bias:.6g}",
        }

        lines = ini_file.read_text(encoding="utf-8", errors="ignore").splitlines()
        new_lines: List[str] = []
        current_sec = ""
        updated_keys_in_sec: set[str] = set()
        matched_section = False

        for raw_line in lines:
            stripped = raw_line.strip()
            if stripped.startswith("[") and stripped.endswith("]"):
                # Before leaving a matched target section, append any missing keys
                if current_sec in target_sections:
                    for k, val_str in ini_updates.items():
                        if k not in updated_keys_in_sec:
                            new_lines.append(f"{k} = {val_str}")
                current_sec = stripped[1:-1].strip().upper()
                updated_keys_in_sec = set()
                if current_sec in target_sections:
                    matched_section = True
                new_lines.append(raw_line)
                continue

            if current_sec in target_sections and "=" in raw_line and not stripped.startswith(("#", ";")):
                m = re.match(r"^(\s*[A-Za-z0-9_]+\s*)=.*$", raw_line)
                if m:
                    key_raw = m.group(1).strip().upper()
                    if key_raw in ini_updates:
                        new_lines.append(f"{m.group(1).strip()} = {ini_updates[key_raw]}")
                        updated_keys_in_sec.add(key_raw)
                        continue

            new_lines.append(raw_line)

        if current_sec in target_sections:
            for k, val_str in ini_updates.items():
                if k not in updated_keys_in_sec:
                    new_lines.append(f"{k} = {val_str}")

        if not matched_section:
            # Append a new [JOINT_N] section at the end if none existed
            new_lines.append("")
            new_lines.append(f"[JOINT_{j}]")
            for k, val_str in ini_updates.items():
                new_lines.append(f"{k} = {val_str}")

        ini_file.write_text("\n".join(new_lines) + "\n", encoding="utf-8")
        return (
            True,
            f"Saved parameters to [{f'JOINT_{j}'}] in {ini_file.name} (Backup: {backup_path.name})",
        )
