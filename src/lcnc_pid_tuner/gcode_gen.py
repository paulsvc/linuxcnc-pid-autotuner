"""Safe tuning G-code generator and trapezoidal trajectory builder."""

from __future__ import annotations

from typing import List, Tuple
import numpy as np

from .models import MotionConfig


def generate_tuning_gcode(config: MotionConfig) -> str:
    """
    Generate a safe, self-contained back-and-forth G-code program
    with dwell phases to evaluate standstill noise, acceleration spikes,
    constant-velocity cruise, deceleration, and settling.
    """
    axis = config.axis_letter.upper()
    x0 = config.start_pos
    x1 = config.start_pos + config.travel_distance
    feed = config.cruise_feedrate
    dwell = config.dwell_time

    lines = [
        f"(LinuxCNC PID & Feedforward Auto-Tuner Test Cycle - Axis {axis})",
        "G21 (Metric units - mm)",
        "G90 (Absolute distance mode)",
        "G64 P0.001 (Exact path / tight blending for accurate tracking test)",
        f"G0 {axis}{x0:.4f}",
        f"G4 P{dwell:.2f} (Pre-move standstill baseline)",
        f"G1 {axis}{x1:.4f} F{feed:.1f} (Positive move: Accel -> Cruise -> Decel)",
        f"G4 P{dwell:.2f} (Post-positive move settling & standstill check)",
        f"G1 {axis}{x0:.4f} F{feed:.1f} (Negative move: Accel -> Cruise -> Decel)",
        f"G4 P{dwell:.2f} (Post-negative move settling & hunting check)",
        "M2",
    ]
    return "\n".join(lines)


def generate_mdi_commands(config: MotionConfig) -> List[str]:
    """Return sequential MDI commands to run the tuning cycle without loading a file."""
    axis = config.axis_letter.upper()
    x0 = config.start_pos
    x1 = config.start_pos + config.travel_distance
    feed = config.cruise_feedrate
    dwell = config.dwell_time

    return [
        "G21 G90 G64 P0.001",
        f"G0 {axis}{x0:.4f}",
        f"G4 P{dwell:.2f}",
        f"G1 {axis}{x1:.4f} F{feed:.1f}",
        f"G4 P{dwell:.2f}",
        f"G1 {axis}{x0:.4f} F{feed:.1f}",
        f"G4 P{dwell:.2f}",
    ]


def build_trapezoidal_profile(
    config: MotionConfig,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Synthesize the exact 1 kHz LinuxCNC trapezoidal trajectory planner command
    (t, pos_cmd, vel_cmd, acc_cmd) matching the generated G-code.
    """
    dt = 1.0 / max(100.0, config.sample_rate_hz)
    dist = abs(config.travel_distance)
    sign = 1.0 if config.travel_distance >= 0 else -1.0
    v_max = config.cruise_velocity_mms
    a_max = max(10.0, config.max_acceleration)
    dwell = max(0.2, config.dwell_time)

    # Time to accelerate to v_max
    t_acc = v_max / a_max
    d_acc = 0.5 * a_max * t_acc**2

    if 2.0 * d_acc >= dist:
        # Triangle profile if travel distance is short
        t_acc = np.sqrt(dist / a_max)
        t_cruise = 0.0
        v_peak = a_max * t_acc
    else:
        t_cruise = (dist - 2.0 * d_acc) / v_max
        v_peak = v_max

    t_move = 2.0 * t_acc + t_cruise
    t_total = dwell + t_move + dwell + t_move + dwell

    t = np.arange(0.0, t_total, dt)
    pos_cmd = np.full_like(t, config.start_pos)
    vel_cmd = np.zeros_like(t)
    acc_cmd = np.zeros_like(t)

    def eval_single_move(tau: np.ndarray, direction: float, p_start: float):
        p = np.zeros_like(tau)
        v = np.zeros_like(tau)
        a = np.zeros_like(tau)

        # Phase 1: Acceleration
        m1 = (tau >= 0.0) & (tau < t_acc)
        t1 = tau[m1]
        a[m1] = direction * a_max
        v[m1] = direction * a_max * t1
        p[m1] = p_start + direction * 0.5 * a_max * t1**2

        # Phase 2: Cruise
        m2 = (tau >= t_acc) & (tau < t_acc + t_cruise)
        t2 = tau[m2] - t_acc
        p_end_acc = p_start + direction * 0.5 * a_max * t_acc**2
        a[m2] = 0.0
        v[m2] = direction * v_peak
        p[m2] = p_end_acc + direction * v_peak * t2

        # Phase 3: Deceleration
        m3 = (tau >= t_acc + t_cruise) & (tau <= t_move)
        t3 = tau[m3] - (t_acc + t_cruise)
        p_end_cruise = p_end_acc + direction * v_peak * t_cruise
        a[m3] = -direction * a_max
        v[m3] = direction * (v_peak - a_max * t3)
        p[m3] = p_end_cruise + direction * (v_peak * t3 - 0.5 * a_max * t3**2)

        # Post-move hold
        m4 = tau > t_move
        p[m4] = p_start + direction * dist
        return p, v, a

    # Segment 1: Initial dwell [0, dwell) -> stays at start_pos
    # Segment 2: Positive move [dwell, dwell + t_move]
    t_pos_start = dwell
    t_pos_end = dwell + t_move
    mask_pos = (t >= t_pos_start) & (t < t_pos_end + dwell)
    p1, v1, a1 = eval_single_move(t[mask_pos] - t_pos_start, sign, config.start_pos)
    pos_cmd[mask_pos] = p1
    vel_cmd[mask_pos] = v1
    acc_cmd[mask_pos] = a1

    # Segment 3: Negative return move [2*dwell + t_move, end]
    t_neg_start = 2.0 * dwell + t_move
    mask_neg = t >= t_neg_start
    p2, v2, a2 = eval_single_move(
        t[mask_neg] - t_neg_start, -sign, config.start_pos + sign * dist
    )
    pos_cmd[mask_neg] = p2
    vel_cmd[mask_neg] = v2
    acc_cmd[mask_neg] = a2

    return t, pos_cmd, vel_cmd, acc_cmd
