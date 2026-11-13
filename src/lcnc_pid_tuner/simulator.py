"""Physics simulator of a LinuxCNC velocity-mode servo + ballscrew + linear scale."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict
import numpy as np

from .gcode_gen import build_trapezoidal_profile
from .kpi_analyzer import segment_motion_phases
from .models import MotionConfig, PIDParams, SampleTrace


@dataclass
class AxisPlantModel:
    """Physical characteristics of a velocity-mode servo axis with linear scale."""

    # Velocity command scaling: 1.0 unit of pid.output -> 10.0 mm/s actual velocity
    # (Therefore true ideal FF1 = 1 / 10.0 = 0.1000 V/(mm/s))
    vel_gain_mms_per_output: float = 10.0
    # Servo drive internal velocity-loop time constant (seconds)
    vel_loop_tau: float = 0.0085  # 8.5 ms lag -> ideal FF2 ~ tau * FF1 = 0.00085
    # Ballscrew + linear guideway axial natural frequency & damping
    mech_omega_n: float = 580.0  # rad/s (~92 Hz axial ballscrew mode)
    mech_zeta: float = 0.75  # Linear guideway + nut viscous damping
    # Mechanical backlash / lost motion before linear scale moves (mm)
    backlash_mm: float = 0.002  # 2 um preload play
    # Linear scale resolution step (mm)
    scale_resolution_mm: float = 0.001  # 1 um glass scale
    # Constant load disturbance in output units (e.g., gravity on Z axis)
    gravity_bias_output: float = 0.0


AXIS_PLANTS: Dict[str, AxisPlantModel] = {
    "X": AxisPlantModel(
        vel_gain_mms_per_output=10.0,
        vel_loop_tau=0.0085,
        backlash_mm=0.002,
        scale_resolution_mm=0.001,
        gravity_bias_output=0.0,
    ),
    "Y": AxisPlantModel(
        vel_gain_mms_per_output=10.0,
        vel_loop_tau=0.0095,
        backlash_mm=0.0025,
        scale_resolution_mm=0.001,
        gravity_bias_output=0.0,
    ),
    "Z": AxisPlantModel(
        vel_gain_mms_per_output=8.0,
        vel_loop_tau=0.010,
        backlash_mm=0.002,
        scale_resolution_mm=0.001,
        gravity_bias_output=-0.35,  # Gravity pulling Z down
    ),
    "A": AxisPlantModel(
        vel_gain_mms_per_output=12.5,
        vel_loop_tau=0.007,
        backlash_mm=0.0015,
        scale_resolution_mm=0.001,
        gravity_bias_output=0.0,
    ),
}


def simulate_axis_run(
    config: MotionConfig,
    params: PIDParams,
    seed: int = 42,
) -> SampleTrace:
    """
    Run a 1 kHz time-domain simulation of the LinuxCNC `pid` component
    controlling a velocity-mode servo drive with a quantized linear scale.
    """
    plant = AXIS_PLANTS.get(config.axis_letter.upper(), AXIS_PLANTS["X"])
    t, pos_cmd, vel_cmd, acc_cmd = build_trapezoidal_profile(config)
    n = len(t)
    dt = float(t[1] - t[0]) if n > 1 else 0.001

    rng = np.random.default_rng(seed)

    pos_fb = np.zeros(n)
    vel_fb = np.zeros(n)
    ferror = np.zeros(n)
    pid_output = np.zeros(n)
    saturated = np.zeros(n)

    # Internal physical states
    motor_pos = float(pos_cmd[0])
    motor_vel = 0.0
    table_pos = float(pos_cmd[0])
    table_vel = 0.0
    prev_scale_pos = float(pos_cmd[0])
    integrator_err = 0.0

    kv = plant.vel_gain_mms_per_output
    tau_v = max(0.001, plant.vel_loop_tau)
    wn = plant.mech_omega_n
    zeta = plant.mech_zeta
    half_bl = 0.5 * plant.backlash_mm
    res = max(1e-5, plant.scale_resolution_mm)

    for i in range(n):
        # 1. Quantized linear scale reading
        scale_pos = np.round(table_pos / res) * res
        pos_fb[i] = scale_pos

        if i > 0:
            vel_fb[i] = (scale_pos - prev_scale_pos) / dt
        prev_scale_pos = scale_pos

        # 2. True Following Error (joint.N.f-error = pos_cmd - pos_fb)
        raw_err = pos_cmd[i] - scale_pos
        ferror[i] = raw_err

        # 3. LinuxCNC HAL pid deadband evaluation
        if abs(raw_err) <= params.deadband:
            pid_err = 0.0
        elif raw_err > 0:
            pid_err = raw_err - params.deadband
        else:
            pid_err = raw_err + params.deadband

        # 4. Integrator with maxerrorI clamp (anti-windup freezes integration if saturated)
        next_integrator_err = integrator_err
        if params.I > 0.0:
            next_integrator_err += pid_err * dt
            if params.maxerrorI > 0.0:
                next_integrator_err = float(
                    np.clip(next_integrator_err, -params.maxerrorI, params.maxerrorI)
                )
        else:
            next_integrator_err = 0.0

        # 5. Derivative term: LinuxCNC pid uses (command_deriv - feedback_deriv)
        err_deriv = vel_cmd[i] - vel_fb[i]

        # 6. Feedforward terms (FF0, FF1, FF2) + PID + bias
        u_ff = (
            params.FF0 * pos_cmd[i]
            + params.FF1 * vel_cmd[i]
            + params.FF2 * acc_cmd[i]
        )
        u_fb = (
            params.P * pid_err
            + params.I * next_integrator_err
            + params.D * err_deriv
        )
        u_raw = params.bias + u_ff + u_fb

        # 7. Output saturation check (maxoutput == 0 disables limiting in LinuxCNC pid)
        if params.maxoutput > 0.0 and abs(u_raw) >= params.maxoutput:
            u_cmd = float(np.clip(u_raw, -params.maxoutput, params.maxoutput))
            saturated[i] = 1.0
            # Freeze integrator on saturation (LinuxCNC pid.c anti-windup)
        else:
            u_cmd = float(u_raw)
            saturated[i] = 0.0
            integrator_err = next_integrator_err

        pid_output[i] = u_cmd

        # 8. Advance physical servo drive + mechanical compliance (sub-stepped 8x for accuracy)
        sub_steps = 8
        dt_sub = dt / sub_steps
        noise_std = 0.001 / np.sqrt(sub_steps)
        for _ in range(sub_steps):
            v_target = kv * (
                u_cmd + plant.gravity_bias_output + rng.normal(0.0, noise_std)
            )
            motor_acc = (v_target - motor_vel) / tau_v
            motor_vel += motor_acc * dt_sub
            motor_pos += motor_vel * dt_sub

            # Smooth backlash deadzone between ballscrew nut and table carriage
            delta_x = motor_pos - table_pos
            if abs(delta_x) <= half_bl:
                spring_deflection = 0.15 * delta_x
            elif delta_x > half_bl:
                spring_deflection = (delta_x - half_bl) + 0.15 * half_bl
            else:
                spring_deflection = (delta_x + half_bl) - 0.15 * half_bl

            table_acc = (
                (wn**2) * spring_deflection
                + 2.0 * zeta * wn * (motor_vel - table_vel)
            )
            table_vel += table_acc * dt_sub
            table_pos += table_vel * dt_sub

    trace = SampleTrace(
        t=t,
        pos_cmd=pos_cmd,
        pos_fb=pos_fb,
        vel_cmd=vel_cmd,
        vel_fb=vel_fb,
        acc_cmd=acc_cmd,
        ferror=ferror,
        pid_output=pid_output,
        saturated=saturated,
    )
    segment_motion_phases(trace)
    return trace
