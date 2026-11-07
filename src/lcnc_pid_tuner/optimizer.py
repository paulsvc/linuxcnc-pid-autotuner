"""Closed-loop PID + Feedforward optimizer for Velocity-Mode Servos with Linear Scales."""

from __future__ import annotations

from typing import List, Optional

from .models import (
    KPIResult,
    MotionConfig,
    PIDParams,
    TuningAttempt,
    TuningSuggestion,
)


class PIDAutoOptimizer:
    """
    Analyzes KPIs from the current test run and the history of previous attempts
    to recommend optimal PID + Feedforward (FF0, FF1, FF2, bias, deadband, maxerrorI)
    parameters for a LinuxCNC velocity-mode servo with linear scale feedback.
    """

    CHATTER_WARN_LIMIT = 0.14
    CHATTER_CRITICAL_LIMIT = 0.25

    def suggest_next(
        self,
        current_params: PIDParams,
        kpis: KPIResult,
        motion: MotionConfig,
        history: Optional[List[TuningAttempt]] = None,
    ) -> TuningSuggestion:
        history = history or []
        next_p = current_params.copy()
        rationale: List[str] = []
        warnings: List[str] = []

        v_cruise = max(1.0, motion.cruise_velocity_mms)
        a_max = max(10.0, motion.max_acceleration)
        scale_res = max(0.0005, kpis.estimated_scale_res)

        # ------------------------------------------------------------------
        # 0. Check Drive Saturation
        # ------------------------------------------------------------------
        if kpis.saturation_pct > 5.0:
            warnings.append(
                f"Drive saturated for {kpis.saturation_pct:.1f}% of the move "
                f"(|pid.output| hit maxoutput={current_params.maxoutput}). "
                "Consider lowering feedrate/acceleration or checking OUTPUT_SCALE."
            )

        # ------------------------------------------------------------------
        # 1. Stability & Anti-Hunting Guard (Highest Priority)
        # ------------------------------------------------------------------
        if kpis.output_chatter_rms > self.CHATTER_CRITICAL_LIMIT:
            stage_name = "Stability Recovery (Reduce Chatter / Buzz)"
            warnings.append(
                f"High-frequency drive chatter detected (RMS={kpis.output_chatter_rms:.3f}). "
                "Linear scale quantization or mechanical resonance is being amplified."
            )
            if current_params.D > 0.005:
                next_p.D = round(current_params.D * 0.5, 5)
                rationale.append(
                    f"Reduced Dgain ({current_params.D:.4g} -> {next_p.D:.4g}) because D amplifies "
                    f"linear scale step quantization ({scale_res*1000:.1f} um)."
                )
            next_p.P = round(max(5.0, current_params.P * 0.78), 4)
            rationale.append(
                f"Backed off Pgain ({current_params.P:.4g} -> {next_p.P:.4g}) by 22% to restore stability margin."
            )
            return TuningSuggestion(
                suggested_params=next_p,
                stage_name=stage_name,
                rationale=rationale,
                warnings=warnings,
            )

        # Check for standstill backlash hunting (limit-cycle oscillation at rest)
        hunting_threshold = max(0.004, scale_res * 4.5, current_params.deadband * 2.5)
        if kpis.standstill_hunting_p2p > hunting_threshold and current_params.I > 0.0:
            stage_name = "Anti-Hunting Protection (Linear Scale Backlash)"
            warnings.append(
                f"Standstill hunting detected ({kpis.standstill_hunting_p2p*1000:.1f} um peak-to-peak). "
                "Integral action is cycling across mechanical backlash."
            )
            next_p.I = round(current_params.I * 0.4, 4)
            next_p.deadband = round(
                max(current_params.deadband * 1.4, scale_res * 2.0, 0.0015), 5
            )
            next_p.maxerrorI = round(min(current_params.maxerrorI, 0.015), 4)
            rationale.append(
                f"Reduced Igain ({current_params.I:.4g} -> {next_p.I:.4g}) and widened deadband "
                f"to {next_p.deadband*1000:.1f} um to stop backlash limit-cycling."
            )
            return TuningSuggestion(
                suggested_params=next_p,
                stage_name=stage_name,
                rationale=rationale,
                warnings=warnings,
            )

        # ------------------------------------------------------------------
        # 2. Ensure Safe Deadband & FF0 for Linear Scale Axis
        # ------------------------------------------------------------------
        min_safe_deadband = round(max(scale_res * 1.5, 0.0015), 5)
        if current_params.deadband < min_safe_deadband:
            next_p.deadband = min_safe_deadband
            rationale.append(
                f"Set deadband to {next_p.deadband*1000:.1f} um (1.5x linear scale resolution) "
                "to prevent encoder quantization dither at standstill."
            )

        if abs(current_params.FF0) > 1e-6:
            next_p.FF0 = 0.0
            rationale.append(
                "Reset FF0 to 0.0 (position feedforward must be 0 on linear axes; "
                "velocity-mode servos use FF1 and FF2)."
            )

        # ------------------------------------------------------------------
        # 3. Closed-Loop Sensitivity Calculations
        # ------------------------------------------------------------------
        # In closed loop during steady cruise:
        #   u_drive = FF1 * v_cmd + P * e_cruise + bias
        # The missing feedforward velocity effort is P * e_cruise:
        #   delta_FF1 = (P * e_cruise) / v_cruise
        p_eff = max(current_params.P, 5.0)
        cruise_err_abs = abs(kpis.cruise_error_mean)
        combined_acc_err = 0.5 * (kpis.accel_peak_error + kpis.decel_peak_error)
        accel_err_abs = abs(combined_acc_err)

        # Stage 1: Coarse Velocity Feedforward (FF1) & Gravity/Load Bias
        if current_params.FF1 <= 1e-5 or cruise_err_abs > 0.012:
            stage_name = "Stage 1: Velocity Feedforward (FF1) & Bias Calibration"

            # Account for deadband offset in P effort when cruise error is large
            sign_c = 1.0 if kpis.cruise_error_mean >= 0 else -1.0
            eff_cruise_err = kpis.cruise_error_mean
            if abs(eff_cruise_err) > current_params.deadband:
                eff_cruise_err -= sign_c * current_params.deadband * 0.5

            gain_step = 0.96 if current_params.FF1 == 0.0 else 0.88
            delta_ff1 = gain_step * (p_eff * eff_cruise_err) / v_cruise
            next_p.FF1 = round(max(0.0, current_params.FF1 + delta_ff1), 6)
            rationale.append(
                f"Cruise following error averaged {kpis.cruise_error_mean*1000:+.1f} um at "
                f"{v_cruise:.1f} mm/s. Updated FF1 ({current_params.FF1:.5g} -> {next_p.FF1:.5g}) "
                "so the velocity command matches axis speed directly."
            )

            # Check directional asymmetry (e.g., Z-axis gravity load)
            if abs(kpis.cruise_asymmetry) > 0.003:
                delta_bias = 0.85 * (p_eff * kpis.cruise_asymmetry)
                next_p.bias = round(current_params.bias + delta_bias, 5)
                rationale.append(
                    f"Detected directional asymmetry ({kpis.cruise_asymmetry*1000:+.1f} um between + and - moves). "
                    f"Adjusted bias ({current_params.bias:.4g} -> {next_p.bias:.4g}) to counter constant load/gravity."
                )

            if current_params.P < 18.0:
                next_p.P = round(max(18.0, current_params.P * 1.4), 4)
                rationale.append(
                    f"Raised Pgain ({current_params.P:.4g} -> {next_p.P:.4g}) to tighten position tracking."
                )

            return TuningSuggestion(
                suggested_params=next_p,
                stage_name=stage_name,
                rationale=rationale,
                warnings=warnings,
            )

        # Stage 2: Acceleration Feedforward (FF2) & Fine FF1 Calibration
        # On velocity-mode servos, the internal velocity loop has a time lag tau_v (~5-12 ms).
        # Tuning FF2 before cranking P/D eliminates acceleration/deceleration spikes and braking overshoot.
        if (current_params.FF2 <= 1e-6 and accel_err_abs > 0.002) or accel_err_abs > 0.004:
            stage_name = "Stage 2: Acceleration Feedforward (FF2) & Fine FF1"

            if cruise_err_abs > 0.0005:
                delta_ff1 = 0.85 * (p_eff * kpis.cruise_error_mean) / v_cruise
                next_p.FF1 = round(max(0.0, current_params.FF1 + delta_ff1), 6)
                rationale.append(
                    f"Refined FF1 ({current_params.FF1:.5g} -> {next_p.FF1:.5g}) to zero out "
                    f"{kpis.cruise_error_mean*1000:+.1f} um residual cruise lag."
                )

            delta_ff2 = 0.80 * (p_eff * combined_acc_err) / a_max
            next_p.FF2 = round(max(0.0, current_params.FF2 + delta_ff2), 6)
            rationale.append(
                f"Accel/Decel transient lag averaged {combined_acc_err*1000:+.1f} um at "
                f"{a_max:.0f} mm/s^2. Updated FF2 ({current_params.FF2:.5g} -> {next_p.FF2:.5g}) "
                "to pre-compensate servo velocity-loop lag and eliminate braking overshoot."
            )

            if abs(kpis.cruise_asymmetry) > 0.002:
                delta_bias = 0.80 * (p_eff * kpis.cruise_asymmetry)
                next_p.bias = round(current_params.bias + delta_bias, 5)
                rationale.append(
                    f"Refined bias ({current_params.bias:.4g} -> {next_p.bias:.4g}) for symmetric tracking."
                )

            return TuningSuggestion(
                suggested_params=next_p,
                stage_name=stage_name,
                rationale=rationale,
                warnings=warnings,
            )

        # Stage 3: Proportional (P) Stiffness & Gentle Derivative (D) Optimization
        # Check if previous P increase degraded score
        p_plateaued = False
        if len(history) >= 1:
            prev = history[-1]
            if (
                current_params.P > prev.params.P * 1.05
                and kpis.overall_score >= prev.kpis.overall_score * 0.99
            ):
                p_plateaued = True

        if (
            not p_plateaued
            and kpis.output_chatter_rms < 0.70 * self.CHATTER_WARN_LIMIT
            and current_params.P < 48.0
        ):
            stage_name = "Stage 3: Proportional (P) Stiffness & Damping Optimization"
            step_factor = 1.30 if current_params.P < 30.0 else 1.18
            next_p.P = round(current_params.P * step_factor, 4)
            rationale.append(
                f"Output chatter is low ({kpis.output_chatter_rms:.3f}). Increased Pgain "
                f"({current_params.P:.4g} -> {next_p.P:.4g}) to reduce reversal spikes and increase stiffness."
            )

            # In velocity mode, the servo drive already closes the velocity loop, so Dgain
            # in LinuxCNC must stay very small (<= 0.025) to avoid amplifying 1 um scale ticks.
            if current_params.D < 0.015 and kpis.overshoot_mm > 0.004:
                next_p.D = round(min(0.020, max(0.005, current_params.D + 0.005)), 5)
                rationale.append(
                    f"Added light Dgain ({current_params.D:.4g} -> {next_p.D:.4g}) to damp "
                    f"{kpis.overshoot_mm*1000:.1f} um settling overshoot without amplifying scale quantization."
                )

            # Simultaneously polish FF1 & FF2 (use gentle gain inside deadband edge)
            if cruise_err_abs > 0.0003:
                ff1_gain = 0.35 if cruise_err_abs <= next_p.deadband * 1.8 else 0.75
                delta_ff1 = ff1_gain * (p_eff * kpis.cruise_error_mean) / v_cruise
                next_p.FF1 = round(max(0.0, current_params.FF1 + delta_ff1), 6)
            if accel_err_abs > 0.0008:
                delta_ff2 = 0.65 * (p_eff * combined_acc_err) / a_max
                next_p.FF2 = round(max(0.0, current_params.FF2 + delta_ff2), 6)
            if abs(kpis.cruise_asymmetry) > 0.0015:
                delta_bias = 0.75 * (p_eff * kpis.cruise_asymmetry)
                next_p.bias = round(current_params.bias + delta_bias, 5)

            return TuningSuggestion(
                suggested_params=next_p,
                stage_name=stage_name,
                rationale=rationale,
                warnings=warnings,
            )

        # Stage 4: Integral (Igain), Anti-Windup (maxerrorI) & Micro-Trim
        stage_name = "Stage 4: Integral (Igain), Anti-Windup & Final Polish"
        if cruise_err_abs > 0.0002:
            ff1_gain = 0.35 if cruise_err_abs <= next_p.deadband * 1.8 else 0.70
            delta_ff1 = ff1_gain * (p_eff * kpis.cruise_error_mean) / v_cruise
            next_p.FF1 = round(max(0.0, current_params.FF1 + delta_ff1), 6)
            rationale.append(
                f"Micro-trimmed FF1 ({current_params.FF1:.5g} -> {next_p.FF1:.5g}) for "
                f"{kpis.cruise_error_mean*1000:+.1f} um cruise accuracy."
            )
        if accel_err_abs > 0.0005:
            delta_ff2 = 0.60 * (p_eff * combined_acc_err) / a_max
            next_p.FF2 = round(max(0.0, current_params.FF2 + delta_ff2), 6)
            rationale.append(
                f"Micro-trimmed FF2 ({current_params.FF2:.5g} -> {next_p.FF2:.5g}) for "
                f"{combined_acc_err*1000:+.1f} um ramp accuracy."
            )
        if abs(kpis.cruise_asymmetry) > 0.001:
            delta_bias = 0.75 * (p_eff * kpis.cruise_asymmetry)
            next_p.bias = round(current_params.bias + delta_bias, 5)

        if kpis.steady_state_error > next_p.deadband * 1.1 and current_params.I < p_eff * 0.5:
            next_p.I = round(
                max(2.0, current_params.I * 1.4 if current_params.I > 0 else p_eff * 0.15),
                4,
            )
            next_p.maxerrorI = round(max(0.008, min(0.02, next_p.deadband * 10.0)), 4)
            rationale.append(
                f"Added conservative Igain ({current_params.I:.4g} -> {next_p.I:.4g}) with "
                f"maxerrorI={next_p.maxerrorI} mm to eliminate standstill residual error safely."
            )
        elif not rationale:
            rationale.append(
                f"Axis is converged (RMS ferror = {kpis.ferror_rms*1000:.1f} um, "
                f"Peak = {kpis.ferror_max*1000:.1f} um). Ready to save to .INI!"
            )

        return TuningSuggestion(
            suggested_params=next_p,
            stage_name=stage_name,
            rationale=rationale,
            warnings=warnings,
        )
