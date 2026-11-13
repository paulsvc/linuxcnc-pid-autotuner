"""Motion phase segmentation and KPI calculation engine for LinuxCNC axes."""

from __future__ import annotations

from typing import Any, Dict, List, Optional
import numpy as np

from .models import KPIResult, MotionPhase, PIDParams, SampleTrace


def segment_motion_phases(
    trace: SampleTrace, settling_window_sec: float = 0.25
) -> np.ndarray:
    """
    Automatically classify each timestamp in the recorded trace into one of
    the 9 MotionPhase states based on commanded velocity and acceleration.
    """
    n = len(trace.t)
    phases = np.full(n, MotionPhase.STANDSTILL, dtype=int)
    if n < 5:
        trace.phase = phases
        return phases

    v = trace.vel_cmd
    a = trace.acc_cmd
    dt = float(np.median(np.diff(trace.t))) if n > 1 else 0.001

    v_peak = float(np.max(np.abs(v)))
    a_peak = float(np.max(np.abs(a)))
    v_thresh = max(1e-3, 0.02 * v_peak)
    a_thresh = max(1e-2, 0.10 * a_peak)

    # Identify active positive and negative motion regions
    pos_move = v > v_thresh
    neg_move = v < -v_thresh

    # Positive move phases
    phases[pos_move & (a > a_thresh)] = MotionPhase.ACCEL_POS
    phases[pos_move & (np.abs(a) <= a_thresh)] = MotionPhase.CRUISE_POS
    phases[pos_move & (a < -a_thresh)] = MotionPhase.DECEL_POS

    # Negative move phases
    phases[neg_move & (a < -a_thresh)] = MotionPhase.ACCEL_NEG
    phases[neg_move & (np.abs(a) <= a_thresh)] = MotionPhase.CRUISE_NEG
    phases[neg_move & (a > a_thresh)] = MotionPhase.DECEL_NEG

    # Identify post-deceleration settling windows
    settling_samples = max(5, int(round(settling_window_sec / max(1e-4, dt))))

    # Find transitions from DECEL_POS to STANDSTILL
    decel_pos_idx = np.where(phases == MotionPhase.DECEL_POS)[0]
    if len(decel_pos_idx) > 0:
        end_pos = int(decel_pos_idx[-1]) + 1
        stop_pos = min(n, end_pos + settling_samples)
        for i in range(end_pos, stop_pos):
            if phases[i] == MotionPhase.STANDSTILL:
                phases[i] = MotionPhase.SETTLING_POS
            else:
                break

    # Find transitions from DECEL_NEG to STANDSTILL
    decel_neg_idx = np.where(phases == MotionPhase.DECEL_NEG)[0]
    if len(decel_neg_idx) > 0:
        end_neg = int(decel_neg_idx[-1]) + 1
        stop_neg = min(n, end_neg + settling_samples)
        for i in range(end_neg, stop_neg):
            if phases[i] == MotionPhase.STANDSTILL:
                phases[i] = MotionPhase.SETTLING_NEG
            else:
                break

    trace.phase = phases
    return phases


def analyze_trace(trace: SampleTrace, params: PIDParams) -> KPIResult:
    """
    Compute all mechanical & control KPIs from a recorded SampleTrace.

    Uses orthogonal motion-phase projection (velocity component, acceleration
    component, and directional bias component) so that FF1, FF2, and bias
    errors are cleanly separated without cross-talk.
    """
    if len(trace.phase) != len(trace.t):
        segment_motion_phases(trace)

    phases = trace.phase
    ferror = trace.ferror
    v_cmd = trace.vel_cmd
    a_cmd = trace.acc_cmd
    t = trace.t
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.001

    kpis = KPIResult()
    if len(ferror) == 0:
        return kpis

    # 1. Global tracking metrics
    kpis.ferror_rms = float(np.sqrt(np.mean(ferror**2)))
    kpis.ferror_max = float(np.max(np.abs(ferror)))

    # 2. Estimate linear scale quantization step (minimum non-zero position step)
    fb_diffs = np.abs(np.diff(trace.pos_fb))
    nonzero_diffs = fb_diffs[fb_diffs > 1e-5]
    if len(nonzero_diffs) > 0:
        kpis.estimated_scale_res = float(min(0.005, np.min(nonzero_diffs)))
    else:
        kpis.estimated_scale_res = 0.001

    # 3. Cruise Following Error (FF1 & Bias KPIs)
    # Trim first 25% of cruise to ensure acceleration transients have settled
    def trim_mask(phase_id: int, trim_start_frac: float = 0.25) -> np.ndarray:
        idx = np.where(phases == phase_id)[0]
        if len(idx) > 8:
            cut = int(len(idx) * trim_start_frac)
            idx = idx[cut:]
        return idx

    idx_cruise_pos = trim_mask(MotionPhase.CRUISE_POS)
    idx_cruise_neg = trim_mask(MotionPhase.CRUISE_NEG)

    if len(idx_cruise_pos) > 0:
        kpis.cruise_error_pos = float(np.mean(ferror[idx_cruise_pos]))
    if len(idx_cruise_neg) > 0:
        kpis.cruise_error_neg = float(np.mean(ferror[idx_cruise_neg]))

    if len(idx_cruise_pos) > 0 and len(idx_cruise_neg) > 0:
        kpis.cruise_error_mean = 0.5 * (kpis.cruise_error_pos - kpis.cruise_error_neg)
        kpis.cruise_asymmetry = 0.5 * (kpis.cruise_error_pos + kpis.cruise_error_neg)
    elif len(idx_cruise_pos) > 0:
        kpis.cruise_error_mean = kpis.cruise_error_pos
        kpis.cruise_asymmetry = 0.0
    elif len(idx_cruise_neg) > 0:
        kpis.cruise_error_mean = -kpis.cruise_error_neg
        kpis.cruise_asymmetry = 0.0

    # 4. Acceleration & Deceleration Transient Errors (FF2 KPIs)
    # Subtract the instantaneous velocity-proportional error e_vel(t) so that
    # accel_peak_error measures pure acceleration lag (even during velocity ramps!)
    v_peak = max(1e-3, float(np.max(np.abs(v_cmd))))
    e_vel_instant = (
        kpis.cruise_error_mean * (v_cmd / v_peak) + kpis.cruise_asymmetry
    )
    residual_dyn_err = ferror - e_vel_instant

    idx_acc_pos = np.where(phases == MotionPhase.ACCEL_POS)[0]
    idx_acc_neg = np.where(phases == MotionPhase.ACCEL_NEG)[0]
    idx_dec_pos = np.where(phases == MotionPhase.DECEL_POS)[0]
    idx_dec_neg = np.where(phases == MotionPhase.DECEL_NEG)[0]

    # Signed acceleration error aligned with sign(a_cmd):
    # Positive value means axis lags behind acceleration/braking -> needs MORE FF2
    acc_aligned = []
    if len(idx_acc_pos) > 4:
        acc_aligned.append(float(np.mean(residual_dyn_err[idx_acc_pos])))
    if len(idx_acc_neg) > 4:
        acc_aligned.append(float(np.mean(-residual_dyn_err[idx_acc_neg])))
    if acc_aligned:
        kpis.accel_peak_error = float(np.mean(acc_aligned))

    dec_aligned = []
    if len(idx_dec_pos) > 4:
        # During + move deceleration, a_cmd < 0, so lag in direction of a_cmd is -residual
        dec_aligned.append(float(np.mean(-residual_dyn_err[idx_dec_pos])))
    if len(idx_dec_neg) > 4:
        # During - move deceleration, a_cmd > 0, so lag in direction of a_cmd is +residual
        dec_aligned.append(float(np.mean(residual_dyn_err[idx_dec_neg])))
    if dec_aligned:
        kpis.decel_peak_error = float(np.mean(dec_aligned))

    # 5. Reversal Spike / Backlash & Stiction indicator (first 40 ms of motion start)
    onset_window = max(3, int(0.040 / max(1e-4, dt)))
    rev_spikes = []
    if len(idx_acc_pos) >= onset_window:
        rev_spikes.append(float(np.max(np.abs(ferror[idx_acc_pos[:onset_window]]))))
    if len(idx_acc_neg) >= onset_window:
        rev_spikes.append(float(np.max(np.abs(ferror[idx_acc_neg[:onset_window]]))))
    if rev_spikes:
        kpis.reversal_spike = float(np.max(rev_spikes))

    # 6. Settling Time, Overshoot & Zero Crossings (P & D Stiffness/Damping KPIs)
    tol = max(params.deadband * 1.5, kpis.estimated_scale_res * 2.0, 0.002)
    overshoots = []
    settling_times = []
    zc_count = 0

    for phase_id, move_dir in [
        (MotionPhase.SETTLING_POS, 1.0),
        (MotionPhase.SETTLING_NEG, -1.0),
    ]:
        idx_settle = np.where(phases == phase_id)[0]
        if len(idx_settle) == 0:
            continue
        err_s = ferror[idx_settle]
        os_val = float(np.max(np.maximum(0.0, -move_dir * err_s)))
        overshoots.append(os_val)

        outside = np.where(np.abs(err_s) > tol)[0]
        if len(outside) == 0:
            settling_times.append(0.0)
        else:
            settling_times.append(float((outside[-1] + 1) * dt * 1000.0))

        active_s = err_s[np.abs(err_s) > params.deadband]
        if len(active_s) > 2:
            signs = np.sign(active_s)
            zc_count += int(np.sum(np.abs(np.diff(signs)) > 1.0))

    if overshoots:
        kpis.overshoot_mm = float(np.max(overshoots))
        denom = max(kpis.ferror_max, 0.010)
        kpis.overshoot_pct = float(min(200.0, (kpis.overshoot_mm / denom) * 100.0))
    if settling_times:
        kpis.settling_time_ms = float(np.max(settling_times))
    kpis.zero_crossings_settling = zc_count

    # 7. Standstill Steady-State Error & Backlash Hunting (I & deadband KPIs)
    idx_standstill = np.where(phases == MotionPhase.STANDSTILL)[0]
    if len(idx_standstill) > 10:
        err_ss = ferror[idx_standstill]
        kpis.steady_state_error = float(np.mean(np.abs(err_ss)))
        kpis.standstill_hunting_p2p = float(
            np.percentile(err_ss, 98) - np.percentile(err_ss, 2)
        )

    # 8. Drive Output High-Frequency Chatter (D & P instability indicator)
    if len(trace.pid_output) > 4:
        hf_diff = np.diff(trace.pid_output, n=2)
        kpis.output_chatter_rms = float(np.sqrt(np.mean(hf_diff**2)))

    # 9. Drive Saturation Ratio
    active_motion = phases != MotionPhase.STANDSTILL
    if np.any(active_motion):
        kpis.saturation_pct = float(
            np.mean(trace.saturated[active_motion] > 0.5) * 100.0
        )

    # 10. Composite Score (lower is better)
    rms_um = kpis.ferror_rms * 1000.0
    max_um = kpis.ferror_max * 1000.0
    cruise_um = abs(kpis.cruise_error_mean) * 1000.0
    acc_um = abs(kpis.accel_peak_error) * 1000.0
    os_um = kpis.overshoot_mm * 1000.0
    hunt_um = kpis.standstill_hunting_p2p * 1000.0
    chatter_penalty = max(0.0, kpis.output_chatter_rms - 0.12) * 200.0
    sat_penalty = kpis.saturation_pct * 10.0

    kpis.overall_score = round(
        1.5 * rms_um
        + 0.6 * max_um
        + 1.2 * cruise_um
        + 0.8 * acc_um
        + 1.0 * os_um
        + 2.0 * hunt_um
        + 0.12 * kpis.settling_time_ms
        + chatter_penalty
        + sat_penalty,
        2,
    )

    return kpis


def summarize_trace_for_ai(
    trace: SampleTrace,
    max_points: int = 120,
    phase_filter: Optional[str] = None,
) -> Dict[str, Any]:
    """
    Build a structured, AI-friendly summary of the raw oscilloscope trace:
      - `phase_breakdown`: statistical summary per motion phase
      - `trace_samples`: downsampled time-series (preserving local ferror peaks)
        so the AI can inspect waveform shape, ringing, stiction, and saturation
        in addition to aggregated KPIs.
    """
    if len(trace.phase) != len(trace.t):
        segment_motion_phases(trace)

    n = len(trace.t)
    if n == 0:
        return {"total_samples": 0, "duration_s": 0.0, "phase_breakdown": {}, "trace_samples": []}

    dt = float(np.median(np.diff(trace.t))) if n > 1 else 0.001
    sample_rate_hz = round(1.0 / max(1e-5, dt), 1)

    # 1. Per-phase breakdown
    phase_breakdown: Dict[str, Dict[str, Any]] = {}
    for phase_id, phase_name in MotionPhase.NAMES.items():
        idx = np.where(trace.phase == phase_id)[0]
        if len(idx) == 0:
            continue
        fe_um = trace.ferror[idx] * 1000.0
        out = trace.pid_output[idx]
        phase_breakdown[phase_name] = {
            "samples": int(len(idx)),
            "duration_ms": round(len(idx) * dt * 1000.0, 1),
            "ferror_mean_um": round(float(np.mean(fe_um)), 2),
            "ferror_min_um": round(float(np.min(fe_um)), 2),
            "ferror_max_um": round(float(np.max(fe_um)), 2),
            "ferror_rms_um": round(float(np.sqrt(np.mean(fe_um**2))), 2),
            "pid_output_mean": round(float(np.mean(out)), 3),
            "pid_output_min": round(float(np.min(out)), 3),
            "pid_output_max": round(float(np.max(out)), 3),
        }

    # 2. Optional phase filter
    if phase_filter:
        pf_lower = phase_filter.strip().lower()
        matching_ids = [
            pid for pid, pname in MotionPhase.NAMES.items() if pf_lower in pname.lower()
        ]
        if matching_ids:
            mask = np.isin(trace.phase, matching_ids)
            sel_indices = np.where(mask)[0]
        else:
            sel_indices = np.arange(n)
    else:
        sel_indices = np.arange(n)

    # 3. Peak-preserving downsampling
    if max_points > 0 and len(sel_indices) > max_points:
        # Divide into buckets and pick both first and peak |ferror| in each bucket
        buckets = np.array_split(sel_indices, max(1, max_points // 2))
        picked: List[int] = []
        for b in buckets:
            if len(b) == 0:
                continue
            picked.append(int(b[0]))
            peak_pos = int(b[int(np.argmax(np.abs(trace.ferror[b])))])
            if peak_pos != int(b[0]):
                picked.append(peak_pos)
        if int(sel_indices[-1]) not in picked:
            picked.append(int(sel_indices[-1]))
        final_idx = sorted(set(picked))[:max_points]
    else:
        final_idx = sel_indices.tolist()

    samples = [
        {
            "t_s": round(float(trace.t[i]), 4),
            "phase": MotionPhase.NAMES.get(int(trace.phase[i]), "Unknown"),
            "pos_cmd_mm": round(float(trace.pos_cmd[i]), 4),
            "pos_fb_mm": round(float(trace.pos_fb[i]), 4),
            "vel_cmd_mms": round(float(trace.vel_cmd[i]), 3),
            "acc_cmd_mms2": round(float(trace.acc_cmd[i]), 2),
            "ferror_um": round(float(trace.ferror[i] * 1000.0), 2),
            "pid_output": round(float(trace.pid_output[i]), 4),
            "saturated": bool(trace.saturated[i] > 0.5),
        }
        for i in final_idx
    ]

    return {
        "total_samples": int(n),
        "returned_samples": len(samples),
        "duration_s": round(float(trace.t[-1] - trace.t[0]), 4) if n > 1 else 0.0,
        "sample_rate_hz": sample_rate_hz,
        "phase_breakdown": phase_breakdown,
        "trace_samples": samples,
    }
