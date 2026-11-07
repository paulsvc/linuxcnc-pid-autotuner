"""Data models for LinuxCNC PID & Feedforward Auto-Tuner."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime
from typing import Dict, List, Optional
import numpy as np


@dataclass
class PIDParams:
    """Complete parameter set for a LinuxCNC HAL PID component."""

    P: float = 10.0
    I: float = 0.0
    D: float = 0.0
    FF0: float = 0.0
    FF1: float = 0.0
    FF2: float = 0.0
    deadband: float = 0.001
    bias: float = 0.0
    maxerrorI: float = 0.05
    maxoutput: float = 10.0

    def copy(self) -> "PIDParams":
        return PIDParams(**asdict(self))

    def to_dict(self) -> Dict[str, float]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, float]) -> "PIDParams":
        valid_keys = {f.name for f in cls.__dataclass_fields__.values()}
        filtered = {k: float(v) for k, v in data.items() if k in valid_keys}
        return cls(**filtered)


@dataclass
class MotionConfig:
    """Configuration for the test G-code move and recording."""

    axis_letter: str = "X"
    joint_index: int = 0
    start_pos: float = 0.0
    travel_distance: float = 50.0  # mm
    cruise_feedrate: float = 1500.0  # mm/min (25 mm/s)
    max_acceleration: float = 250.0  # mm/s^2
    dwell_time: float = 0.4  # seconds at each end
    sample_rate_hz: float = 1000.0  # 1 kHz servo thread

    @property
    def cruise_velocity_mms(self) -> float:
        """Target cruise velocity in machine units per second (mm/s)."""
        return max(1e-3, self.cruise_feedrate / 60.0)


class MotionPhase:
    """Integer labels for time-series phase segmentation."""

    STANDSTILL = 0
    ACCEL_POS = 1
    CRUISE_POS = 2
    DECEL_POS = 3
    SETTLING_POS = 4
    ACCEL_NEG = 5
    CRUISE_NEG = 6
    DECEL_NEG = 7
    SETTLING_NEG = 8

    NAMES = {
        0: "Standstill",
        1: "Accel (+)",
        2: "Cruise (+)",
        3: "Decel (+)",
        4: "Settling (+)",
        5: "Accel (-)",
        6: "Cruise (-)",
        7: "Decel (-)",
        8: "Settling (-)",
    }


@dataclass
class SampleTrace:
    """Recorded HAL oscilloscope signals from a single G-code execution."""

    t: np.ndarray  # Time in seconds
    pos_cmd: np.ndarray  # joint.N.pos-cmd (mm)
    pos_fb: np.ndarray  # joint.N.pos-fb from linear scale (mm)
    vel_cmd: np.ndarray  # joint.N.vel-cmd (mm/s)
    vel_fb: np.ndarray  # pid.N.feedback-deriv (mm/s)
    acc_cmd: np.ndarray  # Derived commanded acceleration (mm/s^2)
    ferror: np.ndarray  # joint.N.f-error (mm)
    pid_output: np.ndarray  # pid.N.output (drive command)
    saturated: np.ndarray  # pid.N.saturated (0 or 1)
    phase: np.ndarray = field(default_factory=lambda: np.array([], dtype=int))


@dataclass
class KPIResult:
    """Comprehensive KPIs extracted from a recorded trace."""

    # Overall tracking KPIs (in mm)
    ferror_rms: float = 0.0
    ferror_max: float = 0.0

    # Feedforward / Motion Phase KPIs (in mm)
    cruise_error_pos: float = 0.0  # Signed mean error during + cruise
    cruise_error_neg: float = 0.0  # Signed mean error during - cruise
    cruise_error_mean: float = 0.0  # Directional signed cruise lag (+ means lagging behind move)
    cruise_asymmetry: float = 0.0  # Difference (+ vs -) indicating gravity/bias load
    accel_peak_error: float = 0.0  # Signed error aligned with acceleration direction
    decel_peak_error: float = 0.0  # Signed error aligned with deceleration direction
    reversal_spike: float = 0.0  # Peak spike at direction reversal (backlash/stiction)

    # Feedback / Step & Settling KPIs
    overshoot_mm: float = 0.0  # Peak overshoot at end of move (mm)
    overshoot_pct: float = 0.0  # Overshoot relative to peak dynamic error (%)
    settling_time_ms: float = 0.0  # Time after decel end for |ferror| <= deadband (ms)
    steady_state_error: float = 0.0  # Mean |ferror| during post-move standstill (mm)
    zero_crossings_settling: int = 0  # Number of oscillations during settling

    # Linear Scale & Stability KPIs
    standstill_hunting_p2p: float = 0.0  # Peak-to-peak hunting amplitude at standstill (mm)
    output_chatter_rms: float = 0.0  # High-frequency ripple in pid.N.output (drive units)
    saturation_pct: float = 0.0  # % of motion samples where drive hit maxoutput
    estimated_scale_res: float = 0.001  # Estimated linear scale step size (mm)

    # Composite quality score (lower is better, 0 = ideal)
    overall_score: float = 0.0

    def to_dict(self) -> Dict[str, float]:
        return asdict(self)


@dataclass
class TuningSuggestion:
    """Recommended parameter update and engineering justification."""

    suggested_params: PIDParams
    stage_name: str
    rationale: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)


@dataclass
class TuningAttempt:
    """Record of one test run in the succession of tuning attempts."""

    attempt_id: int
    timestamp: str
    axis: str
    stage_name: str
    params: PIDParams
    kpis: KPIResult
    suggestion: TuningSuggestion
    trace: SampleTrace
    gcode_used: str = ""

    @classmethod
    def create(
        cls,
        attempt_id: int,
        axis: str,
        stage_name: str,
        params: PIDParams,
        kpis: KPIResult,
        suggestion: TuningSuggestion,
        trace: SampleTrace,
        gcode_used: str = "",
    ) -> "TuningAttempt":
        return cls(
            attempt_id=attempt_id,
            timestamp=datetime.now().strftime("%H:%M:%S"),
            axis=axis,
            stage_name=stage_name,
            params=params.copy(),
            kpis=kpis,
            suggestion=suggestion,
            trace=trace,
            gcode_used=gcode_used,
        )
