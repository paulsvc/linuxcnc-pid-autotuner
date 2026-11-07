"""LinuxCNC PID & Feedforward Auto-Tuner package."""

from .gui import PIDAutoTunerGUI, launch_gui
from .hal_backend import LinuxCNCBackend
from .kpi_analyzer import analyze_trace, segment_motion_phases
from .models import (
    KPIResult,
    MotionConfig,
    MotionPhase,
    PIDParams,
    SampleTrace,
    TuningAttempt,
    TuningSuggestion,
)
from .optimizer import PIDAutoOptimizer
from .simulator import simulate_axis_run

__all__ = [
    "PIDAutoTunerGUI",
    "launch_gui",
    "LinuxCNCBackend",
    "analyze_trace",
    "segment_motion_phases",
    "KPIResult",
    "MotionConfig",
    "MotionPhase",
    "PIDParams",
    "SampleTrace",
    "TuningAttempt",
    "TuningSuggestion",
    "PIDAutoOptimizer",
    "simulate_axis_run",
]
