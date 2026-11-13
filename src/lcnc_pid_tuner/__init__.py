"""LinuxCNC PID & Feedforward Auto-Tuner package."""

from .gui import PIDAutoTunerGUI, launch_gui
from .hal_backend import LinuxCNCBackend
from .kpi_analyzer import analyze_trace, segment_motion_phases
from .mcp_server import PIDTunerMCPServer, run_mcp_http, run_mcp_stdio
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
from .validator import ValidationResult, validate_pid_params

__all__ = [
    "PIDAutoTunerGUI",
    "launch_gui",
    "LinuxCNCBackend",
    "analyze_trace",
    "segment_motion_phases",
    "PIDTunerMCPServer",
    "run_mcp_stdio",
    "run_mcp_http",
    "KPIResult",
    "MotionConfig",
    "MotionPhase",
    "PIDParams",
    "SampleTrace",
    "TuningAttempt",
    "TuningSuggestion",
    "PIDAutoOptimizer",
    "simulate_axis_run",
    "ValidationResult",
    "validate_pid_params",
]
