"""Strict safety validation for AI-proposed or user-edited PID & Feedforward parameters."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from .models import PIDParams


# Hard physical & safety bounds for Velocity-Mode Servos + Linear Scales
PARAM_BOUNDS: Dict[str, Tuple[float, float]] = {
    "P": (0.0, 500.0),
    "I": (0.0, 200.0),
    "D": (0.0, 5.0),
    "FF0": (-10.0, 10.0),
    "FF1": (0.0, 50.0),
    "FF2": (0.0, 5.0),
    "deadband": (0.0, 0.5),
    "bias": (-50.0, 50.0),
    "maxerrorI": (0.0, 2.0),
    "maxoutput": (0.5, 100.0),
}


@dataclass
class ValidationResult:
    """Outcome of validating a set of proposed PID & Feedforward parameters."""

    valid: bool
    validated_params: PIDParams
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    changes: Dict[str, Dict[str, float]] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "valid": self.valid,
            "errors": list(self.errors),
            "warnings": list(self.warnings),
            "validated_params": self.validated_params.to_dict(),
            "changes": dict(self.changes),
        }


def validate_pid_params(
    proposed: Dict[str, Any],
    current: Optional[PIDParams] = None,
    strict_step_limit: bool = True,
) -> ValidationResult:
    """
    Validate a dictionary of proposed PID / FF parameters against:
      1. Numeric type & finiteness (no NaN / Inf).
      2. Hard safety bounds (no negative P/I/D/FF1/FF2/deadband, bounded maxoutput).
      3. Velocity-mode + linear-scale engineering sanity checks (FF0, high D, bias vs maxoutput).
      4. Single-step jump limits relative to `current` when `strict_step_limit=True`.
    """
    errors: List[str] = []
    warnings: List[str] = []

    if not isinstance(proposed, dict):
        return ValidationResult(
            valid=False,
            validated_params=(current.copy() if current else PIDParams()),
            errors=["Proposed parameters must be a JSON object / dictionary."],
        )

    base = current.copy() if current is not None else PIDParams()
    merged = base.to_dict()

    # Support common LinuxCNC aliases (e.g. Pgain -> P, Igain -> I, Dgain -> D)
    alias_map = {
        "Pgain": "P",
        "Igain": "I",
        "Dgain": "D",
        "DEADBAND": "deadband",
        "BIAS": "bias",
        "MAXERRORI": "maxerrorI",
        "MAX_ERROR_I": "maxerrorI",
        "MAXOUTPUT": "maxoutput",
        "MAX_OUTPUT": "maxoutput",
    }

    normalized_input: Dict[str, Any] = {}
    for raw_k, raw_v in proposed.items():
        k = alias_map.get(raw_k, raw_k)
        if k not in PARAM_BOUNDS:
            warnings.append(f"Ignored unknown parameter '{raw_k}'.")
            continue
        normalized_input[k] = raw_v

    # 1. Type, finiteness, and hard bounds checks
    for k, raw_v in normalized_input.items():
        if isinstance(raw_v, bool):
            errors.append(f"Parameter '{k}' must be a number, not boolean.")
            continue
        try:
            val = float(raw_v)
        except (TypeError, ValueError):
            errors.append(f"Parameter '{k}' has non-numeric value: {raw_v!r}.")
            continue

        if not math.isfinite(val):
            errors.append(f"Parameter '{k}' must be finite (got {val}).")
            continue

        lo, hi = PARAM_BOUNDS[k]
        if val < lo or val > hi:
            errors.append(
                f"Parameter '{k}'={val} is outside safe bounds [{lo}, {hi}]."
            )
            continue

        merged[k] = val

    candidate = PIDParams.from_dict(merged)

    # 2. Engineering sanity checks for Velocity-Mode Servo + Linear Scale
    if abs(candidate.FF0) > 1e-6:
        warnings.append(
            f"FF0={candidate.FF0} is non-zero; for linear positioning axes, FF0 should normally be 0.0."
        )

    if candidate.D > 0.05:
        warnings.append(
            f"D={candidate.D} is relatively high for a velocity-mode servo with linear scale feedback "
            "and may amplify encoder quantization noise."
        )

    if abs(candidate.bias) > 0.5 * candidate.maxoutput:
        errors.append(
            f"|bias|={abs(candidate.bias)} exceeds 50% of maxoutput={candidate.maxoutput}."
        )

    if candidate.P == 0.0 and (candidate.I > 0.0 or candidate.D > 0.0):
        warnings.append(
            "P=0.0 while I or D > 0; a position loop normally requires P > 0."
        )

    if candidate.I > 0.0 and candidate.maxerrorI == 0.0:
        warnings.append(
            "I > 0 with maxerrorI=0.0 disables integrator anti-windup clamping in LinuxCNC HAL."
        )

    if candidate.deadband > 0.05:
        warnings.append(
            f"deadband={candidate.deadband} mm is large for linear scale feedback (> 50 um)."
        )

    # 3. Continuous single-step jump limits relative to current parameters (no threshold gaps)
    if strict_step_limit and current is not None:
        max_p = max(30.0, current.P * 3.0)
        if candidate.P > max_p:
            errors.append(
                f"Step jump too large: P ({current.P} -> {candidate.P}) exceeds safe single-step limit ({max_p:.2f})."
            )

        max_d = max(0.05, current.D * 5.0)
        if candidate.D > max_d:
            errors.append(
                f"Step jump too large: D ({current.D} -> {candidate.D}) exceeds safe single-step limit ({max_d:.4g})."
            )

        max_i = max(25.0, current.I * 3.0)
        if candidate.I > max_i:
            errors.append(
                f"Step jump too large: I ({current.I} -> {candidate.I}) exceeds safe single-step limit ({max_i:.2f})."
            )

        max_ff1 = max(2.0, current.FF1 * 2.5)
        if candidate.FF1 > max_ff1:
            errors.append(
                f"Step jump too large: FF1 ({current.FF1} -> {candidate.FF1}) exceeds safe single-step limit ({max_ff1:.4g})."
            )

        max_ff2 = max(0.05, current.FF2 * 5.0)
        if candidate.FF2 > max_ff2:
            errors.append(
                f"Step jump too large: FF2 ({current.FF2} -> {candidate.FF2}) exceeds safe single-step limit ({max_ff2:.4g})."
            )

    # 4. Compute exact parameter diff (only when validation passed)
    is_valid = len(errors) == 0
    changes: Dict[str, Dict[str, float]] = {}
    if is_valid:
        cur_dict = base.to_dict()
        cand_dict = candidate.to_dict()
        for k in PARAM_BOUNDS:
            old_v = cur_dict[k]
            new_v = cand_dict[k]
            if abs(new_v - old_v) > 1e-9:
                changes[k] = {
                    "old": old_v,
                    "new": new_v,
                    "delta": round(new_v - old_v, 6),
                }

    return ValidationResult(
        valid=is_valid,
        validated_params=candidate if is_valid else base,
        errors=errors,
        warnings=warnings,
        changes=changes,
    )
