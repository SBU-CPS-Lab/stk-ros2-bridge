"""ACAV -- Attitude-Coupled Access Verification.

Case study for the stk_bridge paper. Demonstrates that mission planning
which assumes instantaneous, perfect pointing over-promises imaging
capacity, by closing the loop: STK computes target access from the
attitude a Simulink ADCS twin actually achieves.

Phase C1 (this delivery) fixes the cross-phase contracts:

``spec``        Loads and validates config/acav_scenario.yaml.
``slew_model``  Rest-to-rest slew time -- shared by the scheduler (C4)
                and validated against the Simulink twin (C3).
``metrics``     The capture criterion and every reported metric, shared
                by C4, C5, and C6.

Later phases add the STK scenario builder (C2), the Simulink twin (C3),
the scheduler node (C4), the experiment harness (C5), and analysis (C6).
"""

from acav.metrics import (  # noqa: F401
    CaptureOutcome,
    PointingSample,
    RunResult,
    TargetResult,
    capture_rate,
    coverage_rate,
    determinism_report,
    determinism_signature,
    evaluate_capture,
    outcome_breakdown,
    over_promise_gap,
    pointing_error_stats,
)
from acav.slew_model import (  # noqa: F401
    SlewLimits,
    off_nadir_angle_deg,
    slew_between_targets_rad,
    slew_time_deg_s,
    slew_time_s,
)
from acav.spec import AcavSpec, TargetSpec  # noqa: F401

__version__ = "0.1.0"

__all__ = [
    "AcavSpec", "CaptureOutcome", "PointingSample", "RunResult",
    "SlewLimits", "TargetResult", "TargetSpec", "capture_rate",
    "coverage_rate", "determinism_report", "determinism_signature",
    "evaluate_capture", "off_nadir_angle_deg", "outcome_breakdown",
    "over_promise_gap", "pointing_error_stats", "slew_between_targets_rad",
    "slew_time_deg_s", "slew_time_s",
]
