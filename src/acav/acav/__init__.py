"""ACAV: Attitude-Coupled Access Verification.

A case study built on stk_bridge and the MATLAB ADCS twin. It tests
whether mission planning that assumes instantaneous, perfect pointing
over-promises imaging capacity, by closing the loop: STK computes target
access from the attitude a Simulink ADCS twin actually achieves.

Scenario and contracts
    ``spec``          Loads and validates config/acav_scenario.yaml.
    ``slew_model``    Rest-to-rest slew time, shared by scheduler and twin.
    ``metrics``       The capture criterion and every reported metric.

STK scenario
    ``scenario_builder``, ``scenario_verifier``, ``ground_track``,
    ``stk_queries``, ``scenario_generator``, ``scenarios``.

ADCS twin reference
    ``attitude``        Quaternion algebra and frame conventions.
    ``twin_reference``  Dynamics, wheels, profile, control law, mode
                        logic: what the MATLAB twin transcribes.
    ``twin_fixtures``   The cross-language fixtures between the two.

Scheduler
    ``scheduling``, ``capture_tracking``, ``scheduler_node``.

Experiment harness and analysis
    ``run_record``      Run provenance and serialisation.
    ``fault_injection`` Seeded wheel faults, resolved before a run.
    ``campaign``        Run matrices and what their results mean.
    ``campaign_runner`` Executes a matrix through a launch command.
    ``deploy_run_cli``  Orchestrates one run (bridge, twin, scheduler).
    ``paired_analysis``, ``paired_stats``, ``report_figures``.
"""

from acav.attitude import (  # noqa: F401
    boresight_error_rad,
    pointing_error_rad,
    quat_align,
    quat_error,
    quat_from_axis_angle,
    quat_multiply,
    quat_normalize,
    rotate_to_body,
    rotate_to_inertial,
)
from acav.campaign import (  # noqa: F401
    Campaign,
    RunSpec,
    build_e2_scheduler_comparison,
    build_e3_determinism,
    build_e5_fault_campaign,
    build_e5_replay,
    compare_schedulers,
    determinism_comparison,
    replay_matches,
)
from acav.fault_injection import (  # noqa: F401
    WheelFault,
    fault_from_dict,
    plan_campaign_faults,
    plan_fault,
)
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
from acav.run_record import (  # noqa: F401
    Provenance,
    RecordError,
    RunRecord,
    read_record,
    scenario_digest,
    unreportable,
    write_record,
)
from acav.slew_model import (  # noqa: F401
    SlewLimits,
    off_nadir_angle_deg,
    slew_between_targets_rad,
    slew_time_deg_s,
    slew_time_s,
)
from acav.spec import AcavSpec, TargetSpec  # noqa: F401
from acav.twin_reference import (  # noqa: F401
    MissionMode,
    ReactionWheels,
    RigidBody,
    SlewProfile,
    TwinConfig,
    nadir_attitude,
    settling_time_s,
    simulate_slew,
    target_attitude,
    tracking_torque,
    update_mode,
)

__version__ = "0.1.0"

__all__ = [
    "AcavSpec", "Campaign", "CaptureOutcome", "MissionMode",
    "PointingSample", "Provenance", "ReactionWheels", "RecordError",
    "RigidBody", "RunRecord", "RunResult", "RunSpec", "SlewLimits",
    "SlewProfile", "TargetResult", "TargetSpec", "TwinConfig", "WheelFault",
    "boresight_error_rad", "build_e2_scheduler_comparison",
    "build_e3_determinism",
    "build_e5_fault_campaign", "build_e5_replay",
    "capture_rate", "compare_schedulers", "coverage_rate",
    "determinism_comparison", "determinism_report",
    "determinism_signature", "evaluate_capture", "fault_from_dict",
    "nadir_attitude",
    "off_nadir_angle_deg", "outcome_breakdown", "over_promise_gap",
    "pointing_error_rad", "pointing_error_stats", "quat_align",
    "quat_error", "quat_from_axis_angle", "quat_multiply",
    "quat_normalize", "rotate_to_body", "rotate_to_inertial",
    "plan_campaign_faults", "plan_fault", "read_record", "replay_matches",
    "scenario_digest", "settling_time_s", "simulate_slew",
    "slew_between_targets_rad", "slew_time_deg_s", "slew_time_s",
    "target_attitude", "tracking_torque", "unreportable", "update_mode",
    "write_record",
]
