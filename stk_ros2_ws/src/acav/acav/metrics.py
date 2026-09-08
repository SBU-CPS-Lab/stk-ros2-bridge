"""Capture criterion and metric definitions -- the C4/C5/C6 contract.

The capture criterion is the study's dependent variable. It must be
defined once, precisely, and applied identically by the scheduler
(deciding what to attempt), the experiment harness (recording what
happened), and the analysis (reporting it). Three independent
implementations of "was this target captured?" would make the results
uninterpretable.

The criterion
-------------
A target is CAPTURED at its scheduled image epoch ``t_i`` if and only if
BOTH hold:

1. **STK reports sensor access** to the target at ``t_i``. This is the
   geometry half: is the target inside the sensor footprint given the
   attitude the twin actually achieved?

2. **Pointing error stays within tolerance** for the hold window:
   ``theta_err(t) <= capture_error_deg`` for all
   ``t`` in ``[t_i - settle_hold_s, t_i]``. This is the dynamics half:
   did the controller settle in time, and stay settled?

Both conditions, deliberately. Condition 1 alone is what conventional
mission planning checks, and it is exactly what over-promises capacity
when the spacecraft cannot slew fast enough. Condition 2 alone would
ignore whether the spacecraft was pointing at the *right* place.

The gap between "condition 1 only" and "conditions 1 and 2" is the
paper's headline quantity -- see :func:`capture_rate` and
:func:`over_promise_gap`.

A note on where each condition comes from
-----------------------------------------
Condition 1 is STK's answer, obtained through the bridge's access
sensing. Condition 2 is computed from the twin's telemetry. Neither is
computed twice: the harness records both and this module evaluates the
conjunction. If STK and the twin disagree about geometry (they can, at
the margins, because the scheduler's off-nadir estimate is spherical
while STK's is not), STK is correct by definition here.
"""

from __future__ import annotations

import math
import statistics
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Sequence, Tuple


class CaptureOutcome(str, Enum):
    """Why a target succeeded or failed. Ordering matters for reporting.

    Distinguishing the failure modes is essential to the paper's claim.
    "Not captured" is uninformative; "the spacecraft was still slewing"
    versus "the geometry never permitted it" are different findings, and
    only the first supports the dynamics-coupling argument.
    """

    CAPTURED = "captured"
    #: STK reported access, but the controller had not settled in time.
    #: This is the failure mode the study is about.
    NOT_SETTLED = "not_settled"
    #: The controller was settled but STK reported no access -- pointing
    #: was stable at the wrong place, or the geometry was never feasible.
    NO_ACCESS = "no_access"
    #: Neither condition held.
    NO_ACCESS_AND_NOT_SETTLED = "no_access_and_not_settled"
    #: The scheduler chose not to attempt this target.
    NOT_ATTEMPTED = "not_attempted"
    #: Spacecraft was in SAFE mode at the image epoch (E5 fault runs).
    SAFE_MODE = "safe_mode"


@dataclass
class PointingSample:
    """One telemetry sample used to evaluate the hold window."""

    time_s: float          # seconds from window start
    error_deg: float       # angle between boresight and target line of sight
    has_access: bool       # STK's access answer at this epoch
    mode: str = ""         # Stateflow mode name, for SAFE detection


@dataclass
class TargetResult:
    """Per-target outcome for one run."""

    target_id: str
    attempted: bool
    outcome: CaptureOutcome
    image_time_s: float
    #: Pointing error at the image epoch itself, degrees.
    error_at_image_deg: float = float("nan")
    #: Worst pointing error within the hold window, degrees.
    max_error_in_hold_deg: float = float("nan")
    #: Time from slew command to first entering the capture tolerance.
    settle_time_s: float = float("nan")
    #: Slew magnitude commanded to reach this target, degrees.
    slew_magnitude_deg: float = float("nan")
    #: Measured slew duration, for comparison against slew_model's bound.
    slew_duration_s: float = float("nan")

    @property
    def captured(self) -> bool:
        return self.outcome is CaptureOutcome.CAPTURED


@dataclass
class RunResult:
    """All per-target outcomes plus run-level quantities for one run."""

    run_id: str
    scheduler: str                      # "kinematic" | "dynamics_aware"
    targets: List[TargetResult] = field(default_factory=list)
    #: Peak wheel momentum as a fraction of capacity, per axis.
    peak_momentum_fraction: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    wheel_saturation_events: int = 0
    entered_safe_mode: bool = False
    #: Integral of |tau . omega| over the run, joules. Proxy for slew cost.
    wheel_energy_J: float = 0.0
    #: Bridge-reported command drops, for the honesty of the result.
    dropped_attitude_samples: int = 0
    #: Mean closed-loop step wall time, ms. Feeds E6.
    mean_step_latency_ms: float = float("nan")
    seed: Optional[int] = None

    @property
    def attempted_count(self) -> int:
        return sum(1 for t in self.targets if t.attempted)

    @property
    def captured_count(self) -> int:
        return sum(1 for t in self.targets if t.captured)


# ---------------------------------------------------------------------------
# The criterion
# ---------------------------------------------------------------------------

def evaluate_capture(samples: Sequence[PointingSample], image_time_s: float,
                     capture_error_deg: float, settle_hold_s: float,
                     safe_mode_name: str = "SAFE") -> Tuple[CaptureOutcome,
                                                            float, float]:
    """Apply the capture criterion to one target's telemetry window.

    Args:
        samples: Telemetry covering at least
            ``[image_time_s - settle_hold_s, image_time_s]``.
        image_time_s: The scheduled image epoch.
        capture_error_deg: Tolerance from ``pointing.capture_error_deg``.
        settle_hold_s: Hold duration from ``pointing.settle_hold_s``.
        safe_mode_name: Stateflow mode indicating loss of control.

    Returns:
        ``(outcome, error_at_image_deg, max_error_in_hold_deg)``.

    Raises:
        ValueError: if no samples fall inside the hold window. This is
            deliberately an error rather than a silent NOT_SETTLED:
            missing telemetry is a harness bug, and scoring it as a
            capture failure would corrupt the study's headline metric
            with data-collection problems.
    """
    window_start = image_time_s - settle_hold_s
    in_window = [s for s in samples
                 if window_start - 1e-9 <= s.time_s <= image_time_s + 1e-9]

    if not in_window:
        raise ValueError(
            f"no telemetry samples in the hold window "
            f"[{window_start:.3f}, {image_time_s:.3f}] s. Expected at "
            f"least one; check the harness's recording rate against "
            f"pointing.settle_hold_s."
        )

    if any(s.mode == safe_mode_name for s in in_window):
        at_image = _closest(in_window, image_time_s)
        return (CaptureOutcome.SAFE_MODE, at_image.error_deg,
                max(s.error_deg for s in in_window))

    settled = all(s.error_deg <= capture_error_deg for s in in_window)
    # Access is required at the image epoch specifically, not throughout
    # the hold window: the spacecraft may legitimately be slewing into
    # the footprint during settling.
    at_image = _closest(in_window, image_time_s)
    has_access = at_image.has_access

    max_error = max(s.error_deg for s in in_window)

    if has_access and settled:
        outcome = CaptureOutcome.CAPTURED
    elif has_access and not settled:
        outcome = CaptureOutcome.NOT_SETTLED
    elif settled and not has_access:
        outcome = CaptureOutcome.NO_ACCESS
    else:
        outcome = CaptureOutcome.NO_ACCESS_AND_NOT_SETTLED

    return outcome, at_image.error_deg, max_error


def _closest(samples: Sequence[PointingSample],
             time_s: float) -> PointingSample:
    return min(samples, key=lambda s: abs(s.time_s - time_s))


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------

def capture_rate(result: RunResult) -> float:
    """Captured / attempted. NaN if nothing was attempted.

    Denominator is *attempted*, not *total targets*: a scheduler that
    correctly declines an infeasible target should not be penalised for
    it. Coverage against the full target list is reported separately by
    :func:`coverage_rate`.
    """
    attempted = result.attempted_count
    if attempted == 0:
        return float("nan")
    return result.captured_count / attempted


def coverage_rate(result: RunResult, total_targets: int) -> float:
    """Captured / total targets in the scenario.

    The complement to :func:`capture_rate`. A conservative scheduler can
    score 100% capture rate by attempting one easy target; coverage
    catches that. Both must be reported together -- reporting either
    alone is misleading, which is why this module exposes both rather
    than a single "score".
    """
    if total_targets == 0:
        return float("nan")
    return result.captured_count / total_targets


def over_promise_gap(result: RunResult) -> float:
    """Fraction of attempted targets that were promised but not captured.

    ``(attempted - captured) / attempted``. For the kinematic scheduler
    this is the paper's headline quantity: capacity claimed by planning
    that ignores attitude dynamics, minus capacity actually delivered.
    """
    rate = capture_rate(result)
    return float("nan") if math.isnan(rate) else 1.0 - rate


def outcome_breakdown(result: RunResult) -> Dict[str, int]:
    """Count of targets by outcome. Attributes failures to a cause."""
    counts = {outcome.value: 0 for outcome in CaptureOutcome}
    for target in result.targets:
        counts[target.outcome.value] += 1
    return counts


def pointing_error_stats(result: RunResult) -> Dict[str, float]:
    """Pointing-error statistics across attempted targets, degrees."""
    errors = [t.error_at_image_deg for t in result.targets
              if t.attempted and not math.isnan(t.error_at_image_deg)]
    if not errors:
        return {"mean": float("nan"), "median": float("nan"),
                "max": float("nan"), "count": 0}
    return {
        "mean": statistics.fmean(errors),
        "median": statistics.median(errors),
        "max": max(errors),
        "count": len(errors),
    }


# ---------------------------------------------------------------------------
# Determinism (E3)
# ---------------------------------------------------------------------------

def determinism_signature(result: RunResult, decimals: int = 9) -> str:
    """A hash-like signature of a run's outcome, for E3.

    Two runs of the same configuration in lockstep must produce an
    identical signature. In free-run they generally will not. Comparing
    signatures rather than eyeballing summary statistics is what makes
    E3 a real test: two runs can share a capture count while differing
    in every pointing error.

    Rounded to ``decimals`` places before hashing because bit-exact
    float comparison would flag differences far below any physically
    meaningful threshold, turning a real result into noise. Nine places
    is ~1e-9 deg, which is a billion times finer than the 0.5 deg
    capture tolerance.
    """
    parts: List[str] = []
    for target in sorted(result.targets, key=lambda t: t.target_id):
        parts.append(
            f"{target.target_id}:{target.outcome.value}:"
            f"{_round_or_nan(target.error_at_image_deg, decimals)}:"
            f"{_round_or_nan(target.max_error_in_hold_deg, decimals)}"
        )
    parts.append(f"momentum:{[round(m, decimals) for m in result.peak_momentum_fraction]}")
    parts.append(f"saturations:{result.wheel_saturation_events}")
    parts.append(f"safe:{result.entered_safe_mode}")
    return "|".join(parts)


def _round_or_nan(value: float, decimals: int) -> str:
    return "nan" if math.isnan(value) else f"{round(value, decimals):.{decimals}f}"


def determinism_report(results: Sequence[RunResult]) -> Dict[str, object]:
    """Summarise repeat-run variability for E3.

    Returns the number of distinct signatures (1 == fully
    deterministic), plus the spread in capture count and pointing error
    so the magnitude of any non-determinism is visible, not just its
    presence.
    """
    if not results:
        return {"runs": 0, "distinct_signatures": 0, "deterministic": False}

    signatures = {determinism_signature(r) for r in results}
    capture_counts = [r.captured_count for r in results]
    errors = [t.error_at_image_deg for r in results for t in r.targets
              if t.attempted and not math.isnan(t.error_at_image_deg)]

    return {
        "runs": len(results),
        "distinct_signatures": len(signatures),
        "deterministic": len(signatures) == 1,
        "capture_count_min": min(capture_counts),
        "capture_count_max": max(capture_counts),
        "pointing_error_stdev_deg": (
            statistics.pstdev(errors) if len(errors) > 1 else 0.0),
    }
