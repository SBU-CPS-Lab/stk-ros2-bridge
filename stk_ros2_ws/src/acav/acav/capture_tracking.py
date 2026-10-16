"""Turns a stream of telemetry samples into per-target capture outcomes.

Pure bookkeeping: this module owns none of the "how" (no ROS, no STK,
no quaternion math) and all of the "when" -- when has enough telemetry
arrived to score a target, and which samples belong to its hold window.
The actual criterion is applied by :func:`acav.metrics.evaluate_capture`,
never re-derived here, per that module's own stated purpose (a single,
shared definition of "captured").

Why a rolling buffer rather than per-target buffers
----------------------------------------------------
At any instant the schedule has at most one active target (activation
windows are contiguous and non-overlapping -- see
``acav/scheduling.py``), so telemetry never needs to be split across
concurrent per-target collectors. A single time-ordered deque, trimmed
to the longest window any target could still need, is simpler and
cannot leak samples into the wrong target's evaluation by construction:
:meth:`CaptureTracker.finalize` filters by time window, not by "which
target was active when this sample arrived."

Why ``has_access`` is asked for out of band
--------------------------------------------
Whether STK reports sensor access at a target's image epoch can only be
known from STK, generally after that epoch has actually elapsed and the
real-time attitude history up to it has been committed. This module
does not know how that answer was obtained (a live ComputeAccess call,
a recorded value, a test double) -- the caller supplies it to
:meth:`finalize`, which is what keeps this module testable with plain
data and reusable by both the scheduler node and any offline harness.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional

from acav.metrics import (
    CaptureOutcome,
    PointingSample,
    TargetResult,
    evaluate_capture,
)
from acav.scheduling import DeclinedEntry, ScheduleEntry


@dataclass(frozen=True)
class TelemetrySample:
    """One incoming (time, pointing error, mode) observation.

    Deliberately narrower than :class:`acav.metrics.PointingSample`: it
    has no ``has_access`` field, because nothing producing telemetry in
    real time knows that answer yet (see the module docstring). This
    class becomes a ``PointingSample`` only inside :meth:`finalize`,
    once the access answer is available.
    """

    time_s: float
    error_deg: float
    mode: str = ""
    #: Reaction wheel state from the same TwinStatus message, carried
    #: here so :class:`RunAggregate` can accumulate the run-level
    #: quantities without a second subscription to the same topic.
    #: Defaulted because the per-target capture criterion does not use
    #: them: a caller that only cares about capture need not supply them.
    wheel_momentum_Nms: tuple = (0.0, 0.0, 0.0)
    wheels_saturated: bool = False
    wheel_energy_J: float = 0.0
    #: TwinStatus.saturation_events: the twin's own cumulative count,
    #: over every integration step (free-run sub-steps included); None
    #: from a twin that does not report it.
    saturation_events: Optional[int] = None
    #: TwinStatus.max_boresight_error_since_last_rad, in degrees; None
    #: from a twin that does not report it.
    max_error_since_last_deg: Optional[float] = None



@dataclass
class RunAggregate:
    """Run-scoped quantities accumulated from the twin's telemetry.

    Separate from :class:`CaptureTracker` because these are properties
    of the *run*, not of any target's hold window -- peak momentum does
    not belong to whichever target happened to be active when it
    occurred, and attributing it to one would be an invented finding.

    Everything here is a running extremum or count rather than a value
    carried from the last sample, so a dropped or late telemetry message
    degrades the result rather than corrupting it.
    """

    momentum_capacity_Nms: float
    #: Parameterised rather than hardcoded, matching CaptureTracker --
    #: the Stateflow mode name is the chart's to define, and two places
    #: spelling it differently would make SAFE detectable in one of them.
    safe_mode_name: str = "SAFE"
    peak_momentum_fraction: List[float] = field(
        default_factory=lambda: [0.0, 0.0, 0.0])
    wheel_saturation_events: int = 0
    entered_safe_mode: bool = False
    wheel_energy_J: float = 0.0
    _saturated_previously: bool = False
    #: The largest TwinStatus.saturation_events seen.
    _twin_saturation_events: int = 0

    def observe(self, sample: TelemetrySample) -> None:
        for axis in range(3):
            fraction = abs(sample.wheel_momentum_Nms[axis]) \
                / self.momentum_capacity_Nms
            self.peak_momentum_fraction[axis] = max(
                self.peak_momentum_fraction[axis], fraction)

        # Count observed rising edges of the aggregate actuator-limited flag.
        # It includes torque clipping, momentum blocking and wheel failure;
        # it is NOT a count of individual wheels or momentum-capacity hits.
        # Continuous saturation is one episode. Lost samples can merge or
        # hide episodes, so this is a telemetry-observed count, not fault truth.
        if sample.wheels_saturated and not self._saturated_previously:
            self.wheel_saturation_events += 1
        self._saturated_previously = sample.wheels_saturated
        # The twin's own count also sees episodes that began and ended
        # between two published samples (free-run sub-steps); in
        # lockstep it equals the observed rising edges.
        if sample.saturation_events is not None:
            self._twin_saturation_events = max(
                self._twin_saturation_events, int(sample.saturation_events))

        if sample.mode == self.safe_mode_name:
            self.entered_safe_mode = True

        # The twin publishes a cumulative integral, so the maximum seen
        # is the total. Taking the last sample's value instead would let
        # one late-arriving stale message under-report the whole run.
        self.wheel_energy_J = max(self.wheel_energy_J, sample.wheel_energy_J)

    def apply_to(self, result) -> None:
        """Write the accumulated quantities onto a :class:`RunResult`."""
        result.peak_momentum_fraction = (
            self.peak_momentum_fraction[0],
            self.peak_momentum_fraction[1],
            self.peak_momentum_fraction[2])
        result.wheel_saturation_events = max(self.wheel_saturation_events,
                                             self._twin_saturation_events)
        result.entered_safe_mode = self.entered_safe_mode
        result.wheel_energy_J = self.wheel_energy_J


@dataclass
class LatencyAggregate:
    """E6's latency breakdown, accumulated from the bridge's own timing.

    Each ``BridgeStatus`` sample already carries a MEAN over its own 1 Hz
    window (``stk_bridge.core.step_timing``); this takes the mean of
    those per-window means, weighted by how many step-loop iterations
    fed each window. Weighting matters because a window can be short --
    the first status published after a run starts may cover only one or
    two steps -- and an unweighted average would let that thin window
    count as much as a full second's worth of steady-state timing.

    Separate from :class:`RunAggregate`: that one is driven by the
    TWIN's telemetry (``TwinStatus``), this one by the BRIDGE's
    (``BridgeStatus``). Different message, different publisher, no
    reason to force them into one accumulator.
    """

    sense_ms_weighted: float = 0.0
    actuate_ms_weighted: float = 0.0
    barrier_wait_ms_weighted: float = 0.0
    weight: int = 0

    def observe(self, mean_sense_ms: float, mean_actuate_ms: float,
                mean_barrier_wait_ms: float, step_samples: int) -> None:
        """One ``BridgeStatus`` sample's timing window.

        A window with ``step_samples == 0`` is skipped rather than
        averaged in at weight zero: it carries no information (the
        bridge had nothing to time yet), and NaN-guarding a zero-weight
        contribution everywhere it could appear is more code than just
        not observing it.
        """
        if step_samples <= 0:
            return
        self.sense_ms_weighted += mean_sense_ms * step_samples
        self.actuate_ms_weighted += mean_actuate_ms * step_samples
        self.barrier_wait_ms_weighted += mean_barrier_wait_ms * step_samples
        self.weight += step_samples

    def apply_to(self, result) -> None:
        """Write the weighted means onto a :class:`RunResult`.

        Leaves the result's defaults (NaN) untouched if nothing was ever
        observed, rather than writing 0.0 -- a run where the bridge
        never reported timing is "unmeasured", not "instant".
        """
        if self.weight == 0:
            return
        result.mean_sense_latency_ms = self.sense_ms_weighted / self.weight
        result.mean_actuate_latency_ms = (
            self.actuate_ms_weighted / self.weight)
        result.mean_barrier_wait_ms = (
            self.barrier_wait_ms_weighted / self.weight)


def declined_result(entry: DeclinedEntry) -> TargetResult:
    """The trivial outcome for a target the scheduler never attempted."""
    return TargetResult(
        target_id=entry.target_id, attempted=False,
        outcome=CaptureOutcome.NOT_ATTEMPTED,
        image_time_s=entry.target.t_image_s,
    )


def no_telemetry_result(entry, reason: str) -> TargetResult:
    """An attempted target with no twin telemetry in its hold window."""
    return TargetResult(
        target_id=entry.target_id, attempted=True,
        outcome=CaptureOutcome.NO_TELEMETRY,
        image_time_s=entry.t_image_s, reason=reason,
    )


def access_unverified_result(entry, reason: str) -> TargetResult:
    """An attempted target whose image epoch the attitude never reached."""
    return TargetResult(
        target_id=entry.target_id, attempted=True,
        outcome=CaptureOutcome.ACCESS_UNVERIFIED,
        image_time_s=entry.t_image_s, reason=reason,
    )


def no_access_answer_result(entry, reason: str) -> TargetResult:
    """An attempted target whose access answer never arrived."""
    return TargetResult(
        target_id=entry.target_id, attempted=True,
        outcome=CaptureOutcome.NO_ACCESS_ANSWER,
        image_time_s=entry.t_image_s, reason=reason,
    )


class CaptureTracker:
    """Buffers telemetry and finalises capture outcomes as targets pass.

    Usage: call :meth:`observe` on every incoming telemetry sample (in
    non-decreasing time order), and once a target's image epoch has
    elapsed and its access answer is known, call :meth:`finalize`. The
    buffer retains samples for ``retention_s`` seconds behind the newest
    observed time, which must cover at least the longest
    ``settle_hold_s`` any target in the run uses plus whatever delay the
    caller expects before it can supply an access answer.
    """

    def __init__(self, capture_error_deg: float, settle_hold_s: float,
                retention_s: float, safe_mode_name: str = "SAFE"):
        if retention_s < settle_hold_s:
            raise ValueError(
                f"retention_s ({retention_s}) is shorter than "
                f"settle_hold_s ({settle_hold_s}); samples needed to "
                f"score a target's hold window would already have been "
                f"discarded by the time finalize() is called for it"
            )
        self._capture_error_deg = capture_error_deg
        self._settle_hold_s = settle_hold_s
        self._retention_s = retention_s
        self._safe_mode_name = safe_mode_name
        self._buffer: Deque[TelemetrySample] = deque()

    def observe(self, sample: TelemetrySample) -> None:
        """Record one telemetry sample and trim the buffer behind it."""
        if self._buffer and sample.time_s < self._buffer[-1].time_s:
            raise ValueError(
                f"telemetry sample at t={sample.time_s:.3f}s arrived "
                f"after one at t={self._buffer[-1].time_s:.3f}s; "
                f"CaptureTracker requires non-decreasing time order, the "
                f"same requirement STK itself imposes on injected "
                f"real-time samples"
            )
        self._buffer.append(sample)
        cutoff = sample.time_s - self._retention_s
        while self._buffer and self._buffer[0].time_s < cutoff:
            self._buffer.popleft()

    def window(self, image_time_s: float) -> List[TelemetrySample]:
        """Buffered samples inside
        ``[image_time_s - hold_s, image_time_s]``.
        """
        start = image_time_s - self._settle_hold_s
        return [s for s in self._buffer
                if start - 1e-9 <= s.time_s <= image_time_s + 1e-9]

    def finalize(self, entry: ScheduleEntry, has_access: bool) -> TargetResult:
        """Apply the capture criterion to an attempted target.

        Raises the same ``ValueError`` :func:`evaluate_capture` would if
        no telemetry fell inside the hold window -- a harness bug (too
        short a retention, or a scheduler that never actually pointed at
        this target) must not be scored as a physics result.
        """
        samples = [
            PointingSample(time_s=s.time_s, error_deg=s.error_deg,
                          has_access=has_access, mode=s.mode,
                          max_error_since_last_deg=s.max_error_since_last_deg)
            for s in self.window(entry.t_image_s)
        ]
        outcome, error_at_image, max_error = evaluate_capture(
            samples, entry.t_image_s, self._capture_error_deg,
            self._settle_hold_s, self._safe_mode_name)

        return TargetResult(
            target_id=entry.target_id, attempted=True, outcome=outcome,
            image_time_s=entry.t_image_s,
            error_at_image_deg=error_at_image,
            max_error_in_hold_deg=max_error,
            slew_magnitude_deg=float("nan"), slew_duration_s=float("nan"),
        )
