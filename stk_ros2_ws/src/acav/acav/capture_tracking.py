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
from dataclasses import dataclass
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


def declined_result(entry: DeclinedEntry) -> TargetResult:
    """The trivial outcome for a target the scheduler never attempted."""
    return TargetResult(
        target_id=entry.target_id, attempted=False,
        outcome=CaptureOutcome.NOT_ATTEMPTED,
        image_time_s=entry.target.t_image_s,
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
                          has_access=has_access, mode=s.mode)
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
