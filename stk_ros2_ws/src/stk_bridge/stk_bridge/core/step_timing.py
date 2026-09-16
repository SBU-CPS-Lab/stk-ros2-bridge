"""Per-phase wall-clock timing for the step loop -- what E6 measures.

C1 section 8 commits the paper to a transparent latency breakdown rather
than an implied number: not just "the loop does N steps/second" but
which of sense, actuate, and the lockstep barrier wait accounts for it.
That breakdown has to be measured on the machine that actually runs
against STK, since none of the three phases can be timed meaningfully
against the mock.

Kept here, in ``core/``, rather than inline in ``nodes/bridge_node.py``,
for the same reason every other piece of step-loop logic is: it is pure
arithmetic over wall-clock durations, so it can be tested without a
sourced ROS 2 environment. ``bridge_node.py`` only calls
:meth:`PhaseTimings.record` around each phase and reads
:meth:`PhaseTimings.summary` when it publishes status.

Windowed, not cumulative
------------------------
Durations accumulate since the last :meth:`reset` rather than since the
bridge started. ``BridgeStatus`` is published at a fixed 1 Hz regardless
of execution mode, so each summary is "the last second's worth of
steps" -- comparable across a run, and not dominated by one slow
startup iteration the way a lifetime average would be after an hour.

The barrier-wait sample lags the phase it describes by up to one
publish window: in lockstep, ``await_barrier`` happens *after*
``_publish_status`` within an iteration (see ``bridge_node._
lockstep_iteration``), so a given iteration's wait time is folded into
the *next* window's summary. Immaterial for a mean over many iterations
in a multi-minute run, which is what E6 reports.
"""

from __future__ import annotations

import time
from contextlib import contextmanager
from dataclasses import dataclass, field
from typing import Dict, Iterator, List

#: The three phases E6's breakdown names. A string key rather than an
#: enum: this is a thin, internal accumulator and the phase names are
#: also the message field prefixes (``mean_<phase>_ms``), so keeping
#: them as the plain strings used at both ends avoids a translation
#: table that could drift.
PHASES = ("sense", "actuate", "barrier_wait")


@dataclass
class PhaseTimings:
    """Wall-clock durations for each phase, since the last reset."""

    _samples: Dict[str, List[float]] = field(
        default_factory=lambda: {phase: [] for phase in PHASES})

    def record(self, phase: str, duration_ms: float) -> None:
        if phase not in self._samples:
            raise ValueError(
                f"unknown phase {phase!r}; expected one of {PHASES}")
        self._samples[phase].append(duration_ms)

    @contextmanager
    def timed(self, phase: str) -> Iterator[None]:
        """``with timings.timed("sense"): ...`` -- times the block."""
        start = time.perf_counter()
        try:
            yield
        finally:
            self.record(phase, (time.perf_counter() - start) * 1000.0)

    def summary(self) -> Dict[str, float]:
        """Mean duration per phase, ms, and how many samples fed it.

        Zero rather than NaN for a phase with no samples yet -- the
        message field is always a valid float, and ``step_samples``
        tells a consumer whether zero means "fast" or "not measured
        yet". Choosing NaN here would make the message itself invalid
        for a phase this run never happened to exercise (free-run has
        no barrier wait at all).
        """
        out: Dict[str, float] = {}
        for phase in PHASES:
            values = self._samples[phase]
            out[f"mean_{phase}_ms"] = (
                sum(values) / len(values) if values else 0.0)
        out["step_samples"] = max(len(self._samples[p]) for p in PHASES)
        return out

    def reset(self) -> None:
        for phase in PHASES:
            self._samples[phase] = []
