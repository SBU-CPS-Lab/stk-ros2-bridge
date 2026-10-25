"""Per-phase wall-clock timing of the step loop.

The bridge reports where a step's time goes: sensing, actuation and the
lockstep barrier wait. ``nodes/bridge_node.py`` times each phase with
:meth:`PhaseTimings.timed` and reads :meth:`PhaseTimings.summary` when it
publishes ``BridgeStatus``. The arithmetic lives in ``core/`` so that it
can be tested without ROS.

Windowed, not cumulative
------------------------
Durations accumulate since the last :meth:`reset` rather than since the
bridge started. ``BridgeStatus`` is published at a fixed 1 Hz regardless
of execution mode, so each summary is "the last second's worth of
steps", comparable across a run and not dominated by one slow startup
iteration.

The barrier-wait sample lags the phase it describes by up to one
publish window: in lockstep, ``await_barrier`` happens *after*
``_publish_status`` within an iteration (see ``bridge_node._
lockstep_iteration``), so a given iteration's wait time is folded into
the *next* window's summary. This does not affect a mean over a run.
"""

from __future__ import annotations

import time
from contextlib import contextmanager
from dataclasses import dataclass, field
from typing import Dict, Iterator, List

#: The timed phases. The names are also the message field prefixes
#: (``mean_<phase>_ms``).
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

        A phase without samples reports 0.0 rather than NaN, so the
        message fields stay valid; ``step_samples`` tells a consumer
        whether zero means "fast" or "not measured" (free-run has no
        barrier wait).
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
