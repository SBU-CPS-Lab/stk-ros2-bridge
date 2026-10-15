"""Multi-rate scheduling for the sensing pipeline.

Not every telemetry channel deserves the same rate. Attitude and body
rate drive the control loop and need the fast group; access intervals and
link budgets change on orbital timescales and querying them at 10 Hz
wastes the entire latency budget on data nobody reads. Rate groups make
that a configuration decision rather than a code change.

Deterministic by design: due-ness is computed from the *simulation*
clock, not wall time, so a lockstep run produces byte-identical sampling
regardless of how fast the host executes it. That is what makes a
recorded run reproducible.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Callable, Dict, Iterable, List, Optional

LOGGER = logging.getLogger(__name__)


@dataclass
class RateGroup:
    """A named set of work that runs at a fixed period."""

    name: str
    rate_hz: float
    enabled: bool = True

    #: Simulation time of the last execution. None means never run.
    _last_run_s: Optional[float] = field(default=None, repr=False)
    #: Count of executions, for diagnostics.
    _runs: int = field(default=0, repr=False)
    #: Count of periods skipped because the previous run was still
    #: outstanding or the group overran its budget.
    _overruns: int = field(default=0, repr=False)

    def __post_init__(self) -> None:
        if self.rate_hz <= 0.0:
            raise ValueError(
                f"rate group {self.name!r}: rate_hz must be > 0, got "
                f"{self.rate_hz}. Use enabled=False to disable a group "
                f"rather than a zero rate."
            )

    @property
    def period_s(self) -> float:
        return 1.0 / self.rate_hz

    @property
    def runs(self) -> int:
        return self._runs

    @property
    def overruns(self) -> int:
        return self._overruns

    def is_due(self, sim_time_s: float) -> bool:
        if not self.enabled:
            return False
        if self._last_run_s is None:
            return True
        return (sim_time_s - self._last_run_s) >= self.period_s - 1e-9

    def periods_elapsed(self, sim_time_s: float) -> int:
        """How many periods have passed since the last run.

        More than one means the loop fell behind. Reported rather than
        silently absorbed, because a sensing group that keeps missing
        periods is the earliest observable symptom of an STK query that
        has become too slow for the configured rate.
        """
        if self._last_run_s is None:
            return 1
        return max(0, int((sim_time_s - self._last_run_s) / self.period_s))

    def mark_run(self, sim_time_s: float) -> None:
        if self._last_run_s is not None:
            missed = self.periods_elapsed(sim_time_s) - 1
            if missed > 0:
                self._overruns += missed
        self._last_run_s = sim_time_s
        self._runs += 1

    def reset(self) -> None:
        self._last_run_s = None
        self._runs = 0
        self._overruns = 0


class RateGroupScheduler:
    """Owns a set of rate groups and the callbacks bound to them."""

    def __init__(self) -> None:
        self._groups: Dict[str, RateGroup] = {}
        self._callbacks: Dict[str, List[Callable[[float], None]]] = {}

    # -- construction ------------------------------------------------------

    @classmethod
    def from_config(cls, config: Optional[Dict] = None
                    ) -> "RateGroupScheduler":
        """Build from a ``sensing_pipeline.yaml`` mapping.

        Expected shape::

            sensing:
              rate_groups:
                fast:    {rate_hz: 10.0}
                medium:  {rate_hz: 1.0}
                slow:    {rate_hz: 0.1}
                on_demand: {rate_hz: 0.01, enabled: false}
        """
        scheduler = cls()
        sensing = (config or {}).get("sensing", {}) or {}
        groups = sensing.get("rate_groups") or {}

        if not groups:
            groups = {
                "fast": {"rate_hz": 10.0},
                "medium": {"rate_hz": 1.0},
                "slow": {"rate_hz": 0.1},
            }
            LOGGER.info("no rate_groups configured; using defaults "
                        "fast=10Hz medium=1Hz slow=0.1Hz")

        for name, spec in groups.items():
            scheduler.add_group(RateGroup(
                name=str(name),
                rate_hz=float((spec or {}).get("rate_hz", 1.0)),
                enabled=bool((spec or {}).get("enabled", True)),
            ))
        return scheduler

    def add_group(self, group: RateGroup) -> None:
        self._groups[group.name] = group
        self._callbacks.setdefault(group.name, [])

    def subscribe(self, group_name: str,
                  callback: Callable[[float], None]) -> None:
        """Bind a callback to a group.

        Raises immediately on an unknown group rather than creating one,
        because a typo'd group name that silently created a new group
        would produce work that never runs at the intended rate and
        nothing would indicate why.
        """
        if group_name not in self._groups:
            raise KeyError(
                f"unknown rate group {group_name!r}; configured groups are "
                f"{sorted(self._groups)}"
            )
        self._callbacks[group_name].append(callback)

    # -- execution ---------------------------------------------------------

    def due_groups(self, sim_time_s: float) -> List[RateGroup]:
        """Groups due at this simulation time, fastest first.

        Ordering matters when the loop is behind: the fast group carries
        the control-relevant data, so it runs before slower groups get a
        chance to consume the remaining time.
        """
        due = [g for g in self._groups.values() if g.is_due(sim_time_s)]
        return sorted(due, key=lambda g: -g.rate_hz)

    def run_due(self, sim_time_s: float,
                on_error: Optional[Callable[[str, Exception], None]] = None
                ) -> int:
        """Execute every due group's callbacks. Returns groups run.

        A callback that raises does not prevent other groups from
        running: one failing telemetry channel should degrade that
        channel, not stop the bridge. Errors go to ``on_error`` (which
        ``bridge_node`` routes into ``BridgeStatus.last_error``) or to
        the log.
        """
        count = 0
        for group in self.due_groups(sim_time_s):
            for callback in self._callbacks.get(group.name, ()):
                try:
                    callback(sim_time_s)
                except Exception as exc:  # noqa: BLE001 - isolate channels
                    if on_error is not None:
                        on_error(group.name, exc)
                    else:
                        LOGGER.exception("rate group %r callback failed",
                                         group.name)
            group.mark_run(sim_time_s)
            count += 1
        return count

    def fastest_period_s(self) -> Optional[float]:
        """The shortest period among enabled groups, or None."""
        periods = [g.period_s for g in self._groups.values() if g.enabled]
        return min(periods) if periods else None

    def next_due_time(self, sim_time_s: float) -> Optional[float]:
        """Earliest simulation time at which any group next runs.

        Lets a free-running loop sleep exactly as long as it should
        instead of spinning on a fixed tick.
        """
        candidates = []
        for group in self._groups.values():
            if not group.enabled:
                continue
            if group._last_run_s is None:
                return sim_time_s
            candidates.append(group._last_run_s + group.period_s)
        return min(candidates) if candidates else None

    # -- introspection -----------------------------------------------------

    def group(self, name: str) -> RateGroup:
        return self._groups[name]

    def groups(self) -> Iterable[RateGroup]:
        return self._groups.values()

    def reset(self) -> None:
        for group in self._groups.values():
            group.reset()

    def statistics(self) -> Dict[str, Dict[str, float]]:
        return {
            group.name: {
                "rate_hz": group.rate_hz,
                "runs": group.runs,
                "overruns": group.overruns,
                "enabled": group.enabled,
            }
            for group in self._groups.values()
        }

    def __repr__(self) -> str:
        summary = ", ".join(f"{g.name}@{g.rate_hz}Hz"
                            for g in self._groups.values())
        return f"<RateGroupScheduler {summary}>"
