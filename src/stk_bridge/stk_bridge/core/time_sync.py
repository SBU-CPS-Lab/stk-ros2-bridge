"""Simulation-time ownership and the lockstep barrier.

The bridge is the clock authority for the whole ROS graph. It advances
STK's scenario time and publishes it on ``/clock`` and on
``ScenarioTime``. Other nodes take their time from these (ROS nodes with
``use_sim_time:=true``; a Simulink model, whose ROS node does not follow
``/clock``, stamps from ``ScenarioTime``), so all timestamps derive from
one source.

Execution modes
---------------

``free_run``
    STK animates on wall time; the bridge samples at its configured
    rates and mirrors wall time onto ``/clock``. Cheapest, least
    reproducible. Fine for visualisation.

``paced``
    Simulation advances at ``real_time_factor`` times wall clock. Useful
    for watching a run at half speed, or for a soak test at 10x.

``lockstep``
    The bridge advances one step, publishes state and ``/clock``, then
    **waits for every declared participant to acknowledge that step**
    before advancing again. Deterministic and reproducible: the same
    inputs produce the same run regardless of host speed. This is the
    default for control work, and the only mode in which a control law's
    behaviour is a property of the model rather than of the machine.

``external``
    Someone else owns ``/clock``; the bridge follows. For federating
    with a third simulator that must be the master.

Why the barrier is explicit
---------------------------
A controller that misses a step in lockstep is a *correctness* failure,
not a performance hiccup -- the plant advanced without the controller
seeing that state. Silently continuing would produce a run that looks
fine and encodes a control law nobody tested. So the barrier names its
participants up front, times out loudly, and applies a configured policy
rather than a default guess.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Callable, Dict, List, Optional, Set

from stk_bridge.backends.base import StkTime
from stk_bridge.exceptions import LockstepBarrierTimeoutError

LOGGER = logging.getLogger(__name__)


class ExecutionMode(IntEnum):
    """Mirrors the constants in ``stk_msgs/ScenarioTime``."""

    FREE_RUN = 0
    PACED = 1
    LOCKSTEP = 2
    EXTERNAL = 3

    @classmethod
    def parse(cls, value) -> "ExecutionMode":
        if isinstance(value, cls):
            return value
        if isinstance(value, int):
            return cls(value)
        key = str(value).strip().upper().replace("-", "_")
        try:
            return cls[key]
        except KeyError:
            raise ValueError(
                f"unknown execution mode {value!r}; valid values are "
                f"{[m.name.lower() for m in cls]}"
            ) from None


class BarrierTimeoutPolicy:
    """What to do when a lockstep participant misses its deadline."""

    #: Advance anyway. The run continues but is no longer deterministic;
    #: logged at ERROR every time, because a silently non-deterministic
    #: lockstep run is worse than an honest free-running one.
    CONTINUE = "continue"
    #: Keep waiting indefinitely. Safe for correctness, and will hang
    #: forever if a participant has died -- pair with a health watchdog.
    WAIT = "wait"
    #: Raise, and let the node decide (default). Fails loudly and early.
    ABORT = "abort"

    ALL = (CONTINUE, WAIT, ABORT)


@dataclass
class TimeSyncConfig:
    mode: ExecutionMode = ExecutionMode.LOCKSTEP
    step_s: float = 0.1
    real_time_factor: float = 1.0
    barrier_participants: List[str] = field(default_factory=list)
    barrier_timeout_s: float = 5.0
    on_barrier_timeout: str = BarrierTimeoutPolicy.ABORT
    #: Publish /clock even in EXTERNAL mode (normally False -- two clock
    #: publishers on one graph is a classic, hard-to-diagnose fault).
    publish_clock: bool = True

    @classmethod
    def from_config(cls, config: Optional[Dict] = None) -> "TimeSyncConfig":
        section = (config or {}).get("time", {}) or {}
        policy = str(section.get("on_barrier_timeout",
                                 BarrierTimeoutPolicy.ABORT)).lower()
        if policy not in BarrierTimeoutPolicy.ALL:
            raise ValueError(
                f"unknown on_barrier_timeout {policy!r}; valid values are "
                f"{list(BarrierTimeoutPolicy.ALL)}"
            )
        mode = ExecutionMode.parse(section.get("mode", "lockstep"))
        config_obj = cls(
            mode=mode,
            step_s=float(section.get("step_s", 0.1)),
            real_time_factor=float(section.get("real_time_factor", 1.0)),
            barrier_participants=list(
                section.get("barrier_participants", []) or []),
            barrier_timeout_s=float(section.get("barrier_timeout_s", 5.0)),
            on_barrier_timeout=policy,
            publish_clock=bool(section.get("publish_clock",
                                           mode is not ExecutionMode.EXTERNAL)),
        )
        if config_obj.step_s <= 0.0:
            raise ValueError(f"step_s must be > 0, got {config_obj.step_s}")
        if config_obj.real_time_factor <= 0.0:
            raise ValueError("real_time_factor must be > 0")
        if (mode is ExecutionMode.LOCKSTEP
                and not config_obj.barrier_participants):
            LOGGER.warning(
                "lockstep mode with NO barrier participants declared. The "
                "bridge will free-wheel through steps without waiting for "
                "anyone, which is free_run with extra latency. Declare the "
                "controller node in time.barrier_participants to get the "
                "determinism lockstep is for."
            )
        return config_obj


class LockstepBarrier:
    """Tracks per-step acknowledgements from declared participants."""

    def __init__(self, participants: List[str], timeout_s: float = 5.0):
        self._participants: Set[str] = set(participants)
        self._timeout_s = timeout_s
        self._lock = threading.Condition()
        self._step_index = 0
        self._acked: Set[str] = set()
        self._last_accepted: Dict[str, int] = {}
        self._late_acks = 0
        self._timeouts = 0

    @property
    def participants(self) -> Set[str]:
        return set(self._participants)

    @property
    def late_acks(self) -> int:
        return self._late_acks

    @property
    def timeouts(self) -> int:
        return self._timeouts

    def add_participant(self, name: str) -> None:
        with self._lock:
            self._participants.add(name)
            LOGGER.info("lockstep participant registered: %s", name)

    def remove_participant(self, name: str) -> None:
        with self._lock:
            self._participants.discard(name)
            self._acked.discard(name)
            self._lock.notify_all()

    def begin_step(self, step_index: int) -> None:
        with self._lock:
            self._step_index = step_index
            self._acked.clear()
            self._lock.notify_all()

    def acknowledge(self, name: str, step_index: int) -> bool:
        """Record an ack. Returns True if it counted.

        An ack for a step other than the current one is *not* silently
        accepted: it means the participant is behind (or ahead), which is
        precisely the condition lockstep exists to detect. Counting it
        would let a lagging controller appear synchronised.
        """
        with self._lock:
            if step_index != self._step_index:
                if (step_index < self._step_index
                        and self._last_accepted.get(name) == step_index):
                    LOGGER.debug("ignoring duplicate ack from %r for completed step %d",
                                 name, step_index)
                    return False
                self._late_acks += 1
                LOGGER.warning(
                    "ack from %r is for step %d but the current step is %d; "
                    "ignoring unmatched acknowledgement.",
                    name, step_index, self._step_index,
                )
                return False
            if name not in self._participants:
                LOGGER.debug("ack from unregistered participant %r", name)
                return False
            self._acked.add(name)
            self._last_accepted[name] = step_index
            self._lock.notify_all()
            return True

    def missing(self) -> List[str]:
        with self._lock:
            return sorted(self._participants - self._acked)

    def wait(self, timeout_s: Optional[float] = None) -> List[str]:
        """Block until all participants ack. Returns those still missing."""
        timeout = timeout_s if timeout_s is not None else self._timeout_s
        deadline = time.monotonic() + timeout
        with self._lock:
            while self._participants - self._acked:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    self._timeouts += 1
                    return sorted(self._participants - self._acked)
                self._lock.wait(timeout=remaining)
            return []


class TimeSync:
    """Owns simulation time and drives STK's scenario clock."""

    def __init__(self, config: TimeSyncConfig,
                 advance_backend: Optional[Callable[[StkTime], None]] = None):
        self._config = config
        self._advance_backend = advance_backend
        self._barrier = LockstepBarrier(config.barrier_participants,
                                        config.barrier_timeout_s)

        self._epoch: Optional[StkTime] = None
        self._sim_time_s = 0.0
        self._step_index = 0
        self._paused = False
        self._wall_anchor = 0.0
        self._sim_anchor = 0.0
        self._lock = threading.RLock()

    # -- properties --------------------------------------------------------

    @property
    def config(self) -> TimeSyncConfig:
        return self._config

    @property
    def mode(self) -> ExecutionMode:
        return self._config.mode

    @property
    def barrier(self) -> LockstepBarrier:
        return self._barrier

    @property
    def step_index(self) -> int:
        return self._step_index

    @property
    def paused(self) -> bool:
        return self._paused

    @property
    def epoch(self) -> StkTime:
        if self._epoch is None:
            raise RuntimeError("TimeSync.start() has not been called")
        return self._epoch

    def now(self) -> StkTime:
        """Current simulation time.

        In free-running and paced modes this is derived from wall time
        rather than a counter, so a slow tick does not accumulate drift
        against the wall clock the operator is watching.
        """
        with self._lock:
            if self._epoch is None:
                raise RuntimeError("TimeSync.start() has not been called")
            if self._config.mode in (ExecutionMode.LOCKSTEP,
                                     ExecutionMode.EXTERNAL):
                return StkTime(self._epoch.epoch_seconds + self._sim_time_s)
            if self._paused:
                return StkTime(self._epoch.epoch_seconds + self._sim_time_s)
            elapsed = time.monotonic() - self._wall_anchor
            sim = self._sim_anchor + elapsed * self._config.real_time_factor
            return StkTime(self._epoch.epoch_seconds + sim)

    def sim_seconds(self) -> float:
        """Seconds since scenario epoch. What rate groups schedule on."""
        return self.now().epoch_seconds - self.epoch.epoch_seconds

    # -- lifecycle ---------------------------------------------------------

    def start(self, epoch: StkTime,
              start_offset_s: float = 0.0, paused: bool = False) -> None:
        """Start the clock; ``paused`` holds it at exactly start_offset_s
        (the free-run start gate) until resume()."""
        with self._lock:
            self._epoch = epoch
            self._sim_time_s = start_offset_s
            self._sim_anchor = start_offset_s
            self._wall_anchor = time.monotonic()
            self._step_index = 0
            self._paused = paused
        LOGGER.info(
            "time sync started: mode=%s epoch=%.3f step=%.4fs rtf=%.3f "
            "participants=%s",
            self._config.mode.name, epoch.epoch_seconds,
            self._config.step_s, self._config.real_time_factor,
            sorted(self._barrier.participants) or "(none)",
        )

    def pause(self) -> None:
        with self._lock:
            if self._paused:
                return
            self._sim_time_s = self.sim_seconds()
            self._sim_anchor = self._sim_time_s
            self._paused = True

    def resume(self) -> None:
        with self._lock:
            if not self._paused:
                return
            self._wall_anchor = time.monotonic()
            self._sim_anchor = self._sim_time_s
            self._paused = False

    def set_mode(self, mode: ExecutionMode,
                 real_time_factor: Optional[float] = None,
                 participants: Optional[List[str]] = None) -> None:
        with self._lock:
            current = self.sim_seconds() if self._epoch else 0.0
            self._config.mode = mode
            if real_time_factor is not None:
                self._config.real_time_factor = real_time_factor
            if participants is not None:
                self._config.barrier_participants = list(participants)
                self._barrier = LockstepBarrier(
                    participants, self._config.barrier_timeout_s)
            self._sim_time_s = current
            self._sim_anchor = current
            self._wall_anchor = time.monotonic()
        LOGGER.info("execution mode set to %s", mode.name)

    def set_external_time(self, when: StkTime) -> None:
        """Adopt a time published by an external clock authority."""
        if self._config.mode is not ExecutionMode.EXTERNAL:
            LOGGER.warning(
                "external time received but mode is %s; ignoring. Two clock "
                "authorities on one graph produce timestamps that jump "
                "backwards.", self._config.mode.name,
            )
            return
        with self._lock:
            self._sim_time_s = when.epoch_seconds - self.epoch.epoch_seconds

    # -- stepping ----------------------------------------------------------

    def step(self, step_s: Optional[float] = None) -> StkTime:
        """Advance one step. Lockstep only.

        Sequence: advance the clock, push it into STK, open the barrier
        for the new step. Callers publish state and ``/clock`` between
        this and :meth:`await_barrier`, so participants have the new
        state in hand before they are asked to acknowledge it.
        """
        if self._config.mode is not ExecutionMode.LOCKSTEP:
            raise RuntimeError(
                f"step() is only valid in lockstep mode; current mode is "
                f"{self._config.mode.name}. Use pause()/resume() to control "
                f"a free-running or paced clock."
            )
        delta = step_s if step_s and step_s > 0.0 else self._config.step_s
        with self._lock:
            self._sim_time_s += delta
            self._step_index += 1
            now = StkTime(self.epoch.epoch_seconds + self._sim_time_s)
            index = self._step_index

        if self._advance_backend is not None:
            self._advance_backend(now)
        self._barrier.begin_step(index)
        return now

    def await_barrier(self, timeout_s: Optional[float] = None) -> List[str]:
        """Block until participants acknowledge the current step.

        Returns the list of participants that did not respond (empty on
        success). Applies the configured timeout policy.
        """
        if self._config.mode is not ExecutionMode.LOCKSTEP:
            return []
        if not self._barrier.participants:
            return []

        missing = self._barrier.wait(timeout_s)
        if not missing:
            return []

        policy = self._config.on_barrier_timeout
        if policy == BarrierTimeoutPolicy.WAIT:
            LOGGER.error(
                "lockstep barrier timeout at step %d; waiting indefinitely "
                "for %s (policy=wait)", self._step_index, missing,
            )
            while missing:
                missing = self._barrier.wait(self._config.barrier_timeout_s)
            return []

        if policy == BarrierTimeoutPolicy.CONTINUE:
            LOGGER.error(
                "lockstep barrier timeout at step %d; advancing WITHOUT "
                "acks from %s. This run is no longer deterministic -- the "
                "plant has advanced past state the controller never saw.",
                self._step_index, missing,
            )
            return missing

        raise LockstepBarrierTimeoutError(self._step_index, missing)

    def acknowledge(self, participant: str, step_index: int) -> bool:
        return self._barrier.acknowledge(participant, step_index)

    def barrier_participants(self) -> List[str]:
        return sorted(self._barrier.participants)

    def missing_acks(self) -> List[str]:
        """Participants that have not acked the current step (step 0
        throughout free-run, where the barrier never advances)."""
        return self._barrier.missing()

    def sleep_until_next(self, next_due_sim_s: Optional[float]) -> None:
        """Sleep so a free-running/paced loop wakes when work is due.

        Sleeping exactly as long as needed, rather than spinning on a
        fixed tick, keeps a 0.1 Hz-only configuration from burning a core.
        """
        if self._config.mode is ExecutionMode.LOCKSTEP:
            return
        if next_due_sim_s is None:
            time.sleep(0.01)
            return
        remaining_sim = next_due_sim_s - self.sim_seconds()
        if remaining_sim <= 0.0:
            return
        wall = remaining_sim / max(self._config.real_time_factor, 1e-9)
        time.sleep(min(wall, 1.0))

    # -- introspection -----------------------------------------------------

    def statistics(self) -> Dict[str, float]:
        return {
            "mode": self._config.mode.name,
            "sim_seconds": self.sim_seconds() if self._epoch else 0.0,
            "step_index": self._step_index,
            "paused": self._paused,
            "real_time_factor": self._config.real_time_factor,
            "barrier_participants": len(self._barrier.participants),
            "barrier_late_acks": self._barrier.late_acks,
            "barrier_timeouts": self._barrier.timeouts,
        }
