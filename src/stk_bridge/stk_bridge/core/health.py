"""Bridge health state machine and reconnection policy.

Aggregates signals from the backend worker, sensing pipeline, actuation
dispatcher, and time sync into the single ``BridgeStatus.state`` a
consumer can act on, and decides when to attempt reconnection.

The distinction that matters
----------------------------
``DEGRADED`` and ``DISCONNECTED`` are deliberately separate. A bridge
that has lost STK entirely is useless and everyone should stop. A bridge
whose link-budget channel is erroring but whose attitude channel is
healthy is still perfectly good for ADCS work, and forcing a controller
to stop for that would be worse than the fault. Only channels a consumer
actually depends on should escalate, so criticality is declared per
channel rather than inferred.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Callable, Dict, List, Optional

LOGGER = logging.getLogger(__name__)


class BridgeState(IntEnum):
    """Mirrors the constants in ``stk_msgs/BridgeStatus``."""

    STARTING = 0
    CONNECTED = 1
    DEGRADED = 2
    DISCONNECTED = 3
    ERROR = 4


@dataclass
class HealthConfig:
    #: Channels whose failure escalates to DEGRADED. Everything else is
    #: reported but does not change the bridge state.
    critical_channels: List[str] = field(
        default_factory=lambda: ["orbit_position", "attitude"])
    #: Consecutive stale reads on a critical channel before escalating.
    #: Not one: a single stale read at the edge of a real-time object's
    #: Look-Ahead window is normal and flapping the state on it would
    #: make the status signal useless.
    stale_threshold: int = 5
    #: Seconds without a successful backend call before DISCONNECTED.
    heartbeat_timeout_s: float = 10.0
    reconnect_enabled: bool = True
    reconnect_initial_delay_s: float = 1.0
    reconnect_max_delay_s: float = 60.0
    reconnect_max_attempts: int = 0   # 0 = unlimited

    @classmethod
    def from_config(cls, config: Optional[Dict] = None) -> "HealthConfig":
        section = (config or {}).get("health", {}) or {}
        return cls(
            critical_channels=list(section.get("critical_channels")
                                   or ["orbit_position", "attitude"]),
            stale_threshold=int(section.get("stale_threshold", 5)),
            heartbeat_timeout_s=float(
                section.get("heartbeat_timeout_s", 10.0)),
            reconnect_enabled=bool(section.get("reconnect_enabled", True)),
            reconnect_initial_delay_s=float(
                section.get("reconnect_initial_delay_s", 1.0)),
            reconnect_max_delay_s=float(
                section.get("reconnect_max_delay_s", 60.0)),
            reconnect_max_attempts=int(
                section.get("reconnect_max_attempts", 0)),
        )


class HealthMonitor:
    """Tracks bridge health and drives reconnection."""

    def __init__(self, config: HealthConfig):
        self._config = config
        self._state = BridgeState.STARTING
        self._last_success = time.monotonic()
        self._last_error = ""
        self._consecutive_stale: Dict[str, int] = {}
        self._reconnect_attempts = 0
        self._next_reconnect_at = 0.0
        self._state_changes = 0

    # -- properties --------------------------------------------------------

    @property
    def state(self) -> BridgeState:
        return self._state

    @property
    def last_error(self) -> str:
        return self._last_error

    @property
    def reconnect_attempts(self) -> int:
        return self._reconnect_attempts

    # -- signals -----------------------------------------------------------

    def record_success(self) -> None:
        self._last_success = time.monotonic()
        self._reconnect_attempts = 0
        if self._state in (BridgeState.STARTING, BridgeState.DISCONNECTED):
            self._set_state(BridgeState.CONNECTED)

    def record_error(self, message: str, fatal: bool = False) -> None:
        self._last_error = message
        if fatal:
            self._set_state(BridgeState.ERROR)

    def record_disconnect(self, message: str) -> None:
        self._last_error = message
        self._set_state(BridgeState.DISCONNECTED)
        if self._next_reconnect_at == 0.0:
            self._next_reconnect_at = (time.monotonic()
                                       + self._config.reconnect_initial_delay_s)

    def update_from_sensing(self, statistics: Dict[str, Dict]) -> None:
        """Fold sensing channel statistics into the health state."""
        escalate = False
        for name, stats in statistics.items():
            stale = int(stats.get("stale_reads", 0))
            errors = int(stats.get("errors", 0))
            previous = self._consecutive_stale.get(name, 0)

            if stale + errors > previous:
                self._consecutive_stale[name] = stale + errors
            else:
                self._consecutive_stale[name] = 0

            if name not in self._config.critical_channels:
                continue
            if self._consecutive_stale[name] >= self._config.stale_threshold:
                escalate = True
                self._last_error = (
                    f"critical channel {name!r} stale/erroring "
                    f"({self._consecutive_stale[name]} consecutive): "
                    f"{stats.get('last_error', '')}"
                )

        if self._state == BridgeState.CONNECTED and escalate:
            self._set_state(BridgeState.DEGRADED)
        elif self._state == BridgeState.DEGRADED and not escalate:
            self._set_state(BridgeState.CONNECTED)

    def check_heartbeat(self) -> bool:
        """Returns True if the backend is considered alive."""
        if self._state in (BridgeState.DISCONNECTED, BridgeState.ERROR):
            return False
        age = time.monotonic() - self._last_success
        if age > self._config.heartbeat_timeout_s:
            self.record_disconnect(
                f"no successful backend call in {age:.1f}s "
                f"(timeout {self._config.heartbeat_timeout_s:.1f}s)"
            )
            return False
        return True

    # -- reconnection ------------------------------------------------------

    def should_attempt_reconnect(self) -> bool:
        if not self._config.reconnect_enabled:
            return False
        if self._state != BridgeState.DISCONNECTED:
            return False
        if (self._config.reconnect_max_attempts
                and self._reconnect_attempts >= self._config.reconnect_max_attempts):
            return False
        return time.monotonic() >= self._next_reconnect_at

    def record_reconnect_attempt(self, succeeded: bool) -> None:
        """Note an attempt and schedule the next with exponential backoff.

        Backoff matters because the common cause of a lost connection is
        STK having been closed by the operator, and hammering COM
        activation once a second will relaunch it unexpectedly or spam
        the log for hours.
        """
        self._reconnect_attempts += 1
        if succeeded:
            self._next_reconnect_at = 0.0
            self.record_success()
            LOGGER.info("backend reconnected after %d attempt(s)",
                        self._reconnect_attempts)
            return

        delay = min(
            self._config.reconnect_initial_delay_s * (2 ** min(
                self._reconnect_attempts - 1, 16)),
            self._config.reconnect_max_delay_s,
        )
        self._next_reconnect_at = time.monotonic() + delay
        LOGGER.warning("reconnect attempt %d failed; next in %.1fs",
                       self._reconnect_attempts, delay)

    # -- internals ---------------------------------------------------------

    def _set_state(self, state: BridgeState) -> None:
        if state == self._state:
            return
        LOGGER.info("bridge state: %s -> %s", self._state.name, state.name)
        self._state = state
        self._state_changes += 1

    def statistics(self) -> Dict:
        return {
            "state": self._state.name,
            "state_changes": self._state_changes,
            "seconds_since_success": time.monotonic() - self._last_success,
            "reconnect_attempts": self._reconnect_attempts,
            "last_error": self._last_error,
            "stale_channels": {
                name: count
                for name, count in self._consecutive_stale.items() if count
            },
        }
