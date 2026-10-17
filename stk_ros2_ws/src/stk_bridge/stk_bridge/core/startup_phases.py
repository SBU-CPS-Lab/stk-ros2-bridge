"""Where the bridge's startup time goes, recorded as it happens.

Launch to CONNECTED took 43.7-76.1 s over 40 runs, and one run (P10
live session, 3186642) was killed by the runner's timeout while its
log was still inside convention validation. bridge.log only had
timestamps, from which the phases had to be rebuilt by hand.

StartupPhases times each phase (nested phases are allowed, for the
validation checks), logs each duration, and rewrites its JSON file at
every change, so a bridge killed part-way through still says which
phase it was in and since when. ``on_change`` is called with the
current (innermost open) phase at every change; the bridge publishes
BridgeStatus STARTING from it. The runner reads the file into the
run's summary.json.

File shape::

    {"started_at": "...", "current": "validation/quaternion_ordering",
     "current_since": "...", "last_phase": "...", "finished": false,
     "outcome": "", "error": "",
     "phases": [{"phase": "connect", "started_at": "...",
                 "ended_at": "...", "s": 1.5, "ok": true}, ...]}

Phases are listed in the order they ended; times are local wall time.
"""

from __future__ import annotations

import contextlib
import json
import logging
import os
import threading
import time
from typing import Callable, Dict, List, Optional

LOGGER = logging.getLogger(__name__)


def _stamp() -> str:
    now = time.time()
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(now)) + \
        f".{int(now * 1000) % 1000:03d}"


class StartupPhases:
    def __init__(self, path: str = "",
                 clock: Callable[[], float] = time.monotonic,
                 on_change: Optional[Callable[[str], None]] = None):
        self._path = path
        self._clock = clock
        self._on_change = on_change
        self._lock = threading.Lock()
        self._open: List[Dict] = []          # (name, start, started_at)
        self._done: List[Dict] = []
        self._last = ""
        self._state = {"started_at": _stamp(), "finished": False,
                       "outcome": "", "error": ""}

    @property
    def current(self) -> str:
        with self._lock:
            return self._open[-1]["phase"] if self._open else ""

    @property
    def finished(self) -> bool:
        with self._lock:
            return bool(self._state["finished"])

    @contextlib.contextmanager
    def phase(self, name: str):
        """Time ``name``; it fails (ok false) if the body raises."""
        self._begin(name)
        ok = False
        try:
            yield
            ok = True
        finally:
            self._end(name, ok)

    def finish(self, outcome: str, error: str = "") -> None:
        """Startup is over: "connected", or "failed" with the error."""
        with self._lock:
            self._state.update(finished=True, outcome=outcome, error=error)
        self._changed()

    def durations(self) -> Dict[str, float]:
        with self._lock:
            return {entry["phase"]: entry["s"] for entry in self._done}

    def to_dict(self) -> Dict:
        with self._lock:
            current = self._open[-1] if self._open else None
            return dict(self._state,
                        current=current["phase"] if current else "",
                        current_since=(current["started_at"]
                                       if current else ""),
                        last_phase=self._last,
                        phases=[dict(entry) for entry in self._done])

    def _begin(self, name: str) -> None:
        with self._lock:
            self._open.append({"phase": name, "start": self._clock(),
                               "started_at": _stamp()})
            self._last = name
        LOGGER.info("startup phase %s started", name)
        self._changed()

    def _end(self, name: str, ok: bool) -> None:
        with self._lock:
            entry = self._open.pop()
            seconds = round(self._clock() - entry["start"], 3)
            self._done.append({"phase": name,
                               "started_at": entry["started_at"],
                               "ended_at": _stamp(), "s": seconds,
                               "ok": ok})
        (LOGGER.info if ok else LOGGER.error)(
            "startup phase %s: %.1f s%s", name, seconds,
            "" if ok else " (failed)")
        self._changed()

    def _changed(self) -> None:
        state = self.to_dict()
        if self._path:
            try:
                temporary = f"{self._path}.partial"
                with open(temporary, "w", encoding="utf-8") as handle:
                    json.dump(state, handle, indent=2)
                    handle.write("\n")
                os.replace(temporary, self._path)
            except OSError:
                LOGGER.warning("could not write %s", self._path,
                               exc_info=True)
        if self._on_change is not None:
            try:
                self._on_change(state["current"])
            except Exception:           # noqa: BLE001 - startup goes on
                LOGGER.warning("startup phase announcement failed",
                               exc_info=True)


@contextlib.contextmanager
def maybe_phase(phases: Optional[StartupPhases], name: str):
    """``phases.phase(name)``, or nothing when there is no recorder."""
    if phases is None:
        yield
    else:
        with phases.phase(name):
            yield
