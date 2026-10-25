"""Execution time of the STK clock advance, for reporting only.

The bridge submits `Animate * SetValues ... Reset` to the single-thread
COM worker without waiting (nodes/bridge_node.py `_advance_stk_clock`),
so the step's first sensing query queues behind it and E6's "sense"
phase includes this execution. Recorded on the worker thread, read on
the loop thread: a small lock keeps the two consistent.
"""

import threading
from typing import Tuple


class ClockAdvanceTiming:
    """Per-window and per-run execution times of the clock advance, ms."""

    def __init__(self):
        self._lock = threading.Lock()
        self._window = (0.0, 0)
        self._run_total = 0.0
        self._run_count = 0
        self._run_max = 0.0

    def record(self, ms: float) -> None:
        with self._lock:
            total, count = self._window
            self._window = (total + ms, count + 1)
            self._run_total += ms
            self._run_count += 1
            self._run_max = max(self._run_max, ms)

    def take_window(self) -> Tuple[float, int]:
        """(total ms, count) since the last call; resets the window."""
        with self._lock:
            window, self._window = self._window, (0.0, 0)
            return window

    def run_totals(self) -> Tuple[float, int, float]:
        """(total ms, count, max ms) over the whole run."""
        with self._lock:
            return self._run_total, self._run_count, self._run_max

    def summary_line(self) -> str:
        total, count, peak = self.run_totals()
        if not count:
            return "clock advance summary: executed 0"
        return (f"clock advance summary: executed {count}, mean "
                f"{total / count:.1f} ms, max {peak:.1f} ms (on the COM "
                f"worker, inside E6's sense phase)")
