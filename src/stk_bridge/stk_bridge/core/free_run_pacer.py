"""Fixed-rate pacing for the free-run loop, anchored at cycle starts.

The free-run loop used to sleep until the rate groups' next due time,
and the groups mark their run with the time AFTER sensing, so every
cycle cost its work plus a whole period. Here cycle k is scheduled at
start_0 + k * period: a cycle that finishes early sleeps until the next
scheduled start; one that overruns is followed at once, with the
schedule rebased to that moment (no catch-up burst, no negative sleep),
and is counted. Times are simulation seconds, so a real-time factor is
handled by the caller's sleep (TimeSync.sleep_until_next).

Free-run only: the lockstep loop never sleeps and never uses this.
"""


class FreeRunPacer:
    def __init__(self, period_s: float):
        if period_s <= 0.0:
            raise ValueError(f"pacing period must be > 0, got {period_s}")
        self.period_s = float(period_s)
        self.cycles = 0
        self.overruns = 0
        self._scheduled = None

    def begin(self, now_s: float) -> float:
        """Start a cycle; returns its scheduled start (the first at now)."""
        if self._scheduled is None:
            self._scheduled = now_s
        self.cycles += 1
        return self._scheduled

    def end(self, now_s: float) -> float:
        """Finish the cycle; returns when the next one should start."""
        deadline = self._scheduled + self.period_s
        if now_s >= deadline:
            self.overruns += 1
            self._scheduled = now_s
        else:
            self._scheduled = deadline
        return self._scheduled

    def summary_line(self) -> str:
        return (f"free-run pacing summary: cycles {self.cycles}, overruns "
                f"{self.overruns}, period {self.period_s:.3f} s")
