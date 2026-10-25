"""Free-run cycles are paced from the cycle start, not from the end.

The rate groups mark their run with the time AFTER sensing
(_run_rate_groups -> RateGroupScheduler.run_due -> mark_run), and the
free-run loop slept until that mark plus the fast period, so every
cycle cost its work plus ~0.1 s (E3 pilot on 48813ac: 239 ms cycles for
~134 ms of work). Cycles now run on a fixed-rate schedule anchored at
the first cycle's start: period = max(work, period); an overrunning
cycle is followed immediately by the next one, rebased to now -- no
catch-up burst and no negative sleep -- and is counted. The rate groups
are marked at the cycle's scheduled start, so medium and slow keep
their configured rates.
"""

import logging
from types import SimpleNamespace

import pytest

from stk_bridge.backends.base import StkTime
from stk_bridge.core.free_run_pacer import FreeRunPacer
from stk_bridge.core.rate_group import RateGroupScheduler
from stk_bridge.nodes.bridge_node import StkBridgeNode

EPOCH = 1717237800.0
PERIOD = 0.1


class TestPacer:
    def test_short_work_starts_every_period_without_drift(self):
        pacer = FreeRunPacer(PERIOD)
        now = 0.0
        starts = []
        for _ in range(1000):
            starts.append(pacer.begin(now))
            now += 0.03                               # work
            now = max(now, pacer.end(now))            # sleep to the target
        assert all(abs(s - k * PERIOD) < 1e-9 for k, s in enumerate(starts))
        assert pacer.overruns == 0 and pacer.cycles == 1000

    def test_an_overrun_starts_the_next_cycle_at_once_without_catch_up(self):
        pacer = FreeRunPacer(PERIOD)
        assert pacer.begin(0.0) == 0.0
        assert pacer.end(0.25) == 0.25                # overran: no wait
        assert pacer.overruns == 1
        assert pacer.begin(0.25) == 0.25              # rebased to now
        assert pacer.end(0.28) == pytest.approx(0.35)  # one period later
        assert pacer.overruns == 1

    def test_a_late_wake_up_keeps_the_grid(self):
        pacer = FreeRunPacer(PERIOD)
        pacer.begin(0.0)
        target = pacer.end(0.03)
        assert target == pytest.approx(0.1)
        assert pacer.begin(0.104) == pytest.approx(0.1)   # scheduled start
        assert pacer.end(0.13) == pytest.approx(0.2)

    def test_summary_line(self):
        pacer = FreeRunPacer(PERIOD)
        pacer.begin(0.0)
        pacer.end(0.3)
        assert pacer.summary_line() == (
            "free-run pacing summary: cycles 1, overruns 1, period 0.100 s")

    def test_the_period_must_be_positive(self):
        with pytest.raises(ValueError):
            FreeRunPacer(0.0)


class _Clock:
    """Simulation seconds for a fake free-run bridge (rtf 1)."""

    def __init__(self):
        self.sim = 0.0
        self.sleeps = []

    def sleep_until_next(self, due):
        if due is None:
            return
        wait = due - self.sim
        if wait > 0:
            self.sleeps.append(wait)
            self.sim = due


def _node(work):
    clock = _Clock()
    node = StkBridgeNode.__new__(StkBridgeNode)
    node.config = {"time": {}}
    node.time_sync = SimpleNamespace(
        now=lambda: StkTime(EPOCH + clock.sim),
        sim_seconds=lambda: clock.sim,
        sleep_until_next=clock.sleep_until_next, step_index=0)
    node.scheduler = RateGroupScheduler.from_config({"sensing": {
        "rate_groups": {"fast": {"rate_hz": 10.0},
                        "medium": {"rate_hz": 1.0},
                        "slow": {"rate_hz": 0.1}}}})
    node._timings = SimpleNamespace(timed=lambda name: _Null())
    node.health = SimpleNamespace(record_error=lambda message: None)
    node.starts = []

    def apply_commands(*a):
        node.starts.append(clock.sim)

    def sense(now):
        clock.sim += work(len(node.starts))

    node._apply_commands = apply_commands
    node._sense_and_publish = sense
    for name in ("_publish_clock", "_publish_status",
                 "_sensing_summary_checkpoint"):
        setattr(node, name, lambda *a, **k: None)
    return node, clock


class _Null:
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


class TestBridge:
    def _periods(self, node):
        return [b - a for a, b in zip(node.starts, node.starts[1:])]

    def test_cycle_period_is_max_of_work_and_period(self):
        for work_s, expected in ((0.034, PERIOD), (0.134, 0.134)):
            node, _ = _node(lambda k, w=work_s: w)
            for _ in range(50):
                node._free_running_iteration()
            assert all(p == pytest.approx(expected, abs=1e-9)
                       for p in self._periods(node)), work_s

    def test_overruns_are_counted_and_not_caught_up(self):
        works = [0.03] * 10 + [0.35] + [0.03] * 10
        node, clock = _node(lambda k: works[k - 1])
        for _ in works:
            node._free_running_iteration()
        periods = self._periods(node)
        assert periods[10] == pytest.approx(0.35)       # the overrun cycle
        assert all(p == pytest.approx(PERIOD) for p in periods[11:])
        assert node._free_pacer.overruns == 1

    def test_no_drift_over_a_thousand_cycles(self):
        node, _ = _node(lambda k: 0.03 + 0.05 * ((k * 7) % 3) / 2)
        for _ in range(1000):
            node._free_running_iteration()
        assert node.starts[-1] == pytest.approx(999 * PERIOD, abs=1e-6)

    def test_medium_and_slow_keep_their_rates(self):
        node, clock = _node(lambda k: 0.03)
        while clock.sim < 100.0 - 1e-9:
            node._free_running_iteration()
        stats = node.scheduler.statistics()
        assert stats["fast"]["runs"] == pytest.approx(1000, abs=1)
        assert stats["medium"]["runs"] == pytest.approx(100, abs=1)
        assert stats["slow"]["runs"] == pytest.approx(10, abs=1)

    def test_the_pacing_summary_is_logged_with_the_final_summaries(
            self, caplog):
        node, _ = _node(lambda k: 0.03)
        node._free_running_iteration()
        node.sensing = SimpleNamespace(
            combine_summary_line=lambda: "combined attitude/rate summary: x",
            orbit_batch_summary_line=lambda: "orbit batch summary: y")
        with caplog.at_level(logging.INFO):
            node._log_sensing_summaries(final=True)
        assert "free-run pacing summary: cycles 1, overruns 0" in caplog.text
