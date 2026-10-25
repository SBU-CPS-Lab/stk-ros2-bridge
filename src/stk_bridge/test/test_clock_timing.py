"""The STK clock advance's own execution time, measured on the COM worker.

_advance_stk_clock only SUBMITS `Animate * SetValues ... Reset` to the
single-thread COM worker; the "clock advance" figure in the timing line
times that submit. The command itself executes on the worker ahead of
the step's sensing queries, so its cost lands inside E6's "sense" phase.
This measures the execution separately -- for reporting only: no STK
call, message or ordering changes, so determinism signatures cannot.
"""

import logging
import threading
import time
from types import SimpleNamespace

from stk_bridge.backends.base import StkTime
from stk_bridge.core.clock_timing import ClockAdvanceTiming
from stk_bridge.nodes.bridge_node import StkBridgeNode

WHEN = StkTime(1717237800.1)


class TestClockAdvanceTiming:
    def test_window_and_run_totals(self):
        timing = ClockAdvanceTiming()
        for ms in (10.0, 20.0, 30.0):
            timing.record(ms)
        assert timing.take_window() == (60.0, 3)
        assert timing.take_window() == (0.0, 0)        # window was reset
        timing.record(5.0)
        assert timing.run_totals() == (65.0, 4, 30.0)

    def test_summary_line(self):
        timing = ClockAdvanceTiming()
        for ms in (10.0, 20.0, 30.0):
            timing.record(ms)
        assert timing.summary_line() == (
            "clock advance summary: executed 3, mean 20.0 ms, max 30.0 ms "
            "(on the COM worker, inside the sense phase)")

    def test_summary_line_without_samples(self):
        assert ClockAdvanceTiming().summary_line() == (
            "clock advance summary: executed 0")

    def test_concurrent_records_are_not_lost(self):
        timing = ClockAdvanceTiming()

        def worker():
            for _ in range(1000):
                timing.record(1.0)

        threads = [threading.Thread(target=worker) for _ in range(4)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        assert timing.run_totals()[:2] == (4000.0, 4000)


class SlowClockBackend:
    def __init__(self):
        self.calls = []

    def set_current_time(self, when):
        time.sleep(0.02)
        self.calls.append(when)


def _node(backend):
    node = StkBridgeNode.__new__(StkBridgeNode)
    node.worker = SimpleNamespace(
        submit=lambda fn, description: fn(backend))
    node._clock_timing = ClockAdvanceTiming()
    return node


class TestBridgeMeasuresTheExecution:
    def test_the_worker_job_is_timed(self):
        backend = SlowClockBackend()
        node = _node(backend)
        node._advance_stk_clock(WHEN)
        assert backend.calls == [WHEN]                  # same STK call
        total, count, peak = node._clock_timing.run_totals()
        assert count == 1 and total >= 15.0 and peak == total

    def test_a_failing_clock_advance_is_still_timed_and_still_raises(self):
        class Failing:
            def set_current_time(self, when):
                raise RuntimeError("COM error")

        node = _node(Failing())
        try:
            node._advance_stk_clock(WHEN)
        except RuntimeError:
            pass
        else:
            raise AssertionError("the error must reach the worker")
        assert node._clock_timing.run_totals()[1] == 1

    def test_a_node_without_the_timer_still_advances(self):
        """Replay tests build a bare node; measurement must be optional."""
        backend = SlowClockBackend()
        node = SimpleNamespace(worker=SimpleNamespace(
            submit=lambda fn, description: fn(backend)))
        StkBridgeNode._advance_stk_clock(node, WHEN)
        assert backend.calls == [WHEN]


class TestReporting:
    def test_the_timing_line_reports_the_execution(self, caplog):
        node = StkBridgeNode.__new__(StkBridgeNode)
        node._timing_log = {"wall": time.monotonic() - 120.0, "step": 0,
                            "sense": 0.0, "actuate": 0.0,
                            "barrier_wait": 0.0, "samples": 0}
        node.time_sync = SimpleNamespace(step_index=10)
        node.sensing = None
        node._advance_ms = (1.0, 10)
        node._clock_timing = ClockAdvanceTiming()
        for _ in range(10):
            node._clock_timing.record(12.0)
        node._commands_late = node._commands_rejected = 0
        node._commands_missing = node._attitude_mismatches = 0
        with caplog.at_level(logging.INFO):
            node._log_timing_summary({"step_samples": 10})
        line = caplog.records[-1].getMessage()
        assert "clock advance submit 0.1 ms" in line
        assert "clock advance execute 12.0 ms (inside sense)" in line
        assert node._clock_timing.take_window() == (0.0, 0)

    def test_the_final_summaries_include_the_clock(self, caplog):
        node = StkBridgeNode.__new__(StkBridgeNode)
        node.sensing = SimpleNamespace(
            combine_summary_line=lambda: "combined attitude/rate summary: x",
            orbit_batch_summary_line=lambda: "orbit batch summary: y")
        node._clock_timing = ClockAdvanceTiming()
        node._clock_timing.record(8.0)
        with caplog.at_level(logging.INFO):
            node._log_sensing_summaries(final=True)
        assert caplog.records[-1].getMessage() == (
            "clock advance summary: executed 1, mean 8.0 ms, max 8.0 ms "
            "(on the COM worker, inside the sense phase)")
