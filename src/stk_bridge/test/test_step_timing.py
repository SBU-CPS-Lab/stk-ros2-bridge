"""Tests for E6's latency breakdown accumulator.

This is the one piece of E6 that can be verified on the dev machine:
whether the arithmetic is right. Whether the numbers it produces are
*meaningful* -- the actual sense/actuate/barrier split against a real
backend -- can only be measured on the deploy machine.
"""

import pytest

from stk_bridge.core.step_timing import PHASES, PhaseTimings


class TestRecordAndSummarise:
    def test_the_mean_is_correct(self):
        timings = PhaseTimings()
        for value in (10.0, 20.0, 30.0):
            timings.record("sense", value)
        assert timings.summary()["mean_sense_ms"] == pytest.approx(20.0)

    def test_step_samples_is_the_iteration_count(self):
        timings = PhaseTimings()
        for _ in range(5):
            timings.record("sense", 1.0)
            timings.record("actuate", 1.0)
        assert timings.summary()["step_samples"] == 5

    def test_an_unmeasured_phase_is_zero_not_nan(self):
        """A valid float always goes on the wire; step_samples is what
        tells a consumer whether zero means fast or unmeasured."""
        timings = PhaseTimings()
        timings.record("sense", 5.0)
        summary = timings.summary()
        assert summary["mean_actuate_ms"] == 0.0
        assert summary["mean_barrier_wait_ms"] == 0.0

    def test_an_unknown_phase_is_refused(self):
        with pytest.raises(ValueError, match="unknown phase"):
            PhaseTimings().record("compute_the_universe", 1.0)

    def test_every_phase_has_a_summary_key(self):
        summary = PhaseTimings().summary()
        for phase in PHASES:
            assert f"mean_{phase}_ms" in summary


class TestWindowing:
    def test_reset_clears_every_phase(self):
        timings = PhaseTimings()
        timings.record("sense", 100.0)
        timings.record("actuate", 100.0)
        timings.record("barrier_wait", 100.0)
        timings.reset()

        summary = timings.summary()
        assert summary["mean_sense_ms"] == 0.0
        assert summary["mean_actuate_ms"] == 0.0
        assert summary["mean_barrier_wait_ms"] == 0.0
        assert summary["step_samples"] == 0

    def test_a_slow_startup_iteration_does_not_haunt_later_windows(self):
        """The failure mode a lifetime average has: one slow first
        connection would drag the mean down for the rest of an
        hour-long run and understate the achieved step rate."""
        timings = PhaseTimings()
        timings.record("sense", 5000.0)          # one slow outlier
        timings.reset()
        for _ in range(10):
            timings.record("sense", 5.0)
        assert timings.summary()["mean_sense_ms"] == pytest.approx(5.0)


class TestTimedContextManager:
    def test_it_records_a_positive_duration(self):
        import time

        timings = PhaseTimings()
        with timings.timed("sense"):
            time.sleep(0.001)
        assert timings.summary()["mean_sense_ms"] > 0.0
        assert timings.summary()["step_samples"] == 1

    def test_it_records_even_when_the_block_raises(self):
        """A phase that failed still took wall-clock time, and E6's
        breakdown should not silently drop the sample that happened to
        coincide with an error."""
        timings = PhaseTimings()
        with pytest.raises(RuntimeError):
            with timings.timed("actuate"):
                raise RuntimeError("boom")
        assert timings.summary()["step_samples"] == 1
