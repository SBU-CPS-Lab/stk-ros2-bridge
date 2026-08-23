"""Unit tests for core/rate_group.py."""

import pytest

from stk_bridge.core.rate_group import RateGroup, RateGroupScheduler


class TestRateGroup:
    def test_zero_rate_rejected(self):
        with pytest.raises(ValueError):
            RateGroup(name="bad", rate_hz=0.0)

    def test_first_call_is_always_due(self):
        group = RateGroup(name="g", rate_hz=1.0)
        assert group.is_due(0.0)

    def test_due_after_period_elapses(self):
        group = RateGroup(name="g", rate_hz=10.0)   # period 0.1s
        group.mark_run(0.0)
        assert not group.is_due(0.05)
        assert group.is_due(0.1)

    def test_disabled_group_never_due(self):
        group = RateGroup(name="g", rate_hz=10.0, enabled=False)
        assert not group.is_due(100.0)

    def test_overrun_counted_when_multiple_periods_missed(self):
        group = RateGroup(name="g", rate_hz=10.0)
        group.mark_run(0.0)
        group.mark_run(0.5)   # 5 periods late
        assert group.overruns == 4   # 5 elapsed - 1 for the run itself


class TestRateGroupScheduler:
    def test_from_config_defaults(self):
        scheduler = RateGroupScheduler.from_config({})
        names = {g.name for g in scheduler.groups()}
        assert names == {"fast", "medium", "slow"}

    def test_subscribe_to_unknown_group_raises(self):
        scheduler = RateGroupScheduler.from_config({
            "sensing": {"rate_groups": {"fast": {"rate_hz": 10.0}}}})
        with pytest.raises(KeyError):
            scheduler.subscribe("nonexistent", lambda t: None)

    def test_due_groups_ordered_fastest_first(self):
        scheduler = RateGroupScheduler.from_config({"sensing": {
            "rate_groups": {"slow": {"rate_hz": 1.0},
                            "fast": {"rate_hz": 10.0}}}})
        due = scheduler.due_groups(0.0)
        assert [g.name for g in due] == ["fast", "slow"]

    def test_run_due_invokes_callbacks_and_marks_run(self):
        scheduler = RateGroupScheduler.from_config({
            "sensing": {"rate_groups": {"fast": {"rate_hz": 10.0}}}})
        calls = []
        scheduler.subscribe("fast", lambda t: calls.append(t))
        scheduler.run_due(0.0)
        assert calls == [0.0]
        assert scheduler.group("fast").runs == 1

    def test_callback_error_does_not_prevent_other_callbacks(self):
        scheduler = RateGroupScheduler.from_config({
            "sensing": {"rate_groups": {"fast": {"rate_hz": 10.0}}}})
        calls = []

        def bad(t):
            raise RuntimeError("boom")

        scheduler.subscribe("fast", bad)
        scheduler.subscribe("fast", lambda t: calls.append(t))

        errors = []
        scheduler.run_due(0.0, on_error=lambda name, exc: errors.append(exc))
        assert calls == [0.0]
        assert len(errors) == 1

    def test_deterministic_over_simulated_time_not_wall_time(self):
        """Rate groups schedule on the sim-time argument passed in, never
        on time.time()/monotonic() -- this is what makes a lockstep run
        reproducible regardless of host speed."""
        scheduler = RateGroupScheduler.from_config({
            "sensing": {"rate_groups": {"fast": {"rate_hz": 10.0}}}})
        for i in range(21):
            scheduler.run_due(i * 0.05)
        assert scheduler.group("fast").runs == 11   # 0.0, 0.1, ..., 1.0

    def test_reset_clears_history(self):
        scheduler = RateGroupScheduler.from_config({
            "sensing": {"rate_groups": {"fast": {"rate_hz": 10.0}}}})
        scheduler.run_due(0.0)
        scheduler.reset()
        assert scheduler.group("fast").runs == 0
        assert scheduler.group("fast").is_due(0.0)
