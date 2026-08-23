"""Unit tests for core/time_sync.py -- execution modes and the lockstep
barrier.

The barrier tests are the most important in this file: a race here
produces the deadlock or non-determinism the whole design exists to
prevent. See core/time_sync.py's module docstring and
nodes/bridge_node.py's concurrency-model docstring.
"""

import threading
import time

import pytest

from stk_bridge.backends.base import StkTime
from stk_bridge.core.time_sync import (
    BarrierTimeoutPolicy,
    ExecutionMode,
    LockstepBarrier,
    TimeSync,
    TimeSyncConfig,
)
from stk_bridge.exceptions import LockstepBarrierTimeoutError


class TestExecutionModeParsing:
    def test_parse_string_case_insensitive(self):
        assert ExecutionMode.parse("lockstep") is ExecutionMode.LOCKSTEP
        assert ExecutionMode.parse("LOCKSTEP") is ExecutionMode.LOCKSTEP

    def test_parse_hyphen_normalised_to_underscore(self):
        assert ExecutionMode.parse("free-run") is ExecutionMode.FREE_RUN

    def test_parse_int(self):
        assert ExecutionMode.parse(2) is ExecutionMode.LOCKSTEP

    def test_parse_unknown_raises(self):
        with pytest.raises(ValueError):
            ExecutionMode.parse("warp_speed")


class TestTimeSyncConfig:
    def test_step_s_must_be_positive(self):
        with pytest.raises(ValueError):
            TimeSyncConfig.from_config({"time": {"step_s": 0.0}})

    def test_unknown_timeout_policy_rejected(self):
        with pytest.raises(ValueError):
            TimeSyncConfig.from_config(
                {"time": {"on_barrier_timeout": "explode"}})

    def test_lockstep_with_no_participants_warns_but_does_not_raise(self):
        config = TimeSyncConfig.from_config(
            {"time": {"mode": "lockstep", "barrier_participants": []}})
        assert config.mode is ExecutionMode.LOCKSTEP


class TestLockstepBarrier:
    def test_ack_for_wrong_step_is_rejected_not_counted(self):
        """An ack for a stale step must NOT satisfy the current step's
        wait -- that would let a lagging participant appear synced."""
        barrier = LockstepBarrier(["a"], timeout_s=0.2)
        barrier.begin_step(5)
        accepted = barrier.acknowledge("a", 4)   # stale
        assert accepted is False
        assert barrier.missing() == ["a"]

    def test_wait_returns_empty_when_all_ack(self):
        barrier = LockstepBarrier(["a", "b"], timeout_s=1.0)
        barrier.begin_step(1)

        def acker():
            time.sleep(0.02)
            barrier.acknowledge("a", 1)
            barrier.acknowledge("b", 1)

        threading.Thread(target=acker).start()
        missing = barrier.wait()
        assert missing == []

    def test_wait_times_out_and_reports_missing(self):
        barrier = LockstepBarrier(["a", "b"], timeout_s=0.1)
        barrier.begin_step(1)
        barrier.acknowledge("a", 1)
        missing = barrier.wait()
        assert missing == ["b"]
        assert barrier.timeouts == 1

    def test_ack_from_unregistered_participant_ignored(self):
        barrier = LockstepBarrier(["a"], timeout_s=0.1)
        barrier.begin_step(1)
        accepted = barrier.acknowledge("stranger", 1)
        assert accepted is False


class TestTimeSyncLockstep:
    def _make(self, **overrides):
        config = TimeSyncConfig(
            mode=ExecutionMode.LOCKSTEP, step_s=0.1,
            barrier_participants=["twin"], barrier_timeout_s=0.2,
            on_barrier_timeout=BarrierTimeoutPolicy.ABORT,
        )
        for key, value in overrides.items():
            setattr(config, key, value)
        ts = TimeSync(config)
        ts.start(StkTime(1_717_200_000.0))
        return ts

    def test_step_advances_by_configured_step_s(self):
        ts = self._make()
        t0 = ts.now().epoch_seconds
        ts.step()
        assert ts.now().epoch_seconds == pytest.approx(t0 + 0.1)

    def test_step_increments_index(self):
        ts = self._make()
        ts.step()
        ts.step()
        assert ts.step_index == 2

    def test_step_outside_lockstep_raises(self):
        config = TimeSyncConfig(mode=ExecutionMode.FREE_RUN)
        ts = TimeSync(config)
        ts.start(StkTime(0.0))
        with pytest.raises(RuntimeError):
            ts.step()

    def test_await_barrier_no_participants_returns_immediately(self):
        ts = self._make(barrier_participants=[])
        ts.step()
        assert ts.await_barrier() == []

    def test_await_barrier_policy_abort_raises(self):
        ts = self._make()
        ts.step()
        with pytest.raises(LockstepBarrierTimeoutError):
            ts.await_barrier()

    def test_await_barrier_policy_continue_returns_missing_without_raising(self):
        ts = self._make(on_barrier_timeout=BarrierTimeoutPolicy.CONTINUE)
        ts.step()
        missing = ts.await_barrier()
        assert missing == ["twin"]

    def test_ack_from_another_thread_releases_a_blocked_wait(self):
        """The deadlock-avoidance property this whole design rests on:
        a wait() blocked on one thread must be released by an
        acknowledge() called from a DIFFERENT thread."""
        ts = self._make(barrier_timeout_s=2.0)
        ts.step()

        def acker():
            time.sleep(0.05)
            ts.acknowledge("twin", ts.step_index)

        thread = threading.Thread(target=acker)
        thread.start()
        start = time.monotonic()
        missing = ts.await_barrier()
        elapsed = time.monotonic() - start
        thread.join()

        assert missing == []
        assert elapsed < 1.0, "wait did not release promptly on ack"


class TestTimeSyncFreeRunning:
    def test_now_advances_with_wall_clock(self):
        config = TimeSyncConfig(mode=ExecutionMode.FREE_RUN,
                                real_time_factor=1.0)
        ts = TimeSync(config)
        ts.start(StkTime(0.0))
        t0 = ts.now().epoch_seconds
        time.sleep(0.05)
        t1 = ts.now().epoch_seconds
        assert t1 > t0

    def test_pause_freezes_time(self):
        config = TimeSyncConfig(mode=ExecutionMode.FREE_RUN)
        ts = TimeSync(config)
        ts.start(StkTime(0.0))
        ts.pause()
        t0 = ts.now().epoch_seconds
        time.sleep(0.05)
        t1 = ts.now().epoch_seconds
        assert t0 == pytest.approx(t1, abs=1e-6)

    def test_real_time_factor_scales_elapsed_time(self):
        config = TimeSyncConfig(mode=ExecutionMode.FREE_RUN,
                                real_time_factor=10.0)
        ts = TimeSync(config)
        ts.start(StkTime(0.0))
        time.sleep(0.05)
        # At 10x, ~0.05s wall should be ~0.5s sim, well above the noise
        # floor of scheduling jitter.
        assert ts.sim_seconds() > 0.3
