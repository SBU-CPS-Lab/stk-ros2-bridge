"""How a run ends, and what the scheduler says when the twin's clock is wrong.

Both were found on the first live E6 run (2026-09-26). The twin stamped
TwinStatus with wall-clock time, so no sample ever fell in a hold window
and every target failed to finalise; the scheduler then waited for a
grace period counted in SIMULATION seconds past the window's end -- a
step the bridge could never produce, since the STK scenario ends there --
so the run wrote no record at all after two hours.

``SchedulerNode`` is built with ``__new__`` so these run without ROS; the
methods under test touch no rclpy.
"""

import logging
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from acav import scheduler_node as module
from acav.scheduler_node import SchedulerNode
from stk_bridge.backends.base import StkTime

EPOCH = 1717237800.0
TARGETS = ["T1", "T2", "T3"]


@pytest.fixture
def clock(monkeypatch):
    now = {"t": 1000.0}
    monkeypatch.setattr(module.time, "monotonic", lambda: now["t"])
    return now


def _node():
    node = SchedulerNode.__new__(SchedulerNode)
    node._spec = SimpleNamespace(
        window_duration_s=720.0,
        targets=[SimpleNamespace(id=t) for t in TARGETS])
    node._finalized = set()
    node._window_elapsed_wall = None
    node._shutdown_requested = False
    node._finish_run = Mock()
    return node


class TestRunCompletion:
    def test_nothing_happens_before_the_window_ends(self, clock):
        node = _node()
        node._finalized = set(TARGETS)
        node._check_run_complete(719.9)
        node._finish_run.assert_not_called()

    def test_all_targets_scored_finishes_at_the_window_end(self, clock):
        node = _node()
        node._finalized = set(TARGETS)
        node._check_run_complete(720.0)
        node._finish_run.assert_called_once_with(forced=False)

    def test_an_outstanding_access_answer_gets_a_wall_clock_grace(self, clock):
        node = _node()
        node._finalized = {"T1", "T2"}
        node._check_run_complete(720.0)
        node._finish_run.assert_not_called()
        clock["t"] += module.FINALIZE_GRACE_WALL_S - 1.0
        node._maybe_finish_run()
        node._finish_run.assert_not_called()
        clock["t"] += 1.0
        node._maybe_finish_run()          # the timer, with no further ticks
        node._finish_run.assert_called_once_with(forced=True)

    def test_the_timer_alone_does_nothing_before_the_window_ends(self, clock):
        node = _node()
        clock["t"] += 10 * module.FINALIZE_GRACE_WALL_S
        node._maybe_finish_run()
        node._finish_run.assert_not_called()


def _stamp(seconds):
    whole = int(seconds)
    return SimpleNamespace(sec=whole, nanosec=int(round((seconds - whole) * 1e9)))


def _status(stamp_s):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=_stamp(stamp_s)),
        boresight_error_rad=0.01, mode="SLEW",
        wheel_momentum_nms=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        wheels_saturated=False, wheel_energy_j=0.0)


class TestTwinStampCheck:
    def _node(self):
        node = SchedulerNode.__new__(SchedulerNode)
        node._epoch = StkTime(EPOCH)
        node._last_sim_time_s = 58.0
        node._misstamped_twin_status = 0
        node._tracker = Mock()
        node._aggregate = Mock()
        return node

    def test_a_wall_clock_stamp_is_reported_once(self, caplog):
        node = self._node()
        with caplog.at_level(logging.ERROR):
            for _ in range(3):
                node._on_twin_status(_status(1790420797.1))
        errors = [r for r in caplog.records if r.levelno == logging.ERROR]
        assert len(errors) == 1
        assert "wall-clock" in errors[0].getMessage()
        assert node._misstamped_twin_status == 3

    def test_a_scenario_time_stamp_is_quiet(self, caplog):
        node = self._node()
        with caplog.at_level(logging.ERROR):
            node._on_twin_status(_status(EPOCH + 58.0))
        assert caplog.records == []
        node._tracker.observe.assert_called_once()
        assert node._tracker.observe.call_args.args[0].time_s == pytest.approx(58.0)
