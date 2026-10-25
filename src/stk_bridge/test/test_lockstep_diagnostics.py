"""The bridge must say, in its own log, when a lockstep run is going wrong.

Each case here is a failure the first live E6 run hit silently: attitude
commands stamped with wall-clock time were written to STK years outside
the scenario for two hours without a single warning, and the run then
died at the end of the scenario window with a barrier timeout naming the
twin, while the actual cause -- STK returning no state -- was logged at
DEBUG.
"""

import logging
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from stk_bridge.backends.base import StkTime
from stk_bridge.core.time_sync import ExecutionMode
from stk_bridge.exceptions import LockstepBarrierTimeoutError
from stk_bridge.nodes import bridge_node as bridge_module
from stk_bridge.nodes.bridge_node import StkBridgeNode

EPOCH = 1717237800.0
PATH = "*/Satellite/SSS-1"


def _node(mode=ExecutionMode.LOCKSTEP, sim_s=10.0, config=None):
    node = StkBridgeNode.__new__(StkBridgeNode)
    node.config = config or {}
    node._warn_counts = {}
    node._commands_late = 0
    node._commands_rejected = 0
    node._commanded_paths = set()
    node._commands_missing = 0
    node._attitude_mismatches = 0
    node.time_sync = SimpleNamespace(
        mode=mode, config=SimpleNamespace(step_s=0.1),
        epoch=StkTime(EPOCH), step_index=int(sim_s / 0.1),
        now=lambda: StkTime(EPOCH + sim_s))
    return node


def _command(stamp_s):
    return ("attitude", SimpleNamespace(time=StkTime(stamp_s),
                                        object_path=PATH))


class TestCommandTimeScreen:
    def test_a_wall_clock_stamp_is_rejected_loudly(self, caplog):
        node = _node()
        wall_clock = 1790420797.1   # stamps from a live run
        with caplog.at_level(logging.ERROR):
            kept = node._screen_command_times([_command(wall_clock)])
        assert kept == []
        assert node._commands_rejected == 1
        assert "wall-clock" in caplog.text

    def test_a_command_for_the_next_step_is_kept_and_not_late(self, caplog):
        node = _node(sim_s=10.0)
        with caplog.at_level(logging.WARNING):
            kept = node._screen_command_times([_command(EPOCH + 10.1)])
        assert len(kept) == 1
        assert node._commands_late == 0 and node._commands_rejected == 0
        assert caplog.text == ""

    def test_a_command_for_an_already_sensed_step_is_kept_but_counted(self):
        node = _node(sim_s=10.0)
        kept = node._screen_command_times([_command(EPOCH + 10.0)])
        assert len(kept) == 1     # still fixes STK's attitude history
        assert node._commands_late == 1

    def test_free_run_is_not_screened(self):
        node = _node(mode=ExecutionMode.FREE_RUN)
        commands = [_command(1790420797.1)]
        assert node._screen_command_times(commands) == commands

    def test_the_limit_is_configurable(self):
        node = _node(config={"time": {"max_command_offset_s": 5.0}})
        assert node._screen_command_times([_command(EPOCH + 30.0)]) == []


def test_repeated_problems_are_logged_at_1_10_100(caplog):
    node = _node()
    with caplog.at_level(logging.WARNING):
        for _ in range(150):
            node._warn_limited("k", logging.WARNING, "problem")
    counts = [r.getMessage() for r in caplog.records]
    assert counts == ["problem [occurrence 1]", "problem [occurrence 10]",
                      "problem [occurrence 100]"]


class TestSensingGapsAreVisible:
    def _sensing_node(self, worker_call):
        node = _node()
        node.publishers = {PATH: Mock()}
        node.worker = Mock(call=worker_call)
        node.health = Mock()
        node.sensing = Mock()
        node.tf = None
        return node

    def test_empty_state_in_lockstep_warns_and_names_the_consequence(self, caplog):
        node = self._sensing_node(lambda fn, label: [])
        with caplog.at_level(logging.WARNING):
            node._sense_and_publish(StkTime(EPOCH + 720.1))
        assert "no state" in caplog.text
        assert "720.100" in caplog.text
        assert "will not ack" in caplog.text
        node.publishers[PATH].publish_samples.assert_not_called()

    def test_a_sensing_exception_is_a_warning_not_debug(self, caplog):
        def fail(fn, label):
            raise RuntimeError("provider returned nothing")
        node = self._sensing_node(fail)
        with caplog.at_level(logging.WARNING):
            node._sense_and_publish(StkTime(EPOCH + 5.0))
        assert "sensing failed" in caplog.text


def _startup_node(step_index):
    node = StkBridgeNode.__new__(StkBridgeNode)
    node.config = {"time": {"wait_for_startup_participants": True}}
    node._running = threading.Event()
    node._running.set()
    node._step_request = threading.Event()
    node._step_lock = threading.Lock()
    node._step_size = 0
    node._last_startup_log = 0.0
    node._advance_ms = (0.0, 0)
    from stk_bridge.core.step_timing import PhaseTimings
    node._timings = PhaseTimings()
    node.time_sync = Mock(step_index=step_index)
    node.time_sync.await_barrier.side_effect = [
        LockstepBarrierTimeoutError(step_index, ["acav_scheduler"]), None]
    for name in ["_apply_commands", "_publish_clock", "_sense_and_publish",
                 "_run_rate_groups", "_publish_status"]:
        setattr(node, name, Mock())
    return node


def test_startup_republishes_on_a_short_poll_not_the_barrier_timeout():
    node = _startup_node(step_index=1)
    node._lockstep_iteration()
    timeouts = [c.kwargs["timeout_s"]
                for c in node.time_sync.await_barrier.call_args_list]
    assert timeouts == [bridge_module.STARTUP_REPUBLISH_S] * 2


def test_after_step_1_a_timeout_uses_the_full_barrier_timeout_and_aborts():
    node = _startup_node(step_index=2)
    with pytest.raises(LockstepBarrierTimeoutError):
        node._lockstep_iteration()
    assert node.time_sync.await_barrier.call_args.kwargs["timeout_s"] is None
