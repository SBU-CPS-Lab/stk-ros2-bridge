"""Regression checks for startup waiting and fail-closed lockstep."""
from contextlib import nullcontext
from types import SimpleNamespace
from unittest.mock import Mock
import threading
import pytest
from stk_bridge.nodes.bridge_node import StkBridgeNode
from stk_bridge.core.time_sync import ExecutionMode
from stk_bridge.exceptions import LockstepBarrierTimeoutError
from stk_bridge.exceptions import LockstepCommandError
from stk_bridge.backends.base import StkTime
from stk_bridge.interfaces.ros_subscribers import CommandInbox, AttitudeCommand


def test_barrier_abort_does_not_start_another_step():
    node = StkBridgeNode.__new__(StkBridgeNode)
    node._running = threading.Event(); node._running.set()
    node.time_sync = SimpleNamespace(mode=ExecutionMode.LOCKSTEP, now=lambda: 1)
    node.health = Mock(); node._publish_status = Mock()
    error = LockstepBarrierTimeoutError(2, ['matlab_twin'])
    node._lockstep_iteration = Mock(side_effect=error)
    node._loop()
    node._lockstep_iteration.assert_called_once()
    node.health.record_error.assert_called_once_with(str(error), fatal=True)
    assert not node._running.is_set()


def test_startup_retry_republishes_without_advancing_or_resetting_barrier():
    node = StkBridgeNode.__new__(StkBridgeNode)
    node.config = {'time': {'wait_for_startup_participants': True}}
    node._running = threading.Event(); node._running.set()
    node._step_request = threading.Event(); node._step_lock = threading.Lock()
    node._step_size = 0
    node._last_startup_log = 0.0
    node._advance_ms = (0.0, 0)
    node._timings = SimpleNamespace(timed=lambda name: nullcontext())
    node.time_sync = Mock(step_index=1)
    node.time_sync.step.return_value = 0.1
    node.time_sync.await_barrier.side_effect = [LockstepBarrierTimeoutError(1,['matlab_twin']), None]
    for name in ['_apply_commands','_publish_clock','_sense_and_publish','_run_rate_groups','_publish_status']:
        setattr(node,name,Mock())
    node._lockstep_iteration()
    node.time_sync.step.assert_called_once()
    assert node._publish_clock.call_count == 2
    assert node._sense_and_publish.call_count == 2


def _timed_node(barrier_outcomes):
    from stk_bridge.core.step_timing import PhaseTimings
    node = StkBridgeNode.__new__(StkBridgeNode)
    node.config = {'time': {'wait_for_startup_participants': True}}
    node._running = threading.Event()
    node._running.set()
    node._step_request = threading.Event()
    node._step_lock = threading.Lock()
    node._step_size = 0
    node._last_startup_log = 0.0
    node._advance_ms = (0.0, 0)
    node._timings = PhaseTimings()
    node.time_sync = Mock(step_index=1)
    node.time_sync.step.return_value = 0.1
    node.time_sync.await_barrier.side_effect = barrier_outcomes
    for name in ['_apply_commands', '_publish_clock', '_sense_and_publish',
                 '_run_rate_groups', '_publish_status']:
        setattr(node, name, Mock())
    return node


def test_startup_wait_is_not_recorded_as_barrier_wait():
    # Launch latency (a human starting MATLAB) is not loop cost; one such
    # sample would bias E6's run-level mean barrier wait by milliseconds.
    node = _timed_node([LockstepBarrierTimeoutError(1, ['matlab_twin']), None])
    node._lockstep_iteration()
    assert node._timings.summary()["mean_barrier_wait_ms"] == 0.0
    assert node._timings.summary()["step_samples"] == 1   # actuate only


def test_an_ordinary_barrier_wait_is_still_recorded():
    node = _timed_node([None])
    node._lockstep_iteration()
    summary = node._timings.summary()
    assert summary["step_samples"] == 1
    assert len(node._timings._samples["barrier_wait"]) == 1


def _command_node():
    node = _timed_node([None])
    node.config['time']['require_attitude_command_each_step'] = True
    node.sensing_config = SimpleNamespace(satellites=['*/Satellite/SSS-1'])
    node.time_sync.now.return_value = StkTime(10.0)
    node.time_sync.config = SimpleNamespace(step_s=0.1, barrier_timeout_s=0.1)
    node.inbox = Mock()
    node.inbox.wait_for_commands.return_value = []
    node._apply_commands.return_value = {'applied': {'attitude': 1}, 'rejected': {}}
    return node


def test_command_gate_waits_for_next_timestamp_before_advance():
    node = _command_node()
    node._lockstep_iteration()
    node.inbox.wait_for_commands.assert_called_once_with(
        ['attitude'], ['*/Satellite/SSS-1'], StkTime(10.1), 0.1)
    node.time_sync.step.assert_called_once()


def test_missing_command_never_advances():
    node = _command_node()
    node.inbox.wait_for_commands.return_value = ['attitude for */Satellite/SSS-1']
    with pytest.raises(LockstepCommandError):
        node._lockstep_iteration()
    node._apply_commands.assert_not_called()
    node.time_sync.step.assert_not_called()


@pytest.mark.parametrize('result', [None, {'applied': {}, 'rejected': {'attitude': 1}}])
def test_rejected_command_never_advances(result):
    node = _command_node()
    node._apply_commands.return_value = result
    with pytest.raises(LockstepCommandError):
        node._lockstep_iteration()
    node.time_sync.step.assert_not_called()


def test_initial_step_does_not_wait_for_a_command():
    node = _command_node()
    node.time_sync.step_index = 0
    node._lockstep_iteration()
    node.inbox.wait_for_commands.assert_not_called()
    node.time_sync.step.assert_called_once()


def test_command_failure_stops_loop():
    node = _command_node()
    node.time_sync.mode = ExecutionMode.LOCKSTEP
    node.health = Mock()
    node._lockstep_iteration = Mock(side_effect=LockstepCommandError(2, 'rejected'))
    node._loop()
    assert not node._running.is_set()
    node._lockstep_iteration.assert_called_once()


def test_inbox_rejects_wrong_epoch_and_accepts_exact_epoch():
    inbox = CommandInbox()
    path = '*/Satellite/SSS-1'
    inbox.put('attitude', AttitudeCommand(path, StkTime(10), (0, 0, 0, 1)))
    assert inbox.wait_for_commands(["attitude"], [path], StkTime(10.1), 0) == ["attitude for " + path]
    inbox.put('attitude', AttitudeCommand(path, StkTime(10.1), (0, 0, 0, 1)))
    assert inbox.wait_for_commands(["attitude"], [path], StkTime(10.1), 0) == []


def test_inbox_wait_is_released_by_arrival():
    inbox = CommandInbox()
    path = '*/Satellite/SSS-1'
    timer = threading.Timer(0.02, lambda: inbox.put(
        'attitude', AttitudeCommand(path, StkTime(10.1), (0, 0, 0, 1))))
    timer.start()
    try:
        assert inbox.wait_for_commands(["attitude"], [path], StkTime(10.1), 1) == []
    finally:
        timer.join()
