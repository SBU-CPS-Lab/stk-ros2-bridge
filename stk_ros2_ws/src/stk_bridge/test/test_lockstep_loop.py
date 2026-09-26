"""Regression checks for startup waiting and fail-closed lockstep."""
from contextlib import nullcontext
from types import SimpleNamespace
from unittest.mock import Mock
import threading
from stk_bridge.nodes.bridge_node import StkBridgeNode
from stk_bridge.core.time_sync import ExecutionMode
from stk_bridge.exceptions import LockstepBarrierTimeoutError


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
