"""The sensing summary lines must reach bridge.log before the bridge dies.

run_acav_run.py stops the bridge once the scheduler exits; in E6-BC-1
that happened before the bridge's own stop() ran, so summary lines
logged only in stop() never appeared. They are now logged at the
terminal epoch (before its barrier wait, which is never released), as a
"running" summary every SUMMARY_EVERY_STEPS steps, and in stop() only if
the terminal line was not logged already.
"""

import logging
import threading
from contextlib import nullcontext
from types import SimpleNamespace
from unittest.mock import Mock

from stk_bridge.backends.base import StkTime
from stk_bridge.core.time_sync import ExecutionMode
from stk_bridge.exceptions import LockstepBarrierTimeoutError
from stk_bridge.nodes import bridge_node as bridge_module
from stk_bridge.nodes.bridge_node import SUMMARY_EVERY_STEPS, StkBridgeNode

EPOCH = 1717237800.0
STOP = StkTime(EPOCH + 720.0)
COMBINED = "combined attitude/rate summary: x"
ORBIT = "orbit batch summary: y"


def _node(stop=STOP):
    node = StkBridgeNode.__new__(StkBridgeNode)
    node.sensing = SimpleNamespace(combine_summary_line=lambda: COMBINED,
                                   orbit_batch_summary_line=lambda: ORBIT)
    node._scenario_stop = stop
    node.time_sync = SimpleNamespace(step_index=0)
    return node


def _drive(node, steps):
    sim = 0.0
    for index in range(1, steps + 1):
        sim += 0.1
        node.time_sync.step_index = index
        node._sensing_summary_checkpoint(StkTime(EPOCH + sim))


def _messages(caplog):
    return [r.getMessage() for r in caplog.records
            if "summary:" in r.getMessage()]


def test_running_every_period_and_final_at_the_terminal_epoch(caplog):
    node = _node()
    with caplog.at_level(logging.INFO):
        _drive(node, 7200)
    messages = _messages(caplog)
    running = [m for m in messages if m.startswith("running ")]
    final = [m for m in messages if not m.startswith("running ")]
    assert SUMMARY_EVERY_STEPS == 1000
    assert running == ["running " + COMBINED, "running " + ORBIT] * 7
    assert final == [COMBINED, ORBIT]
    assert messages[-2:] == [COMBINED, ORBIT]


def test_stop_does_not_repeat_the_final_lines(caplog):
    node = _node()
    node._running = threading.Event()
    node._step_request = threading.Event()
    node._loop_thread = None
    node.worker = SimpleNamespace(stop=lambda timeout_s: None)
    with caplog.at_level(logging.INFO):
        _drive(node, 7200)
        node.stop()
    final = [m for m in _messages(caplog) if not m.startswith("running ")]
    assert final == [COMBINED, ORBIT]


def test_stop_logs_the_final_lines_when_the_terminal_epoch_was_not_reached(
        caplog):
    node = _node()
    node._running = threading.Event()
    node._step_request = threading.Event()
    node._loop_thread = None
    node.worker = SimpleNamespace(stop=lambda timeout_s: None)
    with caplog.at_level(logging.INFO):
        _drive(node, 50)
        node.stop()
        node.stop()
    assert _messages(caplog) == [COMBINED, ORBIT]


def test_without_a_stop_time_only_running_lines_until_stop(caplog):
    node = _node(stop=None)
    with caplog.at_level(logging.INFO):
        _drive(node, 7200)
    assert all(m.startswith("running ") for m in _messages(caplog))


def test_the_terminal_step_logs_before_its_barrier_wait():
    """At 720 s the twin never acks, so anything after the barrier wait
    would only run after an abort -- or never, if the bridge is killed."""
    node = StkBridgeNode.__new__(StkBridgeNode)
    node.config = {"time": {}}
    node._running = threading.Event()
    node._running.set()
    node._step_request = threading.Event()
    node._step_lock = threading.Lock()
    node._step_size = 0
    node._advance_ms = (0.0, 0)
    node._timings = SimpleNamespace(timed=lambda name: nullcontext())
    node.time_sync = Mock(step_index=7200)
    node.time_sync.step.return_value = STOP
    node.time_sync.await_barrier.side_effect = LockstepBarrierTimeoutError(
        7200, ["matlab_twin"])
    for name in ("_apply_commands", "_publish_clock", "_sense_and_publish",
                 "_run_rate_groups", "_publish_status",
                 "_sensing_summary_checkpoint"):
        setattr(node, name, Mock())
    try:
        node._lockstep_iteration()
    except LockstepBarrierTimeoutError:
        pass
    node._sensing_summary_checkpoint.assert_called_once_with(STOP)


def test_orbit_batch_uses_the_stop_time_read_at_startup():
    node = StkBridgeNode.__new__(StkBridgeNode)
    node.sensing_config = SimpleNamespace(
        orbit_batch_samples=100, orbit_batch_expected_window_s=720.0,
        orbit_batch_audit_position_m=1e-6, orbit_batch_audit_velocity_mps=1e-9,
        equivalence_diagnostic=False)
    node.time_sync = SimpleNamespace(mode=ExecutionMode.LOCKSTEP,
                                     config=SimpleNamespace(step_s=0.1))
    node.scenario = SimpleNamespace(epoch=StkTime(EPOCH))
    node._scenario_stop = STOP
    node.worker = SimpleNamespace(call=Mock(
        side_effect=AssertionError("no second stop-time read")))
    attached = []
    node.sensing = SimpleNamespace(attach_orbit_batch=attached.append)
    node._attach_orbit_batch()
    assert len(attached) == 1 and attached[0].stop_time is STOP


def test_the_module_reads_the_stop_time_once_at_startup():
    import inspect
    # start() runs its steps in _start() (startup phases, P10 report).
    source = inspect.getsource(bridge_module.StkBridgeNode._start)
    assert source.count("_read_scenario_stop()") == 1
