"""The runner's ROS waits use their own rclpy context, and keep their error.

P10 live session (3186642): the first E6 attempt failed at idle_check
with "failed to shutdown: rcl_shutdown already called on the given
context". The four waits (idle check, bridge connected, twin ack, twin
identity) all used rclpy's default context: rclpy.init() on it installs
SIGINT/SIGTERM handlers that shut it down, and each wait's
``finally: rclpy.shutdown()`` then raised on the already shut-down
context and replaced whatever had stopped the wait -- so the run
recorded the shutdown error, not its cause.

Each wait now initialises a private Context with no signal handlers,
spins it with its own executor and ends it with try_shutdown, which
does not raise on a context that is already shut down; the wait's own
exception is the one that propagates. execute_run records it: a failed
step with the message, and the exception's type and traceback under
summary["errors"].

rclpy is replaced by a fake here: these tests run without ROS.
"""

import json
import sys
import types

import pytest

import acav.deploy_run_cli as deploy_run_cli
from acav.deploy_run_cli import ConnectResult, Deps, execute_run

from test_deploy_run_cli import FakeProcess, _config  # noqa: I100

NO_HANDLERS = "NO"


class FakeContext:
    def __init__(self):
        self.initialised = False
        self.shut_down = False


class FakeRos:
    """Enough of rclpy for the four waits, scripted per test."""

    def __init__(self, monkeypatch, on_spin=None, publishers=0):
        self.calls = []
        self.contexts = []
        self.subscriptions = {}
        self.on_spin = on_spin or (lambda fake: None)
        self.publishers = publishers
        self.now = 0.0
        fake = self

        class Node:
            def __init__(self, name, context):
                self.name = name
                self.context = context

            def create_subscription(self, kind, topic, callback, qos):
                fake.subscriptions[topic] = callback

            def count_publishers(self, topic):
                return fake.publishers

            def destroy_node(self):
                fake.calls.append(("destroy_node", self.name))

        class Executor:
            def __init__(self, context=None):
                fake.calls.append(("executor", context))
                self.context = context

            def add_node(self, node):
                assert node.context is self.context
                return True

            def spin_once(self, timeout_sec=None):
                fake.now += timeout_sec or 0.0
                fake.on_spin(fake)

            def shutdown(self, timeout_sec=None):
                fake.calls.append(("executor_shutdown", self.context))

        def make_context():
            context = FakeContext()
            fake.contexts.append(context)
            return context

        def init(args=None, context=None, domain_id=None,
                 signal_handler_options=None):
            fake.calls.append(("init", context, signal_handler_options))
            context.initialised = True

        def create_node(name, context=None):
            fake.calls.append(("create_node", name, context))
            return Node(name, context)

        def shutdown(context=None):
            fake.calls.append(("shutdown", context))
            if context is None or context.shut_down:
                raise RuntimeError("failed to shutdown: rcl_shutdown "
                                   "already called on the given context")
            context.shut_down = True

        def try_shutdown(context=None):
            fake.calls.append(("try_shutdown", context))
            if context is not None:
                context.shut_down = True

        def spin_once(*args, **kwargs):
            raise AssertionError("the global executor spins the default "
                                 "context")

        rclpy = types.ModuleType("rclpy")
        rclpy.init = init
        rclpy.create_node = create_node
        rclpy.shutdown = shutdown
        rclpy.try_shutdown = try_shutdown
        rclpy.spin_once = spin_once
        context_module = types.ModuleType("rclpy.context")
        context_module.Context = make_context
        executors = types.ModuleType("rclpy.executors")
        executors.SingleThreadedExecutor = Executor
        signals = types.ModuleType("rclpy.signals")
        signals.SignalHandlerOptions = types.SimpleNamespace(
            NO=NO_HANDLERS, ALL="ALL")
        rclpy.context = context_module
        rclpy.executors = executors
        rclpy.signals = signals

        qos = types.ModuleType("stk_bridge.interfaces.qos")
        qos.status_qos = lambda depth=1: "status_qos"
        qos.command_qos = lambda depth=10: "command_qos"
        qos.clock_qos = lambda depth=1: "clock_qos"

        class BridgeStatus:
            STARTING, CONNECTED, ERROR = 0, 1, 4

            def __init__(self, state=0, backend_type="com", last_error=""):
                self.state = state
                self.backend_type = backend_type
                self.last_error = last_error

        stk_msgs = types.ModuleType("stk_msgs")
        stk_msgs_msg = types.ModuleType("stk_msgs.msg")
        stk_msgs_msg.BridgeStatus = BridgeStatus
        stk_msgs_msg.ScenarioTime = object
        std_msgs = types.ModuleType("std_msgs")
        std_msgs_msg = types.ModuleType("std_msgs.msg")
        std_msgs_msg.String = object
        self.BridgeStatus = BridgeStatus

        for name, module in (("rclpy", rclpy),
                             ("rclpy.context", context_module),
                             ("rclpy.executors", executors),
                             ("rclpy.signals", signals),
                             ("stk_bridge.interfaces.qos", qos),
                             ("stk_msgs", stk_msgs),
                             ("stk_msgs.msg", stk_msgs_msg),
                             ("std_msgs", std_msgs),
                             ("std_msgs.msg", std_msgs_msg)):
            monkeypatch.setitem(sys.modules, name, module)
        monkeypatch.setattr(deploy_run_cli.time, "monotonic",
                            lambda: fake.now)

    def deliver(self, topic, message):
        self.subscriptions[topic](message)


def _idle(fake):
    deploy_run_cli._check_graph_idle("/stk")


def _bridge(fake):
    return deploy_run_cli._wait_for_bridge_connected_ros("/stk", 2.0, "0")


def _ack(fake):
    return deploy_run_cli._wait_for_twin_ack_ros("/stk", 2.0)


def _identity(fake):
    return deploy_run_cli._wait_for_twin_identity_ros(2.0)


WAITS = [_idle, _bridge, _ack, _identity]


@pytest.mark.parametrize("wait", WAITS, ids=lambda w: w.__name__)
def test_each_wait_uses_its_own_context_with_no_signal_handlers(
        monkeypatch, wait):
    fake = FakeRos(monkeypatch)
    wait(fake)
    inits = [call for call in fake.calls if call[0] == "init"]
    assert len(inits) == 1
    _, context, handlers = inits[0]
    assert context is not None and context is fake.contexts[0]
    assert handlers == NO_HANDLERS
    created = [call for call in fake.calls if call[0] == "create_node"]
    assert created and created[0][2] is context
    assert ("executor", context) in fake.calls
    assert ("try_shutdown", context) in fake.calls
    assert not [call for call in fake.calls if call[0] == "shutdown"]


@pytest.mark.parametrize("wait", WAITS, ids=lambda w: w.__name__)
def test_the_waits_own_error_survives_a_context_shut_down_under_it(
        monkeypatch, wait):
    """The idle_check failure: the context was shut down during the
    poll, and the cleanup must not replace the original error."""
    def shut_down_then_fail(fake):
        fake.contexts[0].shut_down = True
        raise RuntimeError("the wait's own error")

    fake = FakeRos(monkeypatch, on_spin=shut_down_then_fail)
    with pytest.raises(RuntimeError, match="the wait's own error"):
        wait(fake)
    assert ("try_shutdown", fake.contexts[0]) in fake.calls


@pytest.mark.parametrize("wait", WAITS, ids=lambda w: w.__name__)
def test_a_failed_init_is_reported_as_itself(monkeypatch, wait):
    fake = FakeRos(monkeypatch)

    def failing_init(**kwargs):
        raise OSError("rcl_init failed")

    monkeypatch.setattr(sys.modules["rclpy"], "init", failing_init)
    with pytest.raises(OSError, match="rcl_init failed"):
        wait(fake)


def test_the_idle_check_still_refuses_a_busy_graph(monkeypatch):
    fake = FakeRos(monkeypatch, publishers=1)
    with pytest.raises(RuntimeError, match="Another run is active"):
        _idle(fake)
    assert ("try_shutdown", fake.contexts[0]) in fake.calls


def test_the_bridge_wait_still_sees_connected(monkeypatch):
    def connect(fake):
        fake.deliver("/stk/status", fake.BridgeStatus(state=1))

    fake = FakeRos(monkeypatch, on_spin=connect)
    result = _bridge(fake)
    assert result.connected and result.backend_type == "com"


def test_the_identity_wait_still_returns_the_text(monkeypatch):
    from acav.twin_identity import IDENTITY_TOPIC

    def publish(fake):
        fake.deliver(IDENTITY_TOPIC, types.SimpleNamespace(data="v3 x"))

    fake = FakeRos(monkeypatch, on_spin=publish)
    assert _identity(fake) == "v3 x"


# --- execute_run records the original exception ----------------------------

def _summary(tmp_path):
    return json.loads((tmp_path / "run.logs" / "summary.json")
                      .read_text(encoding="utf-8"))


def _deps(**overrides):
    from types import SimpleNamespace

    recorder = SimpleNamespace(close=lambda: None, error=None)
    values = dict(check_idle=lambda prefix: None,
                  start_recorder=lambda *a: recorder,
                  popen=lambda *a, **k: FakeProcess(),
                  wait_for_bridge_connected=lambda *a: ConnectResult(True,
                                                                     "com"),
                  wait_for_twin_ack=lambda *a: True,
                  prompt=lambda message: None)
    values.update(overrides)
    return Deps(**values)


def test_an_idle_check_error_is_recorded_with_its_type(tmp_path):
    def broken(prefix):
        raise ValueError("the original cause")

    assert execute_run(_config(tmp_path), _deps(check_idle=broken)) == 1
    summary = _summary(tmp_path)
    assert summary["steps"][0] == {"step": "idle_check", "ok": False,
                                   "detail": "the original cause"}
    error, = summary["errors"]
    assert error["step"] == "idle_check"
    assert error["type"] == "ValueError"
    assert error["message"] == "the original cause"
    assert "ValueError: the original cause" in error["traceback"]


@pytest.mark.parametrize("step, field", [
    ("bridge_connected", "wait_for_bridge_connected"),
    ("twin_acking", "wait_for_twin_ack"),
    ("twin_identity", "confirm_twin_identity"),
])
def test_an_error_in_a_later_wait_is_a_recorded_failed_step(
        tmp_path, step, field):
    def broken(*args):
        raise RuntimeError(f"{step} broke")

    assert execute_run(_config(tmp_path), _deps(**{field: broken})) == 1
    summary = _summary(tmp_path)
    assert summary["steps"][-1]["step"] in (step, "topic_capture")
    failed = [entry for entry in summary["steps"] if entry["step"] == step]
    assert len(failed) == 1 and failed[0]["ok"] is False
    # bridge_connected adds where the bridge's startup had got to.
    assert failed[0]["detail"].startswith(f"{step} broke")
    error, = summary["errors"]
    assert (error["step"], error["type"]) == (step, "RuntimeError")


def test_an_interrupt_is_recorded_and_still_raised(tmp_path):
    def interrupted(*args):
        raise KeyboardInterrupt()

    with pytest.raises(KeyboardInterrupt):
        execute_run(_config(tmp_path),
                    _deps(wait_for_twin_ack=interrupted))
    error, = _summary(tmp_path)["errors"]
    assert error["type"] == "KeyboardInterrupt"
    assert error["step"] == "run"


def test_a_clean_run_records_no_errors(tmp_path):
    assert execute_run(_config(tmp_path), _deps()) == 0
    assert _summary(tmp_path)["errors"] == []
