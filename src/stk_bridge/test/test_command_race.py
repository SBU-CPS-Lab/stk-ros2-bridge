"""The command/ack race, and what STK reports when it is lost.

Found on a live E6 run. The twin publishes its
attitude command and its lockstep ack on two topics; ROS 2 does not order
delivery across topics, so on ~7% of steps the ack released the barrier
first and the bridge sensed the next step before the command had landed.
STK, holding no sample at that time, reported its BASIC attitude -- a jump
of up to 180 deg -- and the twin, which then integrated from that jump
(and from the body rate STK derived across it), spun up to ~800 deg/s.
"""

import logging
import math
import threading
import time
from types import SimpleNamespace

from stk_bridge.backends.base import StkTime
from stk_bridge.backends.mock_backend import MockBackend
from stk_bridge.core import conversions as conv
from stk_bridge.core import data_providers as dp
from stk_bridge.core.actuation import ActuationDispatcher
from stk_bridge.core.frames import FrameRegistry
from stk_bridge.core.time_sync import ExecutionMode
from stk_bridge.interfaces.ros_subscribers import AttitudeCommand, CommandInbox
from stk_bridge.nodes.bridge_node import StkBridgeNode

SAT = "*/Satellite/SSS-1"
EPOCH = 1717237800.0
Q = conv.normalize_quaternion((0.2, -0.4, 0.1, 0.9))


def _command(t_s, q=Q):
    return AttitudeCommand(object_path=SAT, time=StkTime(EPOCH + t_s),
                           quaternion_xyzw=q, inertial=True)


def _put_later(inbox, command, delay_s):
    def run():
        time.sleep(delay_s)
        inbox.put("attitude", command)
    thread = threading.Thread(target=run)
    thread.start()
    return thread


class TestInboxWait:
    def test_a_command_already_there_returns_at_once(self):
        inbox = CommandInbox()
        inbox.put("attitude", _command(10.1))
        assert inbox.wait_for("attitude", SAT, EPOCH + 10.05, timeout_s=0.0)

    def test_it_waits_for_a_command_that_lands_after_the_ack(self):
        inbox = CommandInbox()
        thread = _put_later(inbox, _command(10.1), 0.05)
        started = time.monotonic()
        assert inbox.wait_for("attitude", SAT, EPOCH + 10.05, timeout_s=2.0)
        assert time.monotonic() - started < 1.0
        thread.join()

    def test_an_older_command_does_not_satisfy_it(self):
        inbox = CommandInbox()
        inbox.put("attitude", _command(10.0))       # this step's, not next
        assert not inbox.wait_for("attitude", SAT, EPOCH + 10.05,
                                  timeout_s=0.05)

    def test_it_gives_up_at_the_timeout(self):
        started = time.monotonic()
        assert not CommandInbox().wait_for("attitude", SAT, EPOCH + 1.0,
                                           timeout_s=0.1)
        assert 0.09 <= time.monotonic() - started < 1.0


def _node(inbox, enabled=True, wait_s=2.0):
    node = StkBridgeNode.__new__(StkBridgeNode)
    node.config = {"time": {"await_attitude_commands": enabled,
                            "command_wait_s": wait_s}}
    node.inbox = inbox
    node._running = threading.Event()
    node._running.set()
    node._commanded_paths = {SAT}
    node._commands_missing = 0
    node._attitude_mismatches = 0
    node._warn_counts = {}
    node.time_sync = SimpleNamespace(
        mode=ExecutionMode.LOCKSTEP, config=SimpleNamespace(step_s=0.1),
        epoch=StkTime(EPOCH))
    return node


class TestBridgeWaitsForTheNextCommand:
    def test_a_late_command_is_waited_for_not_missed(self, caplog):
        inbox = CommandInbox()
        node = _node(inbox)
        thread = _put_later(inbox, _command(10.1), 0.05)
        with caplog.at_level(logging.WARNING):
            node._await_attitude_commands(StkTime(EPOCH + 10.0))
        thread.join()
        assert node._commands_missing == 0 and caplog.text == ""
        assert [c.time.epoch_seconds for _, c in inbox.drain()] == [EPOCH + 10.1]

    def test_a_command_that_never_comes_is_reported(self, caplog):
        node = _node(CommandInbox(), wait_s=0.05)
        with caplog.at_level(logging.WARNING):
            node._await_attitude_commands(StkTime(EPOCH + 10.0))
        assert node._commands_missing == 1
        assert "basic attitude" in caplog.text

    def test_disabled_it_does_not_wait(self):
        node = _node(CommandInbox(), enabled=False, wait_s=5.0)
        started = time.monotonic()
        node._await_attitude_commands(StkTime(EPOCH + 10.0))
        assert time.monotonic() - started < 0.5

    def test_objects_never_commanded_are_not_waited_for(self):
        node = _node(CommandInbox(), wait_s=5.0)
        node._commanded_paths = set()
        started = time.monotonic()
        node._await_attitude_commands(StkTime(EPOCH + 0.1))
        assert time.monotonic() - started < 0.5


class TestWhatStkReportsWhenTheCommandIsMissing:
    """The mock now reproduces it: past the newest sample, basic attitude."""

    def test_sensing_before_the_command_lands_reads_the_basic_attitude(self):
        backend = MockBackend()
        backend.connect()
        backend.new_scenario("Race")
        epoch = backend.get_scenario_epoch()
        dispatcher = ActuationDispatcher(backend, FrameRegistry(),
                                         default_attitude_inertial=True)
        assert dispatcher.set_attitude(SAT, epoch + 10.0, Q)

        def read(t):
            data, m = dp.execute(backend, dp.ATTITUDE_QUATERNION, SAT,
                                 start=t, stop=t)
            return conv.quaternion_from_stk_elements(
                *(dp.first_value(data, dp.ATTITUDE_QUATERNION, m, k)
                  for k in ("q1", "q2", "q3", "q4")))

        # The next step's command has not landed: STK reports its basic
        # attitude, far from anything the controller commanded.
        jump = math.degrees(conv.quaternion_angle_between(Q, read(epoch + 10.1)))
        assert jump > 30.0
        # With the command in place first, it reads back exactly.
        assert dispatcher.set_attitude(SAT, epoch + 10.1, Q)
        assert math.degrees(conv.quaternion_angle_between(Q, read(epoch + 10.1))) < 1e-6


class TestTheBridgeSaysWhenStkIsNotWhereItWasPut:
    def _node(self, written):
        node = _node(CommandInbox())
        node.frames = FrameRegistry()
        node.actuation = SimpleNamespace(last_attitude=lambda path: written)
        return node

    def _samples(self, q):
        return {"attitude": SimpleNamespace(frame="icrf", quaternion_xyzw=q)}

    def test_as_written_is_quiet(self, caplog):
        node = self._node((EPOCH + 10.1, Q, True))
        with caplog.at_level(logging.WARNING):
            node._check_attitude_landed(SAT, StkTime(EPOCH + 10.1), self._samples(Q))
        assert caplog.text == "" and node._attitude_mismatches == 0

    def test_nothing_written_for_this_step_is_reported(self, caplog):
        node = self._node((EPOCH + 10.0, Q, True))     # last write: a step ago
        with caplog.at_level(logging.WARNING):
            node._check_attitude_landed(SAT, StkTime(EPOCH + 10.1),
                                        self._samples((0.0, 0.0, 0.0, 1.0)))
        assert node._attitude_mismatches == 1
        assert "no attitude was written" in caplog.text

    def test_a_read_back_that_differs_is_reported(self, caplog):
        node = self._node((EPOCH + 10.1, Q, True))
        with caplog.at_level(logging.WARNING):
            node._check_attitude_landed(SAT, StkTime(EPOCH + 10.1),
                                        self._samples((0.0, 0.0, 0.0, 1.0)))
        assert node._attitude_mismatches == 1
        assert "deg away" in caplog.text
