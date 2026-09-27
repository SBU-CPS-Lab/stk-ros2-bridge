"""Attitude frames, and attitude history left on a reused scenario.

Both were found on the second live E6 run (2026-09-27):

* The ACAV twin computes attitude in the inertial frame (ICRF position,
  velocity and target vectors), but the bridge read attitude back through
  the Earth-fixed provider and wrote the twin's commands with the
  Earth-fixed ``CBFQuat`` form. The numbers round-tripped unchanged, so
  the twin's own loop looked consistent, while STK's actual attitude was
  rotated by the Earth-rotation angle -- every target reported no access.
* STK keeps an object's real-time attitude samples across bridge runs on
  the same open scenario and rejects any sample earlier than the newest
  it holds, so a run on a reused scenario had every command rejected up
  to the time the previous attempt had reached.

The mock models the Earth rotation between the two quaternion forms and
keeps samples across ``enable_realtime_attitude`` exactly as STK does,
so both are reproducible here.
"""

import logging
import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from stk_bridge.backends.mock_backend import MockBackend
from stk_bridge.core import conversions as conv
from stk_bridge.core import data_providers as dp
from stk_bridge.core.actuation import ActuationDispatcher
from stk_bridge.core.frames import FrameRegistry
from stk_bridge.core.time_sync import ExecutionMode
from stk_bridge.core.validation import (
    ConventionValidator,
    ValidationReport,
    validate_backend,
)
from stk_bridge.exceptions import ConventionValidationError
from stk_bridge.nodes.bridge_node import StkBridgeNode, check_attitude_pairing

SAT = "*/Satellite/SSS-1"
Q_INERTIAL = conv.normalize_quaternion((0.1, -0.3, 0.5, 0.8))


@pytest.fixture
def backend():
    b = MockBackend()
    b.connect()
    b.new_scenario("Frames")
    yield b
    b.disconnect()


def _read(backend, spec, when):
    data, mapping = dp.execute(backend, spec, SAT, start=when, stop=when)
    return conv.quaternion_from_stk_elements(
        *(dp.first_value(data, spec, mapping, k) for k in ("q1", "q2", "q3", "q4")))


class TestValidationFollowsTheConfiguredCommandForm:
    def test_default_config_keeps_the_earth_fixed_pairing(self, backend):
        report = ConventionValidator(backend).validate()
        assert report.attitude_provider == dp.ECF_ATTITUDE_QUATERNION.provider_name
        assert report.attitude_command_inertial is False

    def test_inertial_config_wins_the_tie(self, backend):
        report = validate_backend(
            backend, "", {"actuation": {"default_attitude_inertial": True}})
        assert report.ok
        assert report.attitude_provider == dp.ATTITUDE_QUATERNION.provider_name
        assert report.attitude_command_inertial is True


class TestPairingGuard:
    def test_a_mismatch_refuses_to_start(self):
        report = ValidationReport(attitude_provider="ECF Attitude Quaternions",
                                  attitude_command_inertial=False)
        with pytest.raises(ConventionValidationError, match="Earth-rotation"):
            check_attitude_pairing(
                report, {"actuation": {"default_attitude_inertial": True}})

    def test_a_match_is_accepted(self):
        report = ValidationReport(attitude_provider="Attitude Quaternions",
                                  attitude_command_inertial=True)
        check_attitude_pairing(
            report, {"actuation": {"default_attitude_inertial": True}})

    def test_disabled_validation_is_not_second_guessed(self):
        check_attitude_pairing(
            ValidationReport(), {"actuation": {"default_attitude_inertial": True}})


class TestAnInertialControllerPointsWhereItMeans:
    """What the twin commands must be STK's actual inertial attitude."""

    def test_inertial_command_form_leaves_stk_where_commanded(self, backend):
        epoch = backend.get_scenario_epoch()
        dispatcher = ActuationDispatcher(backend, FrameRegistry(),
                                         default_attitude_inertial=True)
        assert dispatcher.set_attitude(SAT, epoch + 10.0, Q_INERTIAL)
        actual = _read(backend, dp.ATTITUDE_QUATERNION, epoch + 10.0)
        assert math.degrees(conv.quaternion_angle_between(Q_INERTIAL, actual)) \
            < 1e-6

    def test_the_old_earth_fixed_form_rotates_it_silently(self, backend):
        # The try2 configuration: an inertial controller's numbers written
        # as CBFQuat. They read back unchanged through the ECF provider --
        # the twin sees exactly what it sent -- while STK's inertial
        # attitude is off by the Earth-rotation angle.
        epoch = backend.get_scenario_epoch()
        dispatcher = ActuationDispatcher(backend, FrameRegistry(),
                                         default_attitude_inertial=False)
        assert dispatcher.set_attitude(SAT, epoch + 10.0, Q_INERTIAL)
        echoed = _read(backend, dp.ECF_ATTITUDE_QUATERNION, epoch + 10.0)
        actual = _read(backend, dp.ATTITUDE_QUATERNION, epoch + 10.0)
        assert math.degrees(conv.quaternion_angle_between(Q_INERTIAL, echoed)) \
            < 1e-6
        assert math.degrees(conv.quaternion_angle_between(Q_INERTIAL, actual)) \
            > 1.0


class TestAttitudeHistoryFromAnEarlierRun:
    def _earlier_run(self, backend, up_to_s):
        epoch = backend.get_scenario_epoch()
        backend.enable_realtime_attitude(SAT)
        t = 0.1
        while t <= up_to_s:
            backend.add_attitude_quaternion(SAT, epoch + t, Q_INERTIAL)
            t += 0.1
        return epoch

    def test_the_first_command_of_a_new_session_is_accepted(self, backend):
        epoch = self._earlier_run(backend, up_to_s=58.0)
        dispatcher = ActuationDispatcher(backend, FrameRegistry())
        assert dispatcher.set_attitude(SAT, epoch + 0.2, Q_INERTIAL)

    def test_without_the_clear_every_command_is_rejected(self, backend, caplog):
        # What try2 did: STK rejected commands up to t=58 s.
        epoch = self._earlier_run(backend, up_to_s=58.0)
        dispatcher = ActuationDispatcher(
            backend, FrameRegistry(), clear_attitude_on_first_command=False)
        with caplog.at_level(logging.WARNING):
            results = [dispatcher.set_attitude(SAT, epoch + 0.1 * k, Q_INERTIAL)
                       for k in range(2, 102)]
        assert not any(results)
        rejections = [r for r in caplog.records if "AddAttitude rejected" in r.getMessage()]
        assert len(rejections) == 3            # the 1st, 10th and 100th
        assert "earlier run" in rejections[0].getMessage()

    def test_validation_proves_the_clear_works(self, backend):
        report = ConventionValidator(backend).validate()
        check = next(r for r in report.results
                     if r.name == "attitude_history_clears")
        assert check.passed

    def test_validation_catches_a_clear_that_does_not_clear(self, backend):
        backend.clear_attitude_data = lambda path: None     # ACKs, does nothing
        report = ConventionValidator(backend).validate()
        check = next(r for r in report.results
                     if r.name == "attitude_history_clears")
        assert not check.passed
        assert not check.fatal                # non-fatal by default
        assert "REUSED scenario" in check.detail


class TestLockstepRetriesAMissingChannel:
    def _node(self, reads):
        node = StkBridgeNode.__new__(StkBridgeNode)
        node._warn_counts = {}
        node.time_sync = SimpleNamespace(
            mode=ExecutionMode.LOCKSTEP, epoch=conv.StkTime(1717237800.0))
        node.sensing = SimpleNamespace(
            FAST_CHANNELS=("orbit", "attitude", "angular_rate"))
        node.worker = Mock(call=lambda fn, label: reads.pop(0))
        return node

    def test_a_transient_miss_is_recovered(self):
        node = self._node([{"angular_rate": "rate"}])
        merged = node._retry_missing_channels(
            SAT, conv.StkTime(1717237800.0 + 590.1),
            {"orbit": "o", "attitude": "a"})
        assert set(merged) == {"orbit", "attitude", "angular_rate"}

    def test_a_persistent_miss_is_reported(self, caplog):
        node = self._node([{}])
        with caplog.at_level(logging.WARNING):
            merged = node._retry_missing_channels(
                SAT, conv.StkTime(1717237800.0 + 590.1),
                {"orbit": "o", "attitude": "a"})
        assert "angular_rate" not in merged
        assert "no angular_rate" in caplog.text and "will not ack" in caplog.text
