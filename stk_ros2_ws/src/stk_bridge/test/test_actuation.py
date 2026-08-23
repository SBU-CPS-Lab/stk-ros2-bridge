"""Unit tests for core/actuation.py -- the monotonicity guard and
command ordering that survive STK 12.6's NACK-on-out-of-order finding."""

import pytest

from stk_bridge.backends.mock_backend import MockBackend
from stk_bridge.core.actuation import ActuationDispatcher
from stk_bridge.core.frames import FrameRegistry
from stk_bridge.exceptions import FrameResolutionError

SAT = "*/Satellite/SSS-1"


@pytest.fixture
def backend():
    b = MockBackend()
    b.connect()
    b.new_scenario("Test")
    yield b
    b.disconnect()


@pytest.fixture
def dispatcher(backend):
    d = ActuationDispatcher(backend, FrameRegistry())
    d.prepare_object(SAT)
    return d


class TestMonotonicityGuard:
    def test_local_guard_prevents_the_stk_round_trip(self, backend, dispatcher):
        """The guard's real value: it must reject BEFORE reaching the
        backend, not just report the same failure STK would."""
        epoch = backend.get_scenario_epoch()
        assert dispatcher.set_attitude(SAT, epoch + 10.0, (0, 0, 0, 1))
        assert not dispatcher.set_attitude(SAT, epoch + 5.0, (0, 0, 0, 1))

        stats = dispatcher.statistics()[SAT]["attitude"]
        assert stats["accepted"] == 1
        assert stats["dropped_non_monotonic"] == 1
        assert stats["rejected_by_stk"] == 0, (
            "a locally-caught violation must never reach the backend"
        )

    def test_equal_timestamp_also_rejected(self, backend, dispatcher):
        epoch = backend.get_scenario_epoch()
        assert dispatcher.set_attitude(SAT, epoch + 10.0, (0, 0, 0, 1))
        assert not dispatcher.set_attitude(SAT, epoch + 10.0, (0, 0, 0, 1))

    def test_position_and_attitude_guards_are_independent(
            self, backend, dispatcher):
        epoch = backend.get_scenario_epoch()
        assert dispatcher.set_position(SAT, epoch + 1.0, "icrf",
                                       (7.0e6, 0, 0))
        # Attitude at an earlier timestamp than position must still be
        # accepted -- the two channels have independent monotonicity.
        assert dispatcher.set_attitude(SAT, epoch + 0.5, (0, 0, 0, 1))

    def test_reset_clears_local_guard_bookkeeping(self, backend, dispatcher):
        """dispatcher.reset() forgets the dispatcher's OWN last-stamp
        bookkeeping -- it does not, by itself, clear STK-side real-time
        data (that's backend.clear_attitude_data, which
        ScenarioManager.reset() calls alongside this, mirroring
        ResetScenario's service handler). Tested in isolation with the
        backend-side data cleared explicitly, matching real call order.
        """
        epoch = backend.get_scenario_epoch()
        dispatcher.set_attitude(SAT, epoch + 10.0, (0, 0, 0, 1))

        backend.clear_attitude_data(SAT)
        dispatcher.reset(SAT)

        # After BOTH clears, an earlier timestamp is accepted again.
        assert dispatcher.set_attitude(SAT, epoch + 1.0, (0, 0, 0, 1))

    def test_reset_alone_does_not_clear_stk_side_data(self, backend, dispatcher):
        """The gap the above test deliberately avoids: reset() alone is
        NOT sufficient, because the backend still remembers its own last
        accepted timestamp. Documented here so it cannot regress into a
        silent assumption."""
        epoch = backend.get_scenario_epoch()
        dispatcher.set_attitude(SAT, epoch + 10.0, (0, 0, 0, 1))
        dispatcher.reset(SAT)   # local bookkeeping only

        # Local guard would now accept an earlier stamp, but STK itself
        # (the mock, faithfully) still rejects it.
        assert not dispatcher.set_attitude(SAT, epoch + 1.0, (0, 0, 0, 1))
        stats = dispatcher.statistics()[SAT]["attitude"]
        assert stats["rejected_by_stk"] == 1


class TestAngularVelocityOrdering:
    def test_rate_before_any_attitude_is_rejected_locally(
            self, backend, dispatcher):
        epoch = backend.get_scenario_epoch()
        ok = dispatcher.set_angular_velocity(SAT, epoch + 1.0,
                                             (0.0, 0.0, 0.1))
        assert not ok
        stats = dispatcher.statistics()[SAT]["angular_rate"]
        assert "no prior attitude" in stats["last_error"]

    def test_rate_after_attitude_succeeds(self, backend, dispatcher):
        epoch = backend.get_scenario_epoch()
        dispatcher.set_attitude(SAT, epoch + 1.0, (0, 0, 0, 1))
        assert dispatcher.set_angular_velocity(SAT, epoch + 2.0,
                                               (0.0, 0.0, 0.1))


class TestFrameValidation:
    def test_lvlh_attitude_command_rejected_before_reaching_backend(
            self, backend, dispatcher):
        epoch = backend.get_scenario_epoch()
        with pytest.raises(FrameResolutionError):
            dispatcher.set_attitude(SAT, epoch + 1.0, (0, 0, 0, 1),
                                    frame_id="lvlh")

    def test_default_inertial_flag_used_when_no_frame_given(
            self, backend, dispatcher):
        epoch = backend.get_scenario_epoch()
        # default_attitude_inertial=False by construction -> CBFQuat path
        assert dispatcher.set_attitude(SAT, epoch + 1.0, (0, 0, 0, 1))


class TestQuaternionValidation:
    def test_non_unit_quaternion_normalised_not_rejected_at_dispatcher(
            self, backend, dispatcher):
        """Unlike the raw backend (which rejects non-unit input outright
        as a defensive check), the dispatcher normalises first -- a
        controller publishing slightly-denormalised floats due to
        accumulation error should not have every command dropped."""
        epoch = backend.get_scenario_epoch()
        assert dispatcher.set_attitude(SAT, epoch + 1.0,
                                       (0.0, 0.0, 0.70715, 0.70706))

    def test_zero_quaternion_rejected(self, backend, dispatcher):
        epoch = backend.get_scenario_epoch()
        assert not dispatcher.set_attitude(SAT, epoch + 1.0,
                                           (0.0, 0.0, 0.0, 0.0))
