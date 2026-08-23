"""Unit tests for backends/mock_backend.py.

These tests are unusual: they verify the mock is BAD in the same ways
real STK is bad. A mock that is kinder than production defeats its
purpose -- see mock_backend.py's module docstring, and the Phase 3/4
history where an overly forgiving mock let real bugs through.
"""

import math

import pytest

from stk_bridge.backends.base import (
    DataProviderQuery,
    PropagatorType,
    StkTime,
)
from stk_bridge.backends.mock_backend import MockBackend
from stk_bridge.core import conversions as conv
from stk_bridge.exceptions import CommandError, ObjectNotFoundError, QueryError

SAT = "*/Satellite/SSS-1"


@pytest.fixture
def backend():
    b = MockBackend()
    b.connect()
    b.new_scenario("Test")
    yield b
    b.disconnect()


class TestScenarioLifecycle:
    def test_default_satellite_created_on_new_scenario(self, backend):
        assert backend.object_exists(SAT)

    def test_nominal_orbit_altitude_is_500km(self, backend):
        epoch = backend.get_scenario_epoch()
        data = backend.query_data_provider(DataProviderQuery(
            object_path=SAT, provider_name="Cartesian Position",
            group_element="ICRF", start=epoch, stop=epoch))
        x, y, z = (data.columns[k][0] for k in "xyz")
        radius = math.sqrt(x * x + y * y + z * z)
        altitude_km = (radius - 6378137.0) / 1000.0
        assert altitude_km == pytest.approx(500.0, abs=1.0)


class TestGroupedProviderGuard:
    """Real STK raises 'grouped data provider, no group_element given'
    (see com_backend._select_group_member). The mock must reproduce this
    or a code path relying on it would only fail against real STK."""

    def test_missing_group_element_on_grouped_provider_raises(self, backend):
        epoch = backend.get_scenario_epoch()
        with pytest.raises(QueryError):
            backend.query_data_provider(DataProviderQuery(
                object_path=SAT, provider_name="Cartesian Position",
                start=epoch, stop=epoch))   # no group_element


class TestRealtimePropagatorEphemerisCliff:
    """Regression test for the Phase 3 bring-up failure: switching to
    RealTime DISCARDS analytic ephemeris, so a query before any sample
    is injected returns zero rows, not extrapolated Kepler state."""

    def test_switching_to_realtime_clears_existing_ephemeris(self, backend):
        epoch = backend.get_scenario_epoch()
        backend.set_propagator(SAT, PropagatorType.REALTIME)
        data = backend.query_data_provider(DataProviderQuery(
            object_path=SAT, provider_name="Cartesian Position",
            group_element="ICRF", start=epoch, stop=epoch))
        assert data.row_count == 0, (
            "a freshly-realtime object must have NO ephemeris until a "
            "sample is injected -- this is what makes the mock catch the "
            "empty-data-provider bug found on real STK 12.6"
        )

    def test_after_injection_state_exists_near_the_sample(self, backend):
        epoch = backend.get_scenario_epoch()
        backend.set_propagator(SAT, PropagatorType.REALTIME)
        backend.set_position(SAT, epoch + 10.0, "ICRF", (7.0e6, 0.0, 0.0))
        data = backend.query_data_provider(DataProviderQuery(
            object_path=SAT, provider_name="Cartesian Position",
            group_element="ICRF", start=epoch + 10.0, stop=epoch + 10.0))
        assert data.row_count == 1


class TestAttitudeFrameDistinction:
    """Regression test for the CBFQuat<->ECF pairing bug found on real
    STK 12.6: an inertial-referenced read of a fixed-commanded attitude
    must NOT equal the commanded value."""

    def test_cbfquat_read_via_inertial_provider_disagrees(self, backend):
        epoch = backend.get_scenario_epoch()
        commanded = conv.normalize_quaternion((0.0692, 0.1383, 0.2075, 0.9659))
        backend.enable_realtime_attitude(SAT)
        backend.add_attitude_quaternion(SAT, epoch + 500.0, commanded,
                                        inertial=False)

        wrong = backend.query_data_provider(DataProviderQuery(
            object_path=SAT, provider_name="Attitude Quaternions",
            start=epoch + 500.0, stop=epoch + 500.0))
        wrong_q = conv.quaternion_from_stk_elements(
            *[wrong.columns[f"q{i}"][0] for i in (1, 2, 3, 4)])
        error_deg = math.degrees(
            conv.quaternion_angle_between(commanded, wrong_q))
        assert error_deg > 0.01, (
            "the mock must disagree between fixed and inertial read-back, "
            "or it cannot catch a real command/provider frame mismatch"
        )

    def test_cbfquat_read_via_ecf_provider_agrees(self, backend):
        epoch = backend.get_scenario_epoch()
        commanded = conv.normalize_quaternion((0.0692, 0.1383, 0.2075, 0.9659))
        backend.enable_realtime_attitude(SAT)
        backend.add_attitude_quaternion(SAT, epoch + 500.0, commanded,
                                        inertial=False)

        right = backend.query_data_provider(DataProviderQuery(
            object_path=SAT, provider_name="ECF Attitude Quaternions",
            start=epoch + 500.0, stop=epoch + 500.0))
        right_q = conv.quaternion_from_stk_elements(
            *[right.columns[f"q{i}"][0] for i in (1, 2, 3, 4)])
        error_deg = math.degrees(
            conv.quaternion_angle_between(commanded, right_q))
        # 1e-4 deg, not 1e-6: normalize_quaternion + the mock's own
        # ECI<->ECF rotation composition both round through float64, and
        # this threshold is still four orders of magnitude tighter than
        # anything that would matter for attitude control.
        assert error_deg < 1e-4


class TestMonotonicityAndOrdering:
    def test_out_of_order_attitude_sample_raises(self, backend):
        """Confirmed on live STK 12.6: out-of-order attitude is NACKed,
        not silently dropped. The mock matches this."""
        epoch = backend.get_scenario_epoch()
        backend.enable_realtime_attitude(SAT)
        backend.add_attitude_quaternion(SAT, epoch + 10.0, (0, 0, 0, 1))
        with pytest.raises(CommandError):
            backend.add_attitude_quaternion(SAT, epoch + 5.0, (0, 0, 0, 1))

    def test_angvels_before_any_attitude_sample_raises(self, backend):
        epoch = backend.get_scenario_epoch()
        backend.enable_realtime_attitude(SAT)
        with pytest.raises(CommandError):
            backend.add_attitude_angular_velocity(
                SAT, epoch + 1.0, (0.0, 0.0, 0.1))

    def test_non_unit_quaternion_rejected(self, backend):
        epoch = backend.get_scenario_epoch()
        backend.enable_realtime_attitude(SAT)
        with pytest.raises(CommandError):
            backend.add_attitude_quaternion(SAT, epoch + 1.0,
                                            (1.0, 1.0, 1.0, 1.0))

    def test_attitude_without_enable_first_raises(self, backend):
        epoch = backend.get_scenario_epoch()
        with pytest.raises(CommandError):
            backend.add_attitude_quaternion(SAT, epoch + 1.0, (0, 0, 0, 1))


class TestObjectNotFound:
    def test_unknown_object_raises(self, backend):
        with pytest.raises(ObjectNotFoundError):
            backend.query_data_provider(DataProviderQuery(
                object_path="*/Satellite/DoesNotExist",
                provider_name="Cartesian Position", group_element="ICRF"))


class TestProviderInventory:
    def test_list_data_providers_matches_query_dispatch(self, backend):
        """The provider names list_data_providers() advertises must be
        exactly the ones query_data_provider() can serve -- a mismatch
        would let discovery pass while the actual query fails."""
        providers = backend.list_data_providers(SAT)
        epoch = backend.get_scenario_epoch()
        for name, members in providers.items():
            group = members[0] if members else None
            data = backend.query_data_provider(DataProviderQuery(
                object_path=SAT, provider_name=name, group_element=group,
                start=epoch, stop=epoch))
            assert data is not None   # must not raise

    def test_no_body_axes_angular_velocity_provider(self, backend):
        """Confirmed on a live STK 12.6 inventory: this provider does
        not exist. Rates come from Attitude Segment Quaternions."""
        assert "Body Axes Angular Velocity" not in backend.list_data_providers(SAT)
