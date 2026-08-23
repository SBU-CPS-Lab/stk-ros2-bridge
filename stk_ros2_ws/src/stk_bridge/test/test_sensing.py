"""Unit tests for core/sensing.py."""

import pytest

from stk_bridge.backends.base import PropagatorType
from stk_bridge.backends.mock_backend import MockBackend
from stk_bridge.core.frames import FrameRegistry
from stk_bridge.core.sensing import SensingConfig, SensingPipeline
from stk_bridge.core.validation import ValidationReport, ConventionValidator

SAT = "*/Satellite/SSS-1"


@pytest.fixture
def backend():
    b = MockBackend()
    b.connect()
    b.new_scenario("Test")
    yield b
    b.disconnect()


@pytest.fixture
def report(backend):
    return ConventionValidator(backend).validate()


class TestProviderSelection:
    def test_uses_validated_attitude_provider(self, backend, report):
        pipeline = SensingPipeline(backend, FrameRegistry(),
                                   SensingConfig(satellites=[SAT]), report)
        assert pipeline._attitude_spec.provider_name == report.attitude_provider

    def test_falls_back_when_no_report_given(self, backend):
        pipeline = SensingPipeline(backend, FrameRegistry(),
                                   SensingConfig(satellites=[SAT]))
        assert pipeline._attitude_spec.provider_name == \
            "ECF Attitude Quaternions"


class TestOrbitSensing:
    def test_read_orbit_on_default_propagator(self, backend, report):
        pipeline = SensingPipeline(backend, FrameRegistry(),
                                   SensingConfig(satellites=[SAT]), report)
        epoch = backend.get_scenario_epoch()
        sample = pipeline.read_orbit(SAT, epoch)
        assert sample is not None
        assert sample.position_m[0] > 6_000_000   # sane LEO magnitude
        assert not sample.is_extrapolated

    def test_realtime_object_flags_extrapolated(self, backend, report):
        pipeline = SensingPipeline(backend, FrameRegistry(),
                                   SensingConfig(satellites=[SAT]), report)
        backend.set_propagator(SAT, PropagatorType.REALTIME)
        epoch = backend.get_scenario_epoch()
        backend.set_position(SAT, epoch + 1.0, "ICRF", (7.0e6, 0, 0))
        sample = pipeline.read_orbit(SAT, epoch + 1.0)
        assert sample is not None
        assert sample.is_extrapolated


class TestStalenessVsFailure:
    """The core distinction sensing.py makes: an empty read on a
    realtime object is STALE (returns None), everywhere else it's an
    ERROR (raises)."""

    def test_realtime_object_before_any_sample_is_stale_not_error(
            self, backend, report):
        pipeline = SensingPipeline(backend, FrameRegistry(),
                                   SensingConfig(satellites=[SAT]), report)
        backend.set_propagator(SAT, PropagatorType.REALTIME)
        epoch = backend.get_scenario_epoch()
        sample = pipeline.read_orbit(SAT, epoch)   # before any injection
        assert sample is None   # stale, not an exception

        health = pipeline.health()["orbit_position"]
        assert health.stale_reads >= 1
        assert health.errors == 0

    def test_stale_read_never_returns_frozen_old_data(self, backend, report):
        """The dangerous alternative this design avoids: republishing a
        last-known value that looks fresh. Uses explicit invalidation
        (the SetPropagator service's own path) rather than relying on
        the propagator-cache TTL, so this test is not timing-sensitive.
        """
        pipeline = SensingPipeline(backend, FrameRegistry(),
                                   SensingConfig(satellites=[SAT]), report)
        epoch = backend.get_scenario_epoch()

        first = pipeline.read_orbit(SAT, epoch)
        assert first is not None

        backend.set_propagator(SAT, PropagatorType.REALTIME)  # wipes ephemeris
        pipeline.invalidate_propagator_cache(SAT)  # what SetPropagator does
        second = pipeline.read_orbit(SAT, epoch)
        assert second is None, "must not silently return the pre-switch state"

    def test_propagator_cache_has_a_bounded_ttl_not_permanent_staleness(
            self, backend, report):
        """Regression guard for the gap this test file originally found:
        an out-of-band propagator switch with NO explicit invalidation
        must still be reflected within a bounded time, not never. See
        SensingPipeline._is_realtime's docstring on why a permanent
        cache is unsafe."""
        pipeline = SensingPipeline(backend, FrameRegistry(),
                                   SensingConfig(satellites=[SAT]), report)
        epoch = backend.get_scenario_epoch()

        assert pipeline.read_orbit(SAT, epoch) is not None
        assert pipeline._is_realtime(SAT) is False   # cached: not real-time

        backend.set_propagator(SAT, PropagatorType.REALTIME)
        # No explicit invalidate_propagator_cache() call here -- this is
        # exactly the out-of-band case the TTL exists to bound.
        pipeline._REALTIME_CACHE_TTL_S = 0.0   # force immediate expiry
        assert pipeline._is_realtime(SAT) is True, (
            "an expired cache entry must be re-checked, not trusted forever"
        )


class TestAttitudeSensing:
    def test_read_attitude_round_trips_through_mock(self, backend, report):
        pipeline = SensingPipeline(backend, FrameRegistry(),
                                   SensingConfig(satellites=[SAT]), report)
        backend.enable_realtime_attitude(SAT)
        epoch = backend.get_scenario_epoch()
        commanded = (0.0, 0.0, 0.7071, 0.7071)
        backend.add_attitude_quaternion(SAT, epoch + 1.0, commanded,
                                        inertial=False)
        sample = pipeline.read_attitude(SAT, epoch + 1.0)
        assert sample is not None
        for a, b in zip(sample.quaternion_xyzw, commanded):
            assert a == pytest.approx(b, abs=1e-3)


class TestAngularRateScaling:
    def test_rate_uses_measured_scale_from_report(self, backend, report):
        pipeline = SensingPipeline(backend, FrameRegistry(),
                                   SensingConfig(satellites=[SAT]), report)
        assert pipeline.angle_rate_scale.measured
