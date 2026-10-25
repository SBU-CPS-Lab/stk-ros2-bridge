"""sensing.orbit_batch_samples: lockstep orbit reads from audited blocks.

A block is two STK interval queries (Cartesian Position and Velocity) over
the next N lockstep epochs, read with core/orbit_block.py unchanged. It is
used only if its UTCG grid, row counts and values validate and both of
its ends agree with instantaneous STK reads within the strict tolerances
(1e-6 m, 1e-9 m/s by default). Otherwise those epochs are read
instantaneously, as without batching. Rows are cached per object and frame,
for one backend revision only; the terminal epoch is never batched.
"""

import pytest

from stk_bridge.backends.base import StkTime
from stk_bridge.core.frames import FrameRegistry
from stk_bridge.core.orbit_batch import (
    DEFAULT_AUDIT_POSITION_M,
    DEFAULT_AUDIT_VELOCITY_MPS,
    OrbitBatch,
    stop_time_problem,
)
from stk_bridge.core.orbit_block import epoch_ms
from stk_bridge.core.sensing import SensingConfig, SensingPipeline

from test_equivalence_diagnostic import (   # noqa: I100 - shared fake
    EPOCH,
    SAT,
    STOP,
    FakeBackend,
    _epochs,
    _report,
    _samples_as_values,
)


class Backend(FakeBackend):
    propagator = "ePropagatorHPOP"

    def get_propagator_type(self, object_path):
        return self.propagator


def _pipeline(backend, samples=100, **batch):
    pipeline = SensingPipeline(backend, FrameRegistry(),
                               SensingConfig(satellites=[SAT]), _report())
    pipeline.attach_orbit_batch(OrbitBatch(
        samples=samples, step_s=0.1, stop_time=StkTime(STOP), **batch))
    return pipeline


def _orbit_queries(backend):
    return [q for q in backend.queries
            if q.provider_name in ("Cartesian Position",
                                   "Cartesian Velocity")]


class TestConfiguration:
    def test_defaults(self):
        config = SensingConfig()
        assert config.orbit_batch_samples == 1
        assert config.orbit_batch_audit_position_m == 1e-6
        assert config.orbit_batch_audit_velocity_mps == 1e-9
        assert DEFAULT_AUDIT_POSITION_M == 1e-6
        assert DEFAULT_AUDIT_VELOCITY_MPS == 1e-9

    def test_read_from_the_sensing_section(self):
        config = SensingConfig.from_config({"sensing": {
            "orbit_batch_samples": 100,
            "orbit_batch_audit_position_m": 1e-5,
            "orbit_batch_audit_velocity_mps": 1e-8,
            "orbit_batch_expected_window_s": 720.0}})
        assert config.orbit_batch_samples == 100
        assert config.orbit_batch_audit_position_m == 1e-5
        assert config.orbit_batch_audit_velocity_mps == 1e-8
        assert config.orbit_batch_expected_window_s == 720.0

    @pytest.mark.parametrize("relative, expected", [
        (("stk_bridge", "config", "bridge_params.yaml"), 1),
        (("acav", "config", "bridge_params_acav.yaml"), 100)])
    def test_shipped_configs(self, relative, expected):
        import os
        import yaml
        src = os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))))
        with open(os.path.join(src, *relative), encoding="utf-8") as handle:
            settings = yaml.safe_load(handle)
        assert settings["sensing"]["orbit_batch_samples"] == expected


class TestServing:
    def test_six_orbit_queries_per_block_of_one_hundred(self):
        backend = Backend()
        pipeline = _pipeline(backend)
        for when in _epochs(200):
            pipeline.read_all(SAT, when)
        orbit = _orbit_queries(backend)
        interval = [q for q in orbit if not q.is_instantaneous]
        audit = [q for q in orbit if q.is_instantaneous]
        assert len(interval) == 2 * 2 and len(audit) == 4 * 2
        stats = pipeline.orbit_batch_statistics()
        assert stats["blocks_loaded"] == 2
        assert stats["audits_passed"] == 2
        assert stats["epochs_cached"] == 200
        assert stats["epochs_instantaneous"] == 0

    def test_the_audit_reads_both_ends_of_the_block(self):
        backend = Backend()
        pipeline = _pipeline(backend)
        first = _epochs(1)[0]
        pipeline.read_all(SAT, first)
        audit = [q for q in _orbit_queries(backend) if q.is_instantaneous]
        assert sorted({epoch_ms(q.start) for q in audit}) == [
            epoch_ms(first), epoch_ms(first + 99 * 0.1)]

    def test_served_values_equal_the_instantaneous_values(self):
        plain = SensingPipeline(Backend(), FrameRegistry(),
                                SensingConfig(satellites=[SAT]), _report())
        batched = _pipeline(Backend())
        for when in _epochs(150):
            assert (_samples_as_values(batched.read_all(SAT, when))
                    == _samples_as_values(plain.read_all(SAT, when)))


class TestAuditAndFallback:
    def test_a_block_beyond_the_strict_tolerance_is_not_used(self, caplog):
        backend = Backend(interval_dx_m=2e-6)
        pipeline = _pipeline(backend)
        for when in _epochs(100):
            pipeline.read_all(SAT, when)
        stats = pipeline.orbit_batch_statistics()
        assert stats["audits_failed"] == 1 and stats["fallback_blocks"] == 1
        assert stats["epochs_cached"] == 0
        assert stats["epochs_instantaneous"] == 100
        assert "orbit batch fallback" in caplog.text
        # The audit stops at its first failing end (row 0: 2 reads); then
        # the failed block's epochs are read instantaneously, once each.
        instantaneous = [q for q in _orbit_queries(backend)
                         if q.is_instantaneous]
        assert len(instantaneous) == 2 + 2 * 100

    def test_within_the_strict_tolerance_the_block_is_used(self):
        pipeline = _pipeline(Backend(interval_dx_m=5e-7))
        for when in _epochs(10):
            pipeline.read_all(SAT, when)
        assert pipeline.orbit_batch_statistics()["audits_passed"] == 1

    def test_the_tolerance_is_configurable(self):
        pipeline = _pipeline(Backend(interval_dx_m=2e-6),
                             audit_position_m=1e-5)
        for when in _epochs(10):
            pipeline.read_all(SAT, when)
        assert pipeline.orbit_batch_statistics()["audits_passed"] == 1

    def test_a_shifted_grid_falls_back(self):
        pipeline = _pipeline(Backend(interval_time_shift_ms=1))
        for when in _epochs(5):
            pipeline.read_all(SAT, when)
        stats = pipeline.orbit_batch_statistics()
        assert stats["validation_failed"] == 1 and stats["epochs_cached"] == 0

    def test_a_failed_interval_query_falls_back(self):
        pipeline = _pipeline(Backend(interval_fails=True))
        samples = pipeline.read_all(SAT, _epochs(1)[0])
        assert "orbit" in samples
        stats = pipeline.orbit_batch_statistics()
        assert stats["query_failed"] == 1
        assert stats["epochs_instantaneous"] == 1

    def test_after_a_failed_block_the_next_block_is_tried(self):
        backend = Backend(interval_dx_m=2e-6)
        pipeline = _pipeline(backend)
        epochs = _epochs(150)
        for when in epochs[:100]:
            pipeline.read_all(SAT, when)
        backend.interval_dx_m = 0.0
        for when in epochs[100:]:
            pipeline.read_all(SAT, when)
        stats = pipeline.orbit_batch_statistics()
        assert stats["fallback_blocks"] == 1 and stats["blocks_loaded"] == 1
        assert stats["epochs_cached"] == 50


class TestStopTimeAndTerminalEpoch:
    def test_blocks_never_reach_the_stop_time(self):
        backend = Backend()
        pipeline = _pipeline(backend)
        for when in _epochs(7200):                 # t = 0.1 .. 720.0
            pipeline.read_all(SAT, when)
        interval = [q for q in _orbit_queries(backend)
                    if not q.is_instantaneous]
        assert max(epoch_ms(q.stop) for q in interval) < epoch_ms(
            StkTime(STOP))
        stats = pipeline.orbit_batch_statistics()
        assert stats["epochs_cached"] == 7199
        assert stats["epochs_terminal"] == 1

    def test_the_cross_check_against_the_window(self):
        epoch = StkTime(EPOCH)
        assert stop_time_problem(epoch, StkTime(STOP), 720.0) is None
        assert stop_time_problem(epoch, StkTime(STOP), None) is None
        assert "720" in stop_time_problem(epoch, StkTime(STOP), 600.0)
        assert stop_time_problem(epoch, None, 720.0) is not None


class TestEligibility:
    @pytest.mark.parametrize("propagator", [
        "ePropagatorJ4Perturbation", "ePropagatorRealtime", None])
    def test_only_hpop_and_two_body_are_batched(self, propagator):
        backend = Backend()
        backend.propagator = propagator
        pipeline = _pipeline(backend)
        for when in _epochs(5):
            pipeline.read_all(SAT, when)
        assert all(q.is_instantaneous for q in backend.queries)
        stats = pipeline.orbit_batch_statistics()
        assert stats["epochs_unsupported"] == 5

    def test_two_body_is_batched(self):
        backend = Backend()
        backend.propagator = "ePropagatorTwoBody"
        pipeline = _pipeline(backend)
        pipeline.read_all(SAT, _epochs(1)[0])
        assert pipeline.orbit_batch_statistics()["blocks_loaded"] == 1

    def test_a_backend_without_a_revision_is_not_batched(self):
        backend = Backend()
        backend.revision = None
        pipeline = _pipeline(backend)
        for when in _epochs(3):
            pipeline.read_all(SAT, when)
        assert all(q.is_instantaneous for q in backend.queries)


class TestInvalidation:
    def test_a_revision_change_drops_the_block(self):
        backend = Backend()
        pipeline = _pipeline(backend)
        epochs = _epochs(20)
        for when in epochs[:10]:
            pipeline.read_all(SAT, when)
        backend.revision += 1
        for when in epochs[10:]:
            pipeline.read_all(SAT, when)
        stats = pipeline.orbit_batch_statistics()
        assert stats["invalidations_revision"] == 1
        assert stats["blocks_loaded"] == 2
        reloaded = [q for q in _orbit_queries(backend)
                    if not q.is_instantaneous][-1]
        assert epoch_ms(reloaded.start) == epoch_ms(epochs[10])

    def test_rows_are_kept_per_frame(self):
        backend = Backend()
        pipeline = _pipeline(backend)
        when = _epochs(1)[0]
        pipeline.read_orbit(SAT, when)
        pipeline.read_orbit(SAT, when, frame="fixed")
        assert pipeline.orbit_batch_statistics()["blocks_loaded"] == 2


def test_the_summary_line_names_every_counter():
    pipeline = _pipeline(Backend())
    for when in _epochs(10):
        pipeline.read_all(SAT, when)
    line = pipeline.orbit_batch_summary_line()
    assert line.startswith("orbit batch summary:")
    for text in ("blocks loaded 1", "audits passed 1", "failed 0",
                 "fallback blocks 0", "invalidations 0",
                 "epochs cached 10", "instantaneous 0"):
        assert text in line, text


def test_without_a_batch_the_summary_says_disabled():
    pipeline = SensingPipeline(Backend(), FrameRegistry(),
                               SensingConfig(satellites=[SAT]), _report())
    assert pipeline.orbit_batch_summary_line() == \
        "orbit batch summary: disabled"
