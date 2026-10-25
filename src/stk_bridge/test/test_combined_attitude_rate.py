"""(B) sensing.combine_attitude_rate: one segment query for attitude and rate.

'Attitude Segment Quaternions' already carries the quaternion the rate
read fetches. With the option on, the pipeline first proves, at an epoch
where the bridge wrote an inertial attitude sample, that the segment
quaternion equals 'Attitude Quaternions' (sign-aligned angle <= 1e-12
rad); only then does it read the segment provider once per step for
both channels. Until then, and whenever the proof fails or the backend's
structural revision changes, it reads exactly as before.
"""

import math

import pytest

from stk_bridge.backends.base import StkTime
from stk_bridge.core import data_providers as dp
from stk_bridge.core.attitude_equivalence import (
    COMBINE_TOLERANCE_RAD,
    sign_aligned_angle,
)
from stk_bridge.core.frames import FrameRegistry
from stk_bridge.core.sensing import SensingConfig, SensingPipeline

from test_equivalence_diagnostic import (   # noqa: I100 - shared fake
    SAT,
    FakeBackend,
    _attitude,
    _epochs,
    _report,
    _samples_as_values,
)

SEPARATE = ["Cartesian Position", "Cartesian Velocity",
            "Attitude Quaternions", "Attitude Segment Quaternions"]
COMBINED = ["Cartesian Position", "Cartesian Velocity",
            "Attitude Segment Quaternions"]


def _pipeline(backend, combine=True, written=lambda path, when: True,
              **config):
    pipeline = SensingPipeline(
        backend, FrameRegistry(),
        SensingConfig(satellites=config.pop("satellites", [SAT]),
                      combine_attitude_rate=combine, **config),
        _report())
    pipeline.set_attitude_write_probe(written)
    return pipeline


def _names_per_step(backend, steps):
    names = [q.provider_name for q in backend.queries]
    out, index = [], 0
    for count in steps:
        out.append(names[index:index + count])
        index += count
    assert index == len(names)
    return out


class TestOffAndConfiguration:
    def test_off_by_default(self):
        assert SensingConfig().combine_attitude_rate is False
        assert SensingConfig.from_config({}).combine_attitude_rate is False

    def test_off_reads_exactly_as_before(self):
        backend = FakeBackend()
        pipeline = _pipeline(backend, combine=False)
        for when in _epochs(5):
            pipeline.read_all(SAT, when)
        assert _names_per_step(backend, [4] * 5) == [SEPARATE] * 5
        assert pipeline.attitude_provider == "Attitude Quaternions"
        assert pipeline.sensing_mode == "separate"

    @pytest.mark.parametrize("relative, expected", [
        (("stk_bridge", "config", "bridge_params.yaml"), False),
        (("acav", "config", "bridge_params_acav.yaml"), True)])
    def test_shipped_configs(self, relative, expected):
        import os
        import yaml
        src = os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))))
        with open(os.path.join(src, *relative), encoding="utf-8") as handle:
            settings = yaml.safe_load(handle)
        assert settings["sensing"]["combine_attitude_rate"] is expected


class TestActivationGate:
    def test_never_activates_without_a_written_sample(self):
        backend = FakeBackend()
        pipeline = _pipeline(backend, written=lambda path, when: False)
        for when in _epochs(5):
            pipeline.read_all(SAT, when)
        assert _names_per_step(backend, [4] * 5) == [SEPARATE] * 5
        assert pipeline.sensing_mode == "separate"

    def test_activates_after_one_verified_step(self):
        backend = FakeBackend()
        epochs = _epochs(6)
        written_from = epochs[2].epoch_seconds        # first command lands
        pipeline = _pipeline(
            backend, written=lambda p, w: w.epoch_seconds >= written_from)
        for when in epochs:
            pipeline.read_all(SAT, when)
        assert _names_per_step(backend, [4, 4, 4, 3, 3, 3]) == \
            [SEPARATE] * 3 + [COMBINED] * 3
        assert pipeline.attitude_provider == "Attitude Segment Quaternions"
        assert pipeline.sensing_mode == "combined_attitude_rate"

    def test_combined_values_equal_separate_values(self):
        separate, combined = FakeBackend(), FakeBackend()
        plain = _pipeline(separate, combine=False)
        shared = _pipeline(combined)
        for when in _epochs(10):
            assert (_samples_as_values(shared.read_all(SAT, when))
                    == _samples_as_values(plain.read_all(SAT, when)))
        assert shared.sensing_mode == "combined_attitude_rate"

    def test_health_counts_both_channels_in_combined_mode(self):
        pipeline = _pipeline(FakeBackend())
        for when in _epochs(4):
            pipeline.read_all(SAT, when)
        health = pipeline.health()
        assert health["attitude"].samples == 4
        assert health["angular_rate"].samples == 4


class TestSignAndMismatch:
    def test_a_sign_flipped_segment_still_verifies(self):
        backend = FakeBackend(segment_sign=-1.0)
        pipeline = _pipeline(backend)
        epochs = _epochs(3)
        for when in epochs:
            samples = pipeline.read_all(SAT, when)
        assert pipeline.sensing_mode == "combined_attitude_rate"
        # Published from the segment provider, which carries -q.
        assert samples["attitude"].quaternion_xyzw == pytest.approx(
            tuple(-c for c in _attitude(epochs[-1].epoch_seconds)))

    def test_a_mismatch_keeps_the_separate_reads(self, caplog):
        class Rotated(FakeBackend):
            def query_data_provider(self, query):
                data = super().query_data_provider(query)
                if query.provider_name == "Attitude Segment Quaternions":
                    # Rotate by 1e-9 rad about z: above 1e-12 rad.
                    x, y, z, w = (data.columns[k][0]
                                  for k in ("q1", "q2", "q3", "q4"))
                    c, s = math.cos(5e-10), math.sin(5e-10)
                    data.columns["q1"][0] = c * x + s * y
                    data.columns["q2"][0] = c * y - s * x
                    data.columns["q3"][0] = c * z + s * w
                    data.columns["q4"][0] = c * w - s * z
                return data
        backend = Rotated()
        pipeline = _pipeline(backend)
        for when in _epochs(4):
            pipeline.read_all(SAT, when)
        assert _names_per_step(backend, [4] * 4) == [SEPARATE] * 4
        assert pipeline.sensing_mode == "separate"
        assert "combined attitude/rate NOT enabled" in caplog.text

    def test_the_angle_is_sign_aligned_and_resolves_1e_12(self):
        a = (0.0, 0.0, 0.0, 1.0)
        b = (0.0, 0.0, math.sin(1e-12 / 2), math.cos(1e-12 / 2))
        assert sign_aligned_angle(a, b) == pytest.approx(1e-12, rel=1e-6)
        assert sign_aligned_angle(a, tuple(-c for c in a)) == 0.0
        assert COMBINE_TOLERANCE_RAD == 1e-12


class TestSegmentNoData:
    def test_attitude_falls_back_and_rate_stays_missing(self):
        epochs = _epochs(4)
        backend = FakeBackend(segment_empty_at=[epochs[-1].epoch_seconds])
        combined = _pipeline(backend)
        reference = _pipeline(FakeBackend(
            segment_empty_at=[epochs[-1].epoch_seconds]), combine=False)
        for when in epochs:
            samples = combined.read_all(SAT, when)
            expected = reference.read_all(SAT, when)
        assert set(samples) == {"orbit", "attitude"}         # as today
        assert _samples_as_values(samples) == _samples_as_values(expected)
        last = [q.provider_name for q in backend.queries[-4:]]
        assert last == ["Cartesian Position", "Cartesian Velocity",
                        "Attitude Segment Quaternions",
                        "Attitude Quaternions"]
        assert combined.sensing_mode == "combined_attitude_rate"


class TestUnsupportedConfigurations:
    def test_a_forced_provider_stays_on_the_original_path(self):
        backend = FakeBackend()
        pipeline = _pipeline(
            backend, force_attitude_provider="ECF Attitude Quaternions")
        assert pipeline.sensing_mode == "separate"
        assert pipeline.attitude_provider == "ECF Attitude Quaternions"

    def test_more_than_one_satellite_stays_separate(self):
        pipeline = _pipeline(FakeBackend(),
                             satellites=[SAT, "*/Satellite/Other"])
        assert pipeline.sensing_mode == "separate"

    def test_a_backend_without_a_revision_stays_separate(self):
        backend = FakeBackend()
        backend.revision = None
        pipeline = _pipeline(backend)
        for when in _epochs(3):
            pipeline.read_all(SAT, when)
        assert pipeline.sensing_mode == "separate"


class TestInvalidation:
    def test_a_revision_change_requires_a_new_verification(self):
        backend = FakeBackend()
        pipeline = _pipeline(backend)
        epochs = _epochs(6)
        for when in epochs[:3]:
            pipeline.read_all(SAT, when)
        assert pipeline.sensing_mode == "combined_attitude_rate"
        backend.revision += 1                 # e.g. ClearData, lifecycle
        for when in epochs[3:]:
            pipeline.read_all(SAT, when)
        assert _names_per_step(backend, [4, 3, 3, 4, 3, 3]) == [
            SEPARATE, COMBINED, COMBINED, SEPARATE, COMBINED, COMBINED]


def test_the_combined_spec_is_the_rate_spec():
    """The shared result is the one the rate read already used."""
    pipeline = _pipeline(FakeBackend())
    assert pipeline._rate_spec is dp.ATTITUDE_SEGMENT


def test_statistics_count_a_revision_reset_and_its_re_verification():
    backend = FakeBackend()
    pipeline = _pipeline(backend)
    epochs = _epochs(6)
    for when in epochs[:3]:
        pipeline.read_all(SAT, when)
    backend.revision += 1
    for when in epochs[3:]:
        pipeline.read_all(SAT, when)
    stats = pipeline.combine_statistics()
    assert stats["verifications"] == 2
    assert stats["re_verifications"] == 1
    assert stats["revision_resets"] == 1
    assert stats["steps_separate"] == 2 and stats["steps_combined"] == 4


def test_statistics_count_segment_no_data_fallbacks():
    epochs = _epochs(4)
    pipeline = _pipeline(FakeBackend(
        segment_empty_at=[epochs[-1].epoch_seconds]))
    for when in epochs:
        pipeline.read_all(SAT, when)
    assert pipeline.combine_statistics()["segment_no_data_fallbacks"] == 1
