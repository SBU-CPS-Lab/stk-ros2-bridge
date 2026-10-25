"""The opt-in sensing equivalence diagnostic, and the orbit-block reader.

The diagnostic must OBSERVE only: with it off, the queries and the
published samples are exactly what they were; with it on, the published
samples are still the old path's. What it adds is a per-step comparison
of the 'Attitude Segment Quaternions' quaternion against 'Attitude
Quaternions', and an interval orbit block against the instantaneous
orbit read, written next to the run logs.

The fake backend below answers like the COM backend does: UTCG text in
the Time column (DateFormat is UTCG), instantaneous and interval queries,
and NO DATA as an empty result.
"""

import json
import math
import os

import pytest
import yaml

from stk_bridge.backends import connect_protocol as proto
from stk_bridge.backends.base import (
    DataProviderData,
    ObjectInfo,
    StkTime,
)
from stk_bridge.core import conversions as conv
from stk_bridge.core.equivalence_diagnostic import (
    TOLERANCES,
    EquivalenceDiagnostic,
)
from stk_bridge.core.frames import FrameRegistry
from stk_bridge.core.orbit_block import (
    OrbitBlockError,
    block_epochs,
    epoch_ms,
    read_orbit_block,
)
from stk_bridge.core.sensing import SensingConfig, SensingPipeline
from stk_bridge.core.validation import ValidationReport

SAT = "*/Satellite/SSS-1"
EPOCH = 1717237800.0                      # 1 Jun 2024 10:30:00 UTC
STOP = EPOCH + 720.0
JSONL_KEYS = {"step", "t", "angle_rad", "dr_m", "dv_mps",
              "time_mismatch_ms", "flags"}


def _ms_time(seconds):
    """STK evaluates the UTCG text it was given, i.e. the ms-rounded time."""
    return round(seconds * 1000.0) / 1000.0


def _orbit(seconds):
    t = _ms_time(seconds) - EPOCH
    w = 0.0011
    return ((7.0e6 * math.cos(w * t), 7.0e6 * math.sin(w * t), 1234.5),
            (-7.7e3 * math.sin(w * t), 7.7e3 * math.cos(w * t), 0.25))


def _attitude(seconds):
    t = _ms_time(seconds) - EPOCH
    half = 0.0005 * t
    return (0.1, 0.2, math.sin(half) * 0.5,
            math.sqrt(1 - 0.05 - (math.sin(half) * 0.5) ** 2))


class FakeBackend:
    """COM-shaped data providers, with knobs for the failure cases."""

    #: Structural revision, as ComBackend.revision; tests bump it.
    revision = 0

    def __init__(self, segment_sign=1.0, segment_empty_at=(),
                 interval_dx_m=0.0, interval_time_shift_ms=0,
                 interval_fails=False):
        self.queries = []
        self.segment_sign = segment_sign
        self.segment_empty_at = {epoch_ms(StkTime(t))
                                 for t in segment_empty_at}
        self.interval_dx_m = interval_dx_m
        self.interval_time_shift_ms = interval_time_shift_ms
        self.interval_fails = interval_fails

    def list_objects(self):
        return [ObjectInfo(SAT, "Satellite", "HPOP")]

    def query_data_provider(self, query):
        self.queries.append(query)
        if query.is_instantaneous:
            times = [query.start.epoch_seconds]
        else:
            if self.interval_fails:
                from stk_bridge.exceptions import QueryError
                raise QueryError(query.provider_name, query.object_path,
                                 "interval execution failed")
            n = int(round((query.stop.epoch_seconds
                           - query.start.epoch_seconds) / query.step_s)) + 1
            times = [query.start.epoch_seconds + i * query.step_s
                     for i in range(n)]
        name = query.provider_name
        if (name == "Attitude Segment Quaternions"
                and epoch_ms(StkTime(times[0])) in self.segment_empty_at):
            return DataProviderData(object_path=query.object_path,
                                    provider_name=name, columns={},
                                    row_count=0)
        interval = not query.is_instantaneous
        columns = {"Time": []}
        for t in times:
            shift = self.interval_time_shift_ms / 1000.0 if interval else 0.0
            columns["Time"].append(proto.format_utcg(StkTime(t + shift)))
            position, velocity = _orbit(t)
            q = _attitude(t)
            if name == "Cartesian Position":
                dx = self.interval_dx_m if interval else 0.0
                row = {"x": position[0] + dx, "y": position[1],
                       "z": position[2]}
            elif name == "Cartesian Velocity":
                row = dict(zip(("x", "y", "z"), velocity))
            elif name == "Attitude Quaternions":
                row = dict(zip(("q1", "q2", "q3", "q4"), q))
            elif name == "Attitude Segment Quaternions":
                s = self.segment_sign
                row = {"q1": s * q[0], "q2": s * q[1], "q3": s * q[2],
                       "q4": s * q[3], "wx": 0.001, "wy": 0.0, "wz": 0.0}
            else:
                raise AssertionError(f"unexpected provider {name}")
            for key, value in row.items():
                columns.setdefault(key, []).append(value)
        return DataProviderData(object_path=query.object_path,
                                provider_name=name, columns=columns,
                                row_count=len(times))


def _report():
    return ValidationReport(
        attitude_provider="Attitude Quaternions",
        attitude_command_inertial=True,
        angular_rate_provider="Attitude Segment Quaternions",
        angle_rate_scale=conv.AngleRateScale.radians_per_second())


def _epochs(n, start_index=0):
    """Lockstep epochs exactly as TimeSync makes them: sim += step."""
    sim, out = 0.0, []
    for index in range(start_index + n):
        sim += 0.1
        if index >= start_index:
            out.append(StkTime(EPOCH + sim))
    return out


def _pipeline(backend, diagnostic=None, **config):
    pipeline = SensingPipeline(backend, FrameRegistry(),
                               SensingConfig(satellites=[SAT], **config),
                               _report())
    if diagnostic is not None:
        pipeline.attach_diagnostic(diagnostic)
    return pipeline


def _diagnostic(tmp_path, **kwargs):
    return EquivalenceDiagnostic(str(tmp_path), epoch=StkTime(EPOCH),
                                 stop_time=StkTime(STOP), step_s=0.1,
                                 **kwargs)


def _lines(tmp_path):
    with open(tmp_path / "equivalence_diagnostic.jsonl",
              encoding="utf-8") as handle:
        return [json.loads(line) for line in handle]


def _summary(tmp_path):
    with open(tmp_path / "equivalence_diagnostic_summary.json",
              encoding="utf-8") as handle:
        return json.load(handle)


def _samples_as_values(samples):
    return {key: (getattr(value, "position_m", None),
                  getattr(value, "velocity_mps", None),
                  getattr(value, "quaternion_xyzw", None),
                  getattr(value, "angular_velocity_radps", None),
                  value.time.epoch_seconds)
            for key, value in samples.items()}


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

class TestConfiguration:
    def test_off_by_default(self):
        assert SensingConfig().equivalence_diagnostic is False
        assert SensingConfig.from_config({}).equivalence_diagnostic is False

    def test_read_from_the_sensing_section(self):
        config = SensingConfig.from_config({"sensing": {
            "equivalence_diagnostic": True,
            "equivalence_diagnostic_dir": "C:/run.logs"}})
        assert config.equivalence_diagnostic is True
        assert config.equivalence_diagnostic_dir == "C:/run.logs"

    @pytest.mark.parametrize("relative", [
        ("stk_bridge", "config", "bridge_params.yaml"),
        ("acav", "config", "bridge_params_acav.yaml")])
    def test_both_shipped_configs_set_it_false(self, relative):
        src = os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))))
        with open(os.path.join(src, *relative), encoding="utf-8") as handle:
            settings = yaml.safe_load(handle)
        assert settings["sensing"]["equivalence_diagnostic"] is False


# ---------------------------------------------------------------------------
# Flag off: nothing changes. Flag on: published values still old-path.
# ---------------------------------------------------------------------------

class TestObservesOnly:
    def test_flag_off_makes_exactly_the_four_old_queries_per_step(self):
        backend = FakeBackend()
        pipeline = _pipeline(backend)
        for when in _epochs(5):
            pipeline.read_all(SAT, when)
        names = [q.provider_name for q in backend.queries]
        assert names == ["Cartesian Position", "Cartesian Velocity",
                         "Attitude Quaternions",
                         "Attitude Segment Quaternions"] * 5
        assert all(q.is_instantaneous for q in backend.queries)

    def test_flag_on_publishes_the_same_values_as_flag_off(self, tmp_path):
        off, on = FakeBackend(), FakeBackend()
        plain = _pipeline(off)
        observed = _pipeline(on, _diagnostic(tmp_path))
        for when in _epochs(150):
            assert (_samples_as_values(observed.read_all(SAT, when))
                    == _samples_as_values(plain.read_all(SAT, when)))

    def test_flag_on_adds_only_two_interval_queries_per_block(self, tmp_path):
        backend = FakeBackend()
        pipeline = _pipeline(backend, _diagnostic(tmp_path))
        for when in _epochs(150):
            pipeline.read_all(SAT, when)
        interval = [q for q in backend.queries if not q.is_instantaneous]
        assert [q.provider_name for q in interval] == [
            "Cartesian Position", "Cartesian Velocity"] * 2
        instantaneous = [q for q in backend.queries if q.is_instantaneous]
        assert len(instantaneous) == 4 * 150      # the old path, untouched

    def test_a_failing_diagnostic_never_breaks_sensing(self, tmp_path):
        backend = FakeBackend(interval_fails=True)
        pipeline = _pipeline(backend, _diagnostic(tmp_path))
        samples = pipeline.read_all(SAT, _epochs(1)[0])
        assert set(samples) == {"orbit", "attitude", "angular_rate"}
        assert any(f.startswith("block_query_failed")
                   for f in _lines(tmp_path)[0]["flags"])

    def test_a_retry_of_some_channels_is_not_observed(self, tmp_path):
        backend = FakeBackend()
        pipeline = _pipeline(backend, _diagnostic(tmp_path))
        when = _epochs(1)[0]
        pipeline.read_all(SAT, when)
        pipeline.read_all(SAT, when, channels=["angular_rate"])
        assert len(_lines(tmp_path)) == 1


# ---------------------------------------------------------------------------
# attitude: segment quaternion versus Attitude Quaternions
# ---------------------------------------------------------------------------

class TestAttitudeComparison:
    def test_identical_quaternions_compare_as_exact_zero(self, tmp_path):
        pipeline = _pipeline(FakeBackend(), _diagnostic(tmp_path))
        for when in _epochs(3):
            pipeline.read_all(SAT, when)
        assert [line["angle_rad"] for line in _lines(tmp_path)] == [0.0] * 3

    def test_the_comparison_is_sign_aligned(self, tmp_path):
        pipeline = _pipeline(FakeBackend(segment_sign=-1.0),
                             _diagnostic(tmp_path))
        pipeline.read_all(SAT, _epochs(1)[0])
        line = _lines(tmp_path)[0]
        assert line["angle_rad"] == pytest.approx(0.0, abs=1e-15)
        raw = line["raw"]
        assert raw["segment_quaternions"] == [
            -c for c in raw["attitude_quaternions"]]

    @pytest.mark.parametrize("theta", [1e-6, 1e-8, 1e-10, 1e-12])
    def test_the_angle_resolves_the_agreed_tolerance(self, theta):
        from stk_bridge.core.equivalence_diagnostic import sign_aligned_angle
        a = (0.0, 0.0, 0.0, 1.0)
        b = (0.0, 0.0, math.sin(theta / 2), math.cos(theta / 2))
        assert sign_aligned_angle(a, b) == pytest.approx(theta, rel=1e-6)
        assert sign_aligned_angle(a, tuple(-c for c in b)) == \
            pytest.approx(theta, rel=1e-6)

    def test_no_extra_attitude_query_is_made(self, tmp_path):
        backend = FakeBackend()
        pipeline = _pipeline(backend, _diagnostic(tmp_path))
        pipeline.read_all(SAT, _epochs(1)[0])
        segment = [q for q in backend.queries
                   if q.provider_name == "Attitude Segment Quaternions"]
        assert len(segment) == 1

    def test_raw_components_are_logged_for_the_first_steps_only(
            self, tmp_path):
        pipeline = _pipeline(FakeBackend(), _diagnostic(tmp_path,
                                                        raw_steps=20))
        for when in _epochs(25):
            pipeline.read_all(SAT, when)
        lines = _lines(tmp_path)
        assert all("raw" in line for line in lines[:20])
        assert all("raw" not in line for line in lines[20:])
        assert lines[0]["step"] == 1 and lines[0]["t"] == pytest.approx(0.1)

    def test_a_forced_provider_is_flagged_not_compared(self, tmp_path):
        pipeline = _pipeline(
            FakeBackend(), _diagnostic(tmp_path),
            force_attitude_provider="Attitude Segment Quaternions")
        pipeline.read_all(SAT, _epochs(1)[0])
        line = _lines(tmp_path)[0]
        assert line["angle_rad"] is None
        assert any(f.startswith("attitude_provider_not_inertial")
                   for f in line["flags"])


# ---------------------------------------------------------------------------
# orbit: interval block versus instantaneous reads
# ---------------------------------------------------------------------------

class TestBlockGrid:
    def test_block_epochs_follow_the_step_and_stop_before_stop_time(self):
        start = StkTime(EPOCH + 719.5)
        epochs = block_epochs(start, 0.1, 100, StkTime(STOP))
        assert [epoch_ms(e) - epoch_ms(StkTime(EPOCH)) for e in epochs] == [
            719500, 719600, 719700, 719800, 719900]

    def test_no_block_at_or_after_the_stop_time(self):
        assert block_epochs(StkTime(STOP), 0.1, 100, StkTime(STOP)) == []

    def test_an_accumulated_lockstep_grid_lands_on_block_epochs(self):
        """TimeSync adds 0.1 to sim_time 7200 times; a block computes
        start + k*0.1. Both must name the same millisecond every step."""
        epochs = _epochs(7199)                     # t = 0.1 .. 719.9
        for first in range(0, len(epochs), 100):
            block = block_epochs(epochs[first], 0.1, 100, StkTime(STOP))
            wanted = [epoch_ms(e) for e in epochs[first:first + 100]]
            assert [epoch_ms(e) for e in block] == wanted

    def test_a_clean_block_validates(self):
        epochs = block_epochs(_epochs(1)[0], 0.1, 100, StkTime(STOP))
        block = read_orbit_block(FakeBackend(), SAT, "ICRF", epochs, 0.1)
        block.raise_if_invalid()
        assert len(block.positions) == len(block.velocities) == 100

    def test_a_shifted_time_column_is_reported(self):
        epochs = block_epochs(_epochs(1)[0], 0.1, 10, StkTime(STOP))
        block = read_orbit_block(FakeBackend(interval_time_shift_ms=1),
                                 SAT, "ICRF", epochs, 0.1)
        with pytest.raises(OrbitBlockError, match="time grid"):
            block.raise_if_invalid()

    def test_a_wrong_row_count_is_reported(self):
        class Short(FakeBackend):
            def query_data_provider(self, query):
                data = super().query_data_provider(query)
                if not query.is_instantaneous:
                    data.columns = {k: v[:-1] for k, v in data.columns.items()}
                    data.row_count -= 1
                return data
        epochs = block_epochs(_epochs(1)[0], 0.1, 10, StkTime(STOP))
        block = read_orbit_block(Short(), SAT, "ICRF", epochs, 0.1)
        with pytest.raises(OrbitBlockError, match="rows"):
            block.raise_if_invalid()

    def test_a_non_finite_value_is_reported(self):
        class NotFinite(FakeBackend):
            def query_data_provider(self, query):
                data = super().query_data_provider(query)
                if (not query.is_instantaneous
                        and query.provider_name == "Cartesian Velocity"):
                    data.columns["y"][3] = float("nan")
                return data
        epochs = block_epochs(_epochs(1)[0], 0.1, 10, StkTime(STOP))
        block = read_orbit_block(NotFinite(), SAT, "ICRF", epochs, 0.1)
        with pytest.raises(OrbitBlockError, match="finite"):
            block.raise_if_invalid()


class TestOrbitComparison:
    def test_identical_rows_compare_as_exact_zero(self, tmp_path):
        pipeline = _pipeline(FakeBackend(), _diagnostic(tmp_path))
        for when in _epochs(120):
            pipeline.read_all(SAT, when)
        lines = _lines(tmp_path)
        assert {line["dr_m"] for line in lines} == {0.0}
        assert {line["dv_mps"] for line in lines} == {0.0}
        assert {line["time_mismatch_ms"] for line in lines} == {0}

    def test_a_position_difference_is_measured(self, tmp_path):
        pipeline = _pipeline(FakeBackend(interval_dx_m=1e-3),
                             _diagnostic(tmp_path))
        pipeline.read_all(SAT, _epochs(1)[0])
        assert _lines(tmp_path)[0]["dr_m"] == pytest.approx(1e-3)

    def test_a_time_column_shift_is_measured(self, tmp_path):
        pipeline = _pipeline(FakeBackend(interval_time_shift_ms=1),
                             _diagnostic(tmp_path))
        pipeline.read_all(SAT, _epochs(1)[0])
        line = _lines(tmp_path)[0]
        assert line["time_mismatch_ms"] == 1
        assert any(f.startswith("block_invalid") for f in line["flags"])

    def test_the_last_block_stops_before_the_stop_time(self, tmp_path):
        backend = FakeBackend()
        pipeline = _pipeline(backend, _diagnostic(tmp_path))
        for when in _epochs(7200):                  # t = 0.1 .. 720.0
            pipeline.read_all(SAT, when)
        interval = [q for q in backend.queries if not q.is_instantaneous]
        assert max(epoch_ms(q.stop) for q in interval) < epoch_ms(
            StkTime(STOP))
        assert len(interval) == 2 * 72               # 72 blocks, 2 queries


# ---------------------------------------------------------------------------
# Terminal epoch and NO DATA
# ---------------------------------------------------------------------------

class TestTerminalEpoch:
    def test_the_terminal_epoch_is_not_batched_and_logs_raw(self, tmp_path):
        pipeline = _pipeline(FakeBackend(), _diagnostic(tmp_path,
                                                        raw_steps=0))
        terminal = _epochs(1, start_index=7199)[0]   # t = 720.0
        assert epoch_ms(terminal) == epoch_ms(StkTime(STOP))
        pipeline.read_all(SAT, terminal)
        line = _lines(tmp_path)[0]
        assert "terminal_epoch" in line["flags"]
        assert line["dr_m"] is None and "raw" in line

    def test_segment_no_data_is_flagged_and_attitude_still_published(
            self, tmp_path):
        terminal = _epochs(1, start_index=7199)[0]
        backend = FakeBackend(segment_empty_at=[terminal.epoch_seconds])
        pipeline = _pipeline(backend, _diagnostic(tmp_path))
        samples = pipeline.read_all(SAT, terminal)
        assert set(samples) == {"orbit", "attitude"}      # as today
        line = _lines(tmp_path)[0]
        assert "no_segment_result" in line["flags"]
        assert line["angle_rad"] is None
        assert line["raw"]["segment_quaternions"] is None


# ---------------------------------------------------------------------------
# Output files
# ---------------------------------------------------------------------------

class TestOutput:
    def test_every_line_has_the_schema(self, tmp_path):
        pipeline = _pipeline(FakeBackend(), _diagnostic(tmp_path))
        for when in _epochs(30):
            pipeline.read_all(SAT, when)
        for line in _lines(tmp_path):
            assert JSONL_KEYS <= set(line)
            assert set(line) - JSONL_KEYS <= {"raw"}
            assert isinstance(line["flags"], list)

    def test_summary_counts_zeros_and_first_exceedance(self, tmp_path):
        backend = FakeBackend()
        diagnostic = _diagnostic(tmp_path)
        pipeline = _pipeline(backend, diagnostic)
        epochs = _epochs(150)
        for when in epochs[:100]:
            pipeline.read_all(SAT, when)
        backend.interval_dx_m = 2e-6                 # second block only
        for when in epochs[100:]:
            pipeline.read_all(SAT, when)
        diagnostic.close()
        summary = _summary(tmp_path)
        assert summary["equivalence_diagnostic"] is True
        assert summary["tolerances"] == TOLERANCES
        dr = summary["metrics"]["dr_m"]
        assert dr["count"] == 150 and dr["exact_zero"] == 100
        # 7e6 m + 2e-6 m in double precision: resolved to ~1e-9 m.
        assert dr["max"] == pytest.approx(2e-6, rel=1e-3)
        assert dr["first_exceeding"]["step"] == 101
        assert summary["metrics"]["angle_rad"]["first_exceeding"] is None

    def test_tolerances_are_the_agreed_strict_values(self):
        assert TOLERANCES == {"angle_rad": 1e-12, "dr_m": 1e-6,
                              "dv_mps": 1e-9}

    def test_the_summary_is_written_without_close(self, tmp_path):
        """The runner may hard-terminate the bridge on Windows, so the
        summary is refreshed every block, not only on close."""
        pipeline = _pipeline(FakeBackend(), _diagnostic(tmp_path))
        for when in _epochs(100):
            pipeline.read_all(SAT, when)
        assert _summary(tmp_path)["metrics"]["dr_m"]["count"] == 100
