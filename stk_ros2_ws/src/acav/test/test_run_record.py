"""Tests for the C5/C6 record schema.

The properties asserted here are the ones that decide whether a number
may be published: that a mock-backend run cannot be mistaken for a real
one, that a hand-edited record is rejected rather than averaged in, and
that NaN survives a round trip as NaN rather than becoming zero.
"""

import json
import math
import os

import pytest

from acav.metrics import CaptureOutcome, RunResult, TargetResult
from acav.run_record import (
    RECORD_VERSION,
    Provenance,
    RecordError,
    RunRecord,
    read_record,
    scenario_digest,
    unreportable,
    write_record,
)


def _result(run_id="run", seed=7):
    return RunResult(
        run_id=run_id,
        scheduler="dynamics_aware",
        targets=[
            TargetResult(target_id="T1", attempted=True,
                         outcome=CaptureOutcome.CAPTURED,
                         image_time_s=120.0, error_at_image_deg=0.21,
                         max_error_in_hold_deg=0.33, settle_time_s=6.5,
                         slew_magnitude_deg=18.6, slew_duration_s=9.1),
            # Unattempted: its errors are legitimately NaN, which is the
            # case that makes json's bare NaN token a problem.
            TargetResult(target_id="T5", attempted=False,
                         outcome=CaptureOutcome.NOT_ATTEMPTED,
                         image_time_s=300.0),
        ],
        peak_momentum_fraction=(0.41, 0.22, 0.08),
        wheel_saturation_events=2,
        entered_safe_mode=False,
        wheel_energy_J=0.0173,
        dropped_attitude_samples=0,
        mean_step_latency_ms=9.8,
        seed=seed)


def _record(backend="com", **kwargs):
    return RunRecord(result=_result(**kwargs),
                     provenance=Provenance(backend_type=backend,
                                           execution_mode="lockstep",
                                           stk_version="12.6.0"),
                     experiment="E2")


class TestRoundTrip:
    def test_everything_survives_a_round_trip(self, tmp_path):
        original = _record()
        path = str(tmp_path / "run.json")
        write_record(path, original)
        restored = read_record(path)

        assert restored.result.run_id == original.result.run_id
        assert restored.result.seed == original.result.seed
        assert restored.result.peak_momentum_fraction == \
            original.result.peak_momentum_fraction
        assert restored.result.wheel_saturation_events == 2
        assert restored.experiment == "E2"
        assert restored.provenance.backend_type == "com"
        assert restored.signature == original.signature

    def test_nan_round_trips_as_nan_not_zero(self, tmp_path):
        """An unattempted target's error is NaN. Reading it back as 0.0
        would put a perfect pointing score on a target nobody pointed
        at, which flatters every metric that averages over targets."""
        path = str(tmp_path / "run.json")
        write_record(path, _record())
        restored = read_record(path)

        unattempted = next(t for t in restored.result.targets
                           if t.target_id == "T5")
        assert math.isnan(unattempted.error_at_image_deg)
        assert math.isnan(unattempted.max_error_in_hold_deg)

    def test_the_file_is_strict_json_with_no_nan_token(self, tmp_path):
        """``json.dump`` writes NaN as a bare ``NaN``, which is not JSON
        and which MATLAB's jsondecode rejects -- and MATLAB is a reader
        this project actually uses."""
        path = str(tmp_path / "run.json")
        write_record(path, _record())
        text = (tmp_path / "run.json").read_text()

        assert "NaN" not in text
        assert "Infinity" not in text
        json.loads(text)          # strict by default: would raise on NaN

    def test_no_partial_file_is_left_behind(self, tmp_path):
        write_record(str(tmp_path / "run.json"), _record())
        assert sorted(os.listdir(tmp_path)) == ["run.json"]


class TestReportability:
    def test_the_mock_backend_is_not_reportable(self):
        """C1 section 8: no number the mock produced may appear in the
        paper. The mock is an in-process Python object with no STK in
        it, so its numbers are not measurements of anything."""
        assert not _record(backend="mock").reportable

    @pytest.mark.parametrize("backend", ["com", "engine", "connect"])
    def test_real_backends_are_reportable(self, backend):
        assert _record(backend=backend).reportable

    def test_an_unknown_backend_is_not_reportable(self):
        """A run whose backend could not be established is not a run we
        can vouch for, and defaulting to reportable would make a missing
        status message into a publishable number."""
        assert not _record(backend="unknown").reportable

    def test_reportability_is_derived_not_read_from_the_file(self, tmp_path):
        """The stored flag is a convenience for a human reading the
        file. If it could be believed, a record copied from a real run
        onto a mock one -- or simply edited -- would launder the mock's
        numbers into the paper."""
        path = tmp_path / "run.json"
        write_record(str(path), _record(backend="mock"))

        payload = json.loads(path.read_text())
        payload["reportable"] = True            # a lie
        path.write_text(json.dumps(payload))

        assert not read_record(str(path)).reportable

    def test_unreportable_names_the_offending_runs(self):
        records = [_record(backend="com", run_id="a"),
                   _record(backend="mock", run_id="b"),
                   _record(backend="mock", run_id="c")]
        offenders = [r.result.run_id for r in unreportable(records)]
        assert offenders == ["b", "c"]


class TestTamperDetection:
    def test_an_edited_result_is_rejected(self, tmp_path):
        """The signature is recomputed on read. A record whose stored
        signature disagrees with its own result has been edited or
        truncated, and a campaign that reported E3 from it would be
        describing a file rather than a run."""
        path = tmp_path / "run.json"
        write_record(str(path), _record())

        payload = json.loads(path.read_text())
        payload["result"]["targets"][0]["error_at_image_deg"] = 0.99
        path.write_text(json.dumps(payload))

        with pytest.raises(RecordError, match="signature"):
            read_record(str(path))

    def test_a_record_from_another_schema_version_is_rejected(self, tmp_path):
        path = tmp_path / "run.json"
        write_record(str(path), _record())

        payload = json.loads(path.read_text())
        payload["record_version"] = RECORD_VERSION + 1
        path.write_text(json.dumps(payload))

        with pytest.raises(RecordError, match="record version"):
            read_record(str(path))


class TestScenarioDigest:
    def test_the_digest_changes_when_the_scenario_does(self, tmp_path):
        first = tmp_path / "a.yaml"
        second = tmp_path / "b.yaml"
        first.write_text("step_s: 0.1\n")
        second.write_text("step_s: 0.2\n")
        assert scenario_digest(str(first)) != scenario_digest(str(second))

    def test_identical_content_digests_identically(self, tmp_path):
        first = tmp_path / "a.yaml"
        second = tmp_path / "b.yaml"
        first.write_text("step_s: 0.1\n")
        second.write_text("step_s: 0.1\n")
        assert scenario_digest(str(first)) == scenario_digest(str(second))
