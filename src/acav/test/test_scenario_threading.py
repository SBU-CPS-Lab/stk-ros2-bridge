"""Generated scenarios threaded through runs and reports.

- Known scenario files: the reference, the generated files committed
  under config/scenarios/generated (each equal to what the generator
  writes for its seed), and any file named with --scenario. A record's
  scenario is found among them by its digest (legacy digests mapped).
- A scenario's origin (seed, generator version) is read from its header
  and recorded in the record's provenance.
- The runner refuses a scenario whose shared sections (everything the
  twin reads from the reference) differ from the reference's.
- report checks every record against its own scenario and refuses
  records from more than one scenario.
"""

import json
import os

import pytest

from acav import scenarios as sc
from acav import scenario_generator as sg
from acav.deploy_run_cli import ConnectResult, Deps, execute_run
from acav.metrics import CaptureOutcome, RunResult, TargetResult
from acav.run_record import (Provenance, RunRecord, read_record,
                             scenario_digest, write_record)
from acav.spec import AcavSpec

from test_deploy_run_cli import (  # noqa: I100, F401 - shared helpers
    FakeProcess, _config, offline_graph)

GENERATED = sorted(os.listdir(sc.GENERATED_DIR))


def _normalised(path):
    return open(path, "rb").read().replace(b"\r\n", b"\n")


class TestKnownScenarios:
    def test_the_committed_seeds(self):
        assert sorted(GENERATED) == sorted(f"acav_seed{k}.yaml"
                                           for k in range(1, 11))

    @pytest.mark.parametrize("seed", range(1, 11))
    def test_each_equals_the_generator(self, tmp_path, seed):
        fresh = sg.write_scenario(seed, str(tmp_path))
        committed = os.path.join(sc.GENERATED_DIR, f"acav_seed{seed}.yaml")
        assert _normalised(committed) == _normalised(fresh)

    def test_known_by_digest(self):
        known = sc.known_scenarios()
        assert len(known) == 11
        reference = known[scenario_digest(AcavSpec.locate())]
        assert reference.path == AcavSpec.locate()
        seed3 = os.path.join(sc.GENERATED_DIR, "acav_seed3.yaml")
        assert known[scenario_digest(seed3)].target_ids[-1] == "T22"

    def test_an_extra_file_is_known(self, tmp_path):
        extra = sg.write_scenario(42, str(tmp_path))
        assert scenario_digest(extra) in sc.known_scenarios([extra])
        assert scenario_digest(extra) not in sc.known_scenarios()


class TestOrigin:
    def test_a_generated_file(self):
        path = os.path.join(sc.GENERATED_DIR, "acav_seed7.yaml")
        assert sc.scenario_origin(path) == (7, sg.GENERATOR_VERSION)

    def test_the_reference(self):
        assert sc.scenario_origin(AcavSpec.locate()) == (None, "")

    def test_the_provenance_fields(self):
        path = os.path.join(sc.GENERATED_DIR, "acav_seed7.yaml")
        assert sc.scenario_provenance(path) == {
            "scenario_sha256": scenario_digest(path), "scenario_seed": 7,
            "scenario_generator_version": sg.GENERATOR_VERSION}

    def test_round_trip_and_old_records(self, tmp_path):
        record = RunRecord(result=RunResult(run_id="r", scheduler="x"),
                           provenance=Provenance(
                               scenario_seed=7,
                               scenario_generator_version="1"))
        write_record(str(tmp_path / "r.json"), record)
        back = read_record(str(tmp_path / "r.json")).provenance
        assert (back.scenario_seed, back.scenario_generator_version) == \
            (7, "1")
        old = record.to_dict()["provenance"]
        old.pop("scenario_seed")
        old.pop("scenario_generator_version")
        assert Provenance.from_dict(old).scenario_seed is None


class TestTwinParameters:
    def _altered(self, tmp_path, old, new):
        text = open(os.path.join(sc.GENERATED_DIR, "acav_seed1.yaml"),
                    encoding="utf-8").read()
        assert text.count(old) == 1
        path = tmp_path / "altered.yaml"
        path.write_text(text.replace(old, new), encoding="utf-8")
        return str(path)

    def test_a_generated_scenario_matches(self):
        for name in GENERATED:
            path = os.path.join(sc.GENERATED_DIR, name)
            assert sc.twin_parameter_differences(path) == []

    def test_a_changed_vehicle_is_named(self, tmp_path):
        path = self._altered(tmp_path, "momentum_capacity_Nms: 0.020",
                             "momentum_capacity_Nms: 0.030")
        assert sc.twin_parameter_differences(path) == ["actuators"]

    def test_a_changed_step_is_named(self, tmp_path):
        path = self._altered(tmp_path, "step_s: 0.1\n", "step_s: 0.2\n")
        assert "scenario" in sc.twin_parameter_differences(path)


class TestRunner:
    def _run(self, tmp_path, scenario):
        launched = []
        config = _config(tmp_path, scenario=scenario)
        deps = Deps(popen=lambda argv, **k: launched.append(argv)
                    or FakeProcess(),
                    wait_for_bridge_connected=lambda *a: ConnectResult(
                        True, "com"),
                    wait_for_twin_ack=lambda *a: True, prompt=lambda m: None,
                    disable_quickedit=lambda: ("disabled", None))
        code = execute_run(config, deps)
        summary = json.load(open(os.path.join(config.resolved_log_dir(),
                                              "summary.json")))
        return code, summary, launched

    def test_a_scenario_the_twin_would_not_match_is_refused(self,
                                                            tmp_path):
        text = open(os.path.join(sc.GENERATED_DIR, "acav_seed1.yaml"),
                    encoding="utf-8").read()
        altered = tmp_path / "altered.yaml"
        altered.write_text(text.replace("momentum_capacity_Nms: 0.020",
                                        "momentum_capacity_Nms: 0.030"),
                           encoding="utf-8")
        code, summary, launched = self._run(tmp_path, str(altered))
        assert code == 1 and launched == []
        step = [s for s in summary["steps"] if s["step"] == "scenario"]
        assert step and not step[0]["ok"] and "actuators" in step[0]["detail"]

    def test_a_generated_scenario_runs_and_is_recorded(self, tmp_path):
        path = os.path.join(sc.GENERATED_DIR, "acav_seed1.yaml")
        code, summary, launched = self._run(tmp_path, path)
        assert code == 0 and launched
        assert summary["scenario"] == {
            "path": path, "sha256": scenario_digest(path),
            "seed": 1, "generator_version": sg.GENERATOR_VERSION}

    def test_no_scenario_keeps_the_summary_unchanged(self, tmp_path):
        code, summary, _ = self._run(tmp_path, "")
        assert code == 0 and "scenario" not in summary


def _record(run_id, path):
    targets = sc.known_scenarios([path])[scenario_digest(path)].target_ids
    return RunRecord(
        result=RunResult(run_id=run_id, scheduler="dynamics_aware",
                         targets=[TargetResult(
                             target_id=t, attempted=False,
                             outcome=CaptureOutcome.NOT_ATTEMPTED,
                             image_time_s=0.0) for t in targets]),
        provenance=Provenance(backend_type="com",
                              execution_mode="lockstep",
                              scenario_sha256=scenario_digest(path)),
        experiment="E6")


class TestReport:
    def _report(self, tmp_path, records, caplog):
        from acav.campaign_cli import main

        for record in records:
            write_record(str(tmp_path / f"{record.result.run_id}.json"),
                         record)
        code = main(["report", "--experiment", "E6", "--repeats",
                     str(len(records)), "--no-figures", "--output-dir",
                     str(tmp_path)])
        return code, caplog.text

    def test_records_from_two_scenarios_are_refused(self, tmp_path, caplog):
        seed3 = os.path.join(sc.GENERATED_DIR, "acav_seed3.yaml")
        code, text = self._report(tmp_path, [
            _record("a", seed3), _record("b", AcavSpec.locate())], caplog)
        assert code == 1 and "scenarios" in text

    def test_records_of_one_generated_scenario_are_checked_against_it(
            self, tmp_path, caplog):
        seed3 = os.path.join(sc.GENERATED_DIR, "acav_seed3.yaml")
        code, text = self._report(tmp_path, [_record("a", seed3)], caplog)
        assert "do not cover" not in text and "scenarios" not in text

    def test_a_record_from_an_unknown_scenario_is_refused(self, tmp_path,
                                                          caplog):
        unknown = sg.write_scenario(99, str(tmp_path / "elsewhere"))
        code, text = self._report(tmp_path, [_record("a", unknown)], caplog)
        assert code == 1 and "not among the known scenario" in text
