"""A record must cover exactly its scenario's targets to be used.

A free-run record at rtf 1.0 was once written without T4 (the scheduler
dropped unscored targets). Such a record still reads, and its signature is self-consistent, but every count in it is
short: report would score it 3/4 instead of 3/5. The expected target
set is the scenario's own -- the scenario the record names by
scenario_sha256 -- never a hard-coded list.
"""

from acav.campaign import RunSpec
from acav.campaign_runner import check_record
from acav.metrics import CaptureOutcome, RunResult, TargetResult
from acav.run_record import (
    Provenance,
    RunRecord,
    scenario_targets,
    target_coverage_problems,
    write_record,
)

SCENARIO = scenario_targets()


def _target(target_id):
    return TargetResult(target_id=target_id, attempted=False,
                        outcome=CaptureOutcome.NOT_ATTEMPTED,
                        image_time_s=0.0)


def _record(ids, sha=SCENARIO.sha256, run_id="run"):
    return RunRecord(
        result=RunResult(run_id=run_id, scheduler="dynamics_aware",
                         targets=[_target(i) for i in ids]),
        provenance=Provenance(backend_type="com", execution_mode="free_run",
                              scenario_sha256=sha, real_time_factor=1.0),
        experiment="E3")


ALL = list(SCENARIO.target_ids)
WITHOUT_T4 = [i for i in ALL if i != "T4"]


class TestScenarioTargets:
    def test_read_from_the_scenario_not_hard_coded(self):
        assert SCENARIO.target_ids == ("T1", "T2", "T3", "T4", "T5", "T6",
                                       "T7")
        assert len(SCENARIO.sha256) == 64
        assert SCENARIO.path.endswith("acav_scenario.yaml")

    def test_another_scenario_file_gives_its_own_targets(self, tmp_path):
        text = open(SCENARIO.path, encoding="utf-8").read()
        assert "    - id: T7\n" in text.replace("\r\n", "\n")
        shorter = text.replace("id: T7", "id: T8")
        path = tmp_path / "other.yaml"
        path.write_text(shorter, encoding="utf-8")
        other = scenario_targets(str(path))
        assert other.target_ids[-1] == "T8"
        assert other.sha256 != SCENARIO.sha256


class TestCoverage:
    def test_a_complete_record_passes(self):
        assert target_coverage_problems(_record(ALL), SCENARIO) == []

    def test_the_rtf1_style_record_is_refused_naming_t4(self):
        problems = target_coverage_problems(_record(WITHOUT_T4), SCENARIO)
        assert len(problems) == 1
        assert "missing T4" in problems[0]

    def test_an_extra_target_is_named(self):
        problems = target_coverage_problems(_record(ALL + ["T9"]), SCENARIO)
        assert any("extra T9" in p for p in problems)

    def test_a_duplicate_target_is_named(self):
        problems = target_coverage_problems(_record(ALL + ["T2"]), SCENARIO)
        assert any("duplicate T2" in p for p in problems)

    def test_a_record_from_another_scenario_is_refused(self):
        problems = target_coverage_problems(_record(ALL, sha="0" * 64),
                                            SCENARIO)
        assert len(problems) == 1
        assert "another scenario" in problems[0]

    def test_a_record_naming_no_scenario_is_refused(self):
        problems = target_coverage_problems(_record(ALL, sha=""), SCENARIO)
        assert len(problems) == 1
        assert "names no scenario" in problems[0]


class TestCheckRecord:
    def test_check_record_refuses_a_missing_target(self):
        run = RunSpec(experiment="E3", execution_mode="free_run")
        problems = check_record(run, _record(WITHOUT_T4, run_id=run.run_id),
                                scenario=SCENARIO)
        assert any("missing T4" in p for p in problems)

    def test_check_record_defaults_to_the_installed_scenario(self):
        run = RunSpec(experiment="E3", execution_mode="free_run")
        assert check_record(run, _record(ALL, run_id=run.run_id)) == []
        assert check_record(run, _record(WITHOUT_T4, run_id=run.run_id))


class TestReport:
    def _report(self, tmp_path):
        from acav.campaign_cli import main
        return main(["report", "--experiment", "E6", "--repeats", "1",
                     "--no-figures", "--output-dir", str(tmp_path)])

    def test_report_refuses_naming_the_run_and_target(self, tmp_path,
                                                      caplog):
        write_record(str(tmp_path / "short.json"),
                     _record(WITHOUT_T4, run_id="short"))
        write_record(str(tmp_path / "full.json"), _record(ALL, run_id="full"))
        assert self._report(tmp_path) == 1
        assert "short" in caplog.text and "missing T4" in caplog.text
        assert "full (" not in caplog.text

    def test_report_does_not_refuse_complete_records(self, tmp_path,
                                                     caplog):
        write_record(str(tmp_path / "full.json"), _record(ALL, run_id="full"))
        self._report(tmp_path)
        assert "do not cover" not in caplog.text
