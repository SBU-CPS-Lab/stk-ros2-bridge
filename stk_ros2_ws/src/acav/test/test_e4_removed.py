"""E4 (controller gain sweep) is gone from the code; old records still load.

E4 was dropped from the study (docs/C1_case_study_spec.md section 9). Its
campaign builder, its aggregation, its figure and its CLI choice are
removed. Records already written with the E4 or E5 label must still be
readable by `report` (backward compatible): the label is a string in the
record, never checked against the current experiment list. E5 stays a
CLI choice for the seeded fault campaign until the fault runs become an
E3 arm.
"""

import json

import pytest

import acav
import acav.campaign as campaign
import acav.report_figures as report_figures
from acav.campaign_cli import _parser, load_results, main
from acav.fault_injection import plan_fault
from acav.run_record import Provenance, RunRecord, read_record, write_record

from test_campaign import _result, covering  # noqa: I100

#: Committed before the removal; the removal commit drops it. strict: a
#: test that passes before the removal fails the suite.
PENDING = pytest.mark.xfail(strict=True, reason="E4 not removed yet")

BASE = ["--output-dir", "out"]


@PENDING
def test_e4_is_not_an_experiment_choice():
    with pytest.raises(SystemExit):
        _parser().parse_args(["plan", "--experiment", "E4"] + BASE)


def test_the_remaining_experiments_are_choices():
    for experiment in ("E1", "E2", "E3", "E5", "E6"):
        _parser().parse_args(["plan", "--experiment", experiment] + BASE)


@PENDING
def test_the_e4_code_is_gone():
    assert not hasattr(campaign, "build_e4_gain_sweep")
    assert not hasattr(campaign, "gain_surface")
    assert not hasattr(report_figures, "e4_gain_surface_figure")
    assert "build_e4_gain_sweep" not in acav.__all__
    assert "gain_surface" not in acav.__all__


def _old_style(tmp_path, experiment, run_id, seed=None, fault=None):
    """A record as written before the newest provenance fields existed
    (synthetic test fixture): stk_process, matlab_version, graphics_mode,
    real_time_factor, scenario_seed and generator version removed."""
    record = covering(RunRecord(
        result=_result(run_id, seed=seed),
        provenance=Provenance(backend_type="com", execution_mode="lockstep",
                              fault=fault)))
    raw = record.to_dict()
    raw["experiment"] = experiment
    for key in ("stk_process", "matlab_version", "graphics_mode",
                "real_time_factor", "scenario_seed",
                "scenario_generator_version"):
        raw["provenance"].pop(key, None)
    path = tmp_path / f"{run_id}.json"
    path.write_text(json.dumps(raw), encoding="utf-8")
    return path


def test_old_e4_and_e5_records_still_load(tmp_path):
    fault = plan_fault(0, 720.0).to_dict()
    e4 = _old_style(tmp_path, "E4", "E4-dynamics_aware-lockstep-wn1-auth0p8")
    e5 = _old_style(tmp_path, "E5", "E5-dynamics_aware-lockstep-seed0",
                    seed=0, fault=fault)
    assert read_record(str(e4)).result.run_id.startswith("E4-")
    assert read_record(str(e5)).provenance.fault["seed"] == 0
    loaded = sorted(r.result.run_id for r in load_results(str(tmp_path)))
    assert loaded == ["E4-dynamics_aware-lockstep-wn1-auth0p8",
                      "E5-dynamics_aware-lockstep-seed0"]


def test_report_reads_an_old_e5_record(tmp_path, capsys, caplog):
    fault = plan_fault(0, 720.0).to_dict()
    _old_style(tmp_path, "E5", "E5-dynamics_aware-lockstep-seed0",
               seed=0, fault=fault)
    code = main(["report", "--experiment", "E5", "--seeds", "1",
                 "--no-figures", "--output-dir", str(tmp_path)])
    assert code in (0, 1)              # read and analysed, not refused (2)
    assert "skipping" not in caplog.text
    assert "E5-dynamics_aware-lockstep-seed0" in capsys.readouterr().out


def test_a_record_written_now_keeps_its_label(tmp_path):
    path = str(tmp_path / "r.json")
    write_record(path, RunRecord(result=_result("E2-x"),
                                 provenance=Provenance(backend_type="com")))
    assert read_record(path).result.run_id == "E2-x"
