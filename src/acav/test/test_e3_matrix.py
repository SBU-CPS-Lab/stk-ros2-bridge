"""E3 as one matrix: execution mode x fault x repeats, all launchable.

build_e3_matrix(n_repeats, n_fault_seeds): for the fault cell "none" and
each of K seeded wheel faults (acav.fault_injection.plan_campaign_faults,
one fault per seed, the same in every mode):
  lockstep               2 repeats (to show bit-identical signatures)
  free-run, rtf 1.0      N repeats
  free-run, rtf 0.5      N repeats
    runs = (1 + K) x (2 + 2 N)
on the reference scenario (as build_e3_determinism, campaign.py), ordered
lockstep, then rtf 1.0, then rtf 0.5. The real-time factor reaches each
run through the {real_time_factor} placeholder and the run CLI's existing
--real-time-factor, so free-run fault runs launch like every other run;
the plan emits every run's command, each checked here against the run
CLI's own parser. Free-run rtf 1.0 runs without a fault keep the run ids
build_e3_determinism gave them.

Report: the lockstep replay check (each lockstep cell's repeats share
one signature) and the free-run spread per (rtf, fault) cell, from the
records' existing fields. E3 alone may hold two real-time factors in one
folder (it is a factor of the matrix); a record outside the planned
cells is refused, and every other experiment refuses a folder that mixes
lockstep with free-run. Records are synthetic test fixtures.
"""

import json

import pytest

from acav.campaign import RunSpec
from acav.fault_injection import plan_campaign_faults
from acav.run_record import Provenance, RunRecord, write_record

from test_campaign import _result, covering  # noqa: I100

WINDOW = 720.0


@pytest.mark.parametrize("n, k, expected", [(1, 0, 4), (3, 3, 32),
                                            (5, 5, 72), (2, 1, 12)])
def test_the_run_count_formula(n, k, expected):
    from acav.campaign import build_e3_matrix, e3_run_count
    campaign = build_e3_matrix(n, k, WINDOW)
    assert len(campaign) == expected == e3_run_count(n, k)
    assert len({r.run_id for r in campaign.runs}) == expected


def test_cells_and_order():
    from acav.campaign import build_e3_matrix
    runs = build_e3_matrix(3, 2, WINDOW).runs
    blocks = [(r.execution_mode, r.real_time_factor) for r in runs]
    first_free = blocks.index(("free_run", 1.0))
    first_half = blocks.index(("free_run", 0.5))
    assert set(blocks[:first_free]) == {("lockstep", 1.0)}
    assert set(blocks[first_free:first_half]) == {("free_run", 1.0)}
    assert set(blocks[first_half:]) == {("free_run", 0.5)}
    lockstep = [r for r in runs if r.execution_mode == "lockstep"]
    assert len(lockstep) == 2 * 3
    assert all(r.experiment == "E3" for r in runs)


def test_one_fault_per_seed_the_same_in_every_mode():
    from acav.campaign import build_e3_matrix
    runs = build_e3_matrix(2, 3, WINDOW).runs
    planned = plan_campaign_faults([0, 1, 2], WINDOW)
    faults = {r.fault for r in runs if r.fault is not None}
    assert faults == set(planned.values())
    for fault in faults:
        modes = {(r.execution_mode, r.real_time_factor) for r in runs
                 if r.fault == fault}
        assert modes == {("lockstep", 1.0), ("free_run", 1.0),
                         ("free_run", 0.5)}


def test_no_fault_rtf1_runs_keep_build_e3_determinism_ids():
    from acav.campaign import build_e3_determinism, build_e3_matrix
    old = {r.run_id for r in build_e3_determinism(2).runs}
    new = {r.run_id for r in build_e3_matrix(2, 0, WINDOW).runs}
    assert old <= new


def test_the_run_id_names_the_half_rate():
    run = RunSpec(experiment="E3", execution_mode="free_run", seed=0,
                  real_time_factor=0.5)
    assert run.run_id == "E3-dynamics_aware-free_run-seed0-rtf0p5"


def test_every_emitted_command_parses_and_says_what_the_run_is(tmp_path):
    from acav.campaign import build_e3_matrix
    from acav.campaign_runner import planned_commands
    from acav.deploy_run_cli import _parser
    campaign = build_e3_matrix(2, 2, WINDOW)
    commands = planned_commands(campaign, str(tmp_path), "matlab-batch")
    assert len(commands) == len(campaign)
    for run, argv in zip(campaign.runs, commands):
        assert argv[:2] == ["python", "src\\acav\\scripts\\run_acav_run.py"]
        args = _parser().parse_args(argv[2:])
        assert args.run_id == run.run_id
        assert args.execution_mode == run.execution_mode
        assert args.real_time_factor == run.real_time_factor
        assert args.twin_launch == "matlab-batch"
        assert not args.allow_com_started_stk
        fault = json.loads(args.fault_json)
        assert fault == (None if run.fault is None else run.fault.to_dict())
        assert args.run_config.endswith(f"{run.run_id}.runconfig.json")


def test_the_launcher_substitutes_the_real_time_factor():
    import inspect

    from acav.campaign_runner import CommandLauncher
    assert '"real_time_factor"' in inspect.getsource(CommandLauncher.__call__)


def test_plan_prints_formula_times_and_combined_campaign(tmp_path, capsys):
    from acav.campaign_cli import main
    code = main(["plan", "--experiment", "E3", "--repeats", "3",
                 "--fault-seeds", "3", "--output-dir", str(tmp_path)])
    out = capsys.readouterr().out
    assert code == 0
    assert ("E3 runs = (1 + N_FAULT_SEEDS) x (2 lockstep + 2 x N_REPEATS "
            "free-run) = (1 + 3) x (2 + 2 x 3) = 32") in out
    # Measured once in the live session on d0bf796 (summary.json
    # started_at to finished_at of LIVE_E3_d0bf796's one run).
    assert ("free-run rtf 0.5: 27.3 min (1 run: LIVE_E3_d0bf796 "
            "E3-dynamics_aware-free_run-seed0-rtf0p5-faultx218p375)") in out
    assert "not measured" not in out
    assert "total measured part: 16.75-18.58 h" in out
    assert "combined campaign" in out
    assert "E2 (10 seeds x 2 arms): 20 lockstep runs" in out
    assert "python src\\acav\\scripts\\run_acav_run.py" in out
    assert "resume:" in out


# --- report --------------------------------------------------------------------

def _record(tmp_path, run, signature_tweak=0.0, saturations=1,
            captured=True):
    result = _result(run.run_id, seed=run.seed, captured=captured,
                     error=0.2 + signature_tweak)
    result.wheel_saturation_events = saturations
    record = covering(RunRecord(result=result, provenance=Provenance(
        backend_type="com", execution_mode=run.execution_mode,
        real_time_factor=run.real_time_factor, graphics_mode="off",
        fault=None if run.fault is None else run.fault.to_dict())))
    write_record(str(tmp_path / f"{run.run_id}.json"), record)


def test_report_checks_lockstep_replay_and_free_run_spread(tmp_path,
                                                            capsys):
    from acav.campaign import build_e3_matrix
    from acav.campaign_cli import main
    campaign = build_e3_matrix(2, 1, WINDOW)
    for run in campaign.runs:
        free = run.execution_mode == "free_run"
        _record(tmp_path, run,
                signature_tweak=(0.001 * run.seed if free else 0.0),
                saturations=(run.seed if free else 1))
    code = main(["report", "--experiment", "E3", "--repeats", "2",
                 "--fault-seeds", "1", "--no-figures", "--output-dir",
                 str(tmp_path)])
    report = json.loads(capsys.readouterr().out)
    replay = report["lockstep_replay"]
    assert set(replay) == {"none", "fault seed 0"}
    assert all(cell["reproduced"] for cell in replay.values())
    spread = report["free_run_spread"]
    cell = spread["rtf 1.0, none"]
    assert cell["runs"] == 2
    assert cell["saturation_events"] == {"min": 0, "max": 1}
    assert cell["distinct_signatures"] == 2
    assert code == 0


def test_a_lockstep_replay_that_differs_is_named(tmp_path, capsys):
    from acav.campaign import build_e3_matrix
    from acav.campaign_cli import main
    campaign = build_e3_matrix(1, 1, WINDOW)
    lockstep = [r for r in campaign.runs if r.execution_mode == "lockstep"]
    for run in campaign.runs:
        tweak = 0.5 if run is lockstep[-1] else 0.0
        _record(tmp_path, run, signature_tweak=tweak)
    code = main(["report", "--experiment", "E3", "--repeats", "1",
                 "--fault-seeds", "1", "--no-figures", "--output-dir",
                 str(tmp_path)])
    report = json.loads(capsys.readouterr().out)
    assert report["lockstep_replay"]["fault seed 0"]["reproduced"] is False
    assert code == 1


def test_e3_refuses_a_record_outside_the_planned_cells(tmp_path, capsys,
                                                        caplog):
    from acav.campaign import build_e3_matrix
    from acav.campaign_cli import main
    for run in build_e3_matrix(1, 0, WINDOW).runs:
        _record(tmp_path, run)
    stray = RunSpec(experiment="E3", execution_mode="free_run", seed=7,
                    real_time_factor=2.0)
    _record(tmp_path, stray)
    code = main(["report", "--experiment", "E3", "--repeats", "1",
                 "--no-figures", "--output-dir", str(tmp_path)])
    assert code == 1
    assert "outside the planned E3 cells" in caplog.text
    assert stray.run_id in caplog.text


def test_other_experiments_refuse_mixed_execution_modes(tmp_path, capsys,
                                                         caplog):
    from acav.campaign_cli import main
    lock = RunSpec(experiment="E6", seed=0)
    free = RunSpec(experiment="E6", execution_mode="free_run", seed=1,
                   real_time_factor=1.0)
    _record(tmp_path, lock)
    _record(tmp_path, free)
    code = main(["report", "--experiment", "E6", "--repeats", "2",
                 "--no-figures", "--output-dir", str(tmp_path)])
    assert code == 1
    assert "execution modes" in caplog.text
    assert free.run_id in caplog.text


def test_check_record_compares_the_real_time_factor():
    from acav.campaign_runner import check_record
    run = RunSpec(experiment="E3", execution_mode="free_run", seed=0,
                  real_time_factor=0.5)
    record = covering(RunRecord(result=_result(run.run_id, seed=0),
                                provenance=Provenance(
                                    backend_type="com",
                                    execution_mode="free_run",
                                    real_time_factor=1.0)))
    assert any("real_time_factor" in p for p in check_record(run, record))


def test_old_e5_records_still_load(tmp_path):
    from acav.fault_injection import plan_fault
    from acav.run_record import read_record
    from test_e4_removed import _old_style
    path = _old_style(tmp_path, "E5", "E5-dynamics_aware-lockstep-seed4",
                      seed=4, fault=plan_fault(4, WINDOW).to_dict())
    assert read_record(str(path)).provenance.fault["seed"] == 4
