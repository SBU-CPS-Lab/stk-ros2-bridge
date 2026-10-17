"""`report --experiment E2` analyses seeded scenarios as pairs.

Each seeded scenario is its own E2 campaign folder (two runs with the
same run ids in every folder), so `report` reads the extra folders given
with --include-dir. The pairing unit is the scenario seed; a pair is the
seed's kinematic and dynamics-aware record; the difference is
dynamics-aware minus kinematic. Incomplete pairs are dropped and listed
by name, never imputed. The analysis refuses, naming the records, when
the records differ in code version, twin identity (build and runtime
fingerprints and polling rate; the run-config digest differs by design),
how STK was started, execution mode or generator version, or when a
record is not from a seeded scenario.

Records here are SYNTHETIC TEST FIXTURES built on the committed
generated scenarios (their target lists), not measurements.
"""

import dataclasses
import json
import os

import pytest

from acav.campaign_cli import main
from acav.metrics import CaptureOutcome, RunResult, TargetResult
from acav.run_record import (Provenance, RunRecord, scenario_targets,
                              write_record)

PENDING = pytest.mark.xfail(strict=True,
                            reason="paired E2 report not written yet")

HERE = os.path.dirname(os.path.abspath(__file__))
GENERATED = os.path.join(HERE, "..", "config", "scenarios", "generated")
IDENTITY = "v3 {cfg} 1111222233334444 1111222233334444 100Hz"


def _scenario(seed):
    return scenario_targets(os.path.join(GENERATED, f"acav_seed{seed}.yaml"))


def _record(seed, scheduler, captured, **provenance):
    scenario = _scenario(seed)
    targets = [TargetResult(
        target_id=t, attempted=True, image_time_s=60.0,
        outcome=(CaptureOutcome.CAPTURED if i < captured
                 else CaptureOutcome.NOT_SETTLED),
        error_at_image_deg=0.01, max_error_in_hold_deg=0.01)
        for i, t in enumerate(scenario.target_ids)]
    fields = dict(backend_type="com", execution_mode="lockstep",
                  scenario_sha256=scenario.sha256, scenario_seed=seed,
                  scenario_generator_version="2",
                  code_version="0123456789abcdef0123456789abcdef01234567",
                  twin_identity=IDENTITY.format(cfg=f"{seed}{scheduler}"),
                  graphics_mode="off", real_time_factor=1.0,
                  stk_process={"pid": 1, "started_by": "operator"})
    fields.update(provenance)
    return RunRecord(
        result=RunResult(run_id=f"E2-{scheduler}-lockstep",
                         scheduler=scheduler, targets=targets,
                         wheel_energy_J=1.0 + captured,
                         wheel_saturation_events=captured % 3),
        provenance=Provenance(**fields))


def _folders(tmp_path, captures, overrides=None):
    """captures: {seed: (kinematic captured, dynamics_aware captured)};
    None leaves that run out. overrides: {(seed, scheduler): provenance}."""
    overrides = overrides or {}
    folders = []
    for seed, pair in captures.items():
        folder = tmp_path / f"E2_seed{seed}"
        folder.mkdir()
        for scheduler, captured in zip(("kinematic", "dynamics_aware"),
                                       pair):
            if captured is None:
                continue
            record = _record(seed, scheduler, captured,
                             **overrides.get((seed, scheduler), {}))
            write_record(str(folder / f"{record.result.run_id}.json"),
                         record)
        folders.append(str(folder))
    return folders


def _report(folders, capsys):
    argv = ["report", "--experiment", "E2", "--no-figures",
            "--output-dir", folders[0]]
    for folder in folders[1:]:
        argv += ["--include-dir", folder]
    code = main(argv)
    return code, capsys.readouterr().out


SIX = {seed: (10, 14 + seed % 3) for seed in range(1, 7)}


@PENDING
def test_six_seeds_are_analysed_as_pairs(tmp_path, capsys):
    code, out = _report(_folders(tmp_path, SIX), capsys)
    assert code == 0
    report = json.loads(out[:out.index("\nminimum attainable")])
    paired = report["paired"]
    assert paired["n_pairs"] == 6
    assert paired["direction"] == "dynamics_aware - kinematic"
    assert paired["dropped_incomplete"] == []
    coverage = paired["metrics"]["coverage_rate"]
    assert coverage["n_pairs"] == 6
    assert all(d > 0 for d in coverage["differences"])
    assert coverage["p_exact"] == pytest.approx(0.03125)
    assert "p_holm" in coverage
    assert paired["bootstrap"]["seed"] == paired["metrics"][
        "coverage_rate"]["bootstrap_median_ci"]["seed"]
    assert paired["bootstrap"]["resamples"] == 10000
    assert ("minimum attainable two-sided exact p for n = 6 pairs: "
            "0.03125") in out


@PENDING
def test_an_incomplete_pair_is_dropped_and_named(tmp_path, capsys):
    captures = dict(SIX)
    captures[7] = (10, None)
    code, out = _report(_folders(tmp_path, captures), capsys)
    assert code == 0
    report = json.loads(out[:out.index("\nminimum attainable")])
    assert report["paired"]["n_pairs"] == 6
    assert report["paired"]["dropped_incomplete"] == [
        "seed 7: kinematic only (E2_seed7/E2-kinematic-lockstep)"]


@PENDING
def test_three_seeds_print_the_underpowered_flag(tmp_path, capsys):
    three = {seed: SIX[seed] for seed in (1, 2, 3)}
    code, out = _report(_folders(tmp_path, three), capsys)
    assert code == 0
    assert ("minimum attainable two-sided exact p for n = 3 pairs: 0.25 "
            "-- ABOVE 0.05: underpowered") in out


@PENDING
@pytest.mark.parametrize("override, words", [
    ({"code_version": "fedcba9876543210fedcba9876543210fedcba98"},
     "code version"),
    ({"twin_identity": "v3 x 9999888877776666 9999888877776666 100Hz"},
     "twin identity"),
    ({"stk_process": {"pid": 2, "started_by": "com"}}, "STK"),
    ({"execution_mode": "free_run"}, "execution mode"),
    ({"scenario_generator_version": "1"}, "generator version"),
])
def test_differences_are_refused_naming_the_records(tmp_path, capsys,
                                                     caplog, override,
                                                     words):
    folders = _folders(tmp_path, SIX, {(3, "dynamics_aware"): override})
    code, out = _report(folders, capsys)
    assert code == 1
    assert words in caplog.text
    assert "E2_seed3/E2-dynamics_aware-lockstep" in caplog.text
    assert '"paired"' not in out


@PENDING
def test_a_record_not_from_a_seeded_scenario_is_refused(tmp_path, capsys,
                                                         caplog):
    folders = _folders(tmp_path, SIX)
    reference = scenario_targets()
    record = _record(1, "kinematic", 3)
    record = dataclasses.replace(record, provenance=dataclasses.replace(
        record.provenance, scenario_seed=None, scenario_generator_version="",
        scenario_sha256=reference.sha256))
    record.result.targets = [TargetResult(
        target_id=t, attempted=False, outcome=CaptureOutcome.NOT_ATTEMPTED,
        image_time_s=0.0) for t in reference.target_ids]
    extra = tmp_path / "E2_reference"
    extra.mkdir()
    write_record(str(extra / "E2-kinematic-lockstep.json"), record)
    code, out = _report(folders + [str(extra)], capsys)
    assert code == 1
    assert "not from a seeded scenario" in caplog.text
    assert "E2_reference/E2-kinematic-lockstep" in caplog.text


def test_one_folder_still_reports_as_before(tmp_path, capsys):
    folders = _folders(tmp_path, {1: (10, 14)})
    code, out = _report(folders, capsys)
    report = json.loads(out)
    assert "paired" not in report
    assert "schedulers" in report
