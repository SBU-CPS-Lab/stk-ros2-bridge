"""A re-run never leaves the previous attempt where it could be mistaken
for the new one.

Before this, re-running a run (--no-resume, or a record that did not
match its run) left the old record at the output path while the new
attempt ran. A launcher that then exited cleanly without writing made
the runner read the OLD record as the new result. The old record, its
run configuration and its logs are now moved aside first, under names
that neither `report` (*.json) nor the diagnostic scan (*.logs) reads,
and kept: nothing is deleted.
"""

import glob
import os

from acav.campaign import Campaign, RunSpec
from acav.campaign_cli import load_results
from acav.campaign_runner import (
    RunnerOptions,
    run_campaign,
    run_config_path,
)

from test_campaign import _writing_launcher   # noqa: I100 - shared helper


def _campaign(seed=4):
    return Campaign("c", "E5", "", runs=(RunSpec(experiment="E5",
                                                 seed=seed),))


def _paths(tmp_path):
    run = _campaign().runs[0]
    output = os.path.join(str(tmp_path), run.output_name)
    base = os.path.splitext(output)[0]
    return output, base + ".logs", run_config_path(output)


def _first_attempt(tmp_path, launcher=None):
    run_campaign(_campaign(), launcher or _writing_launcher(),
                 RunnerOptions(output_dir=str(tmp_path)))
    output, logs, _ = _paths(tmp_path)
    os.makedirs(logs)
    with open(os.path.join(logs, "run.log"), "w", encoding="utf-8") as f:
        f.write("first attempt\n")
    with open(output, "rb") as handle:
        return handle.read()


def _superseded(tmp_path):
    return sorted(os.path.basename(p) for p in
                  glob.glob(os.path.join(str(tmp_path), "*.superseded-*")))


def test_a_clean_exit_without_a_record_no_longer_reuses_the_old_one(
        tmp_path):
    _first_attempt(tmp_path)
    outcome = run_campaign(_campaign(), lambda *_: None, RunnerOptions(
        output_dir=str(tmp_path), resume=False))
    assert outcome.outcomes[0].status == "failed"
    assert "wrote no record" in outcome.outcomes[0].detail
    assert outcome.records == []


def test_the_previous_attempt_is_kept_aside(tmp_path):
    old = _first_attempt(tmp_path)
    run_campaign(_campaign(), _writing_launcher(), RunnerOptions(
        output_dir=str(tmp_path), resume=False))
    output, logs, config = _paths(tmp_path)
    kept = _superseded(tmp_path)
    assert len(kept) == 3
    record = [n for n in kept if n.startswith(os.path.basename(output))]
    assert len(record) == 1
    with open(os.path.join(str(tmp_path), record[0]), "rb") as handle:
        assert handle.read() == old
    kept_logs = [n for n in kept if n.startswith(os.path.basename(logs))]
    with open(os.path.join(str(tmp_path), kept_logs[0], "run.log"),
              encoding="utf-8") as handle:
        assert handle.read() == "first attempt\n"
    assert any(n.startswith(os.path.basename(config)) for n in kept)
    assert os.path.exists(output) and os.path.exists(config)
    assert not os.path.exists(logs)


def test_kept_attempts_are_invisible_to_report(tmp_path):
    _first_attempt(tmp_path)
    run_campaign(_campaign(), _writing_launcher(), RunnerOptions(
        output_dir=str(tmp_path), resume=False))
    for name in _superseded(tmp_path):
        assert not name.endswith(".json") and not name.endswith(".logs")
    assert len(load_results(str(tmp_path))) == 1


def test_a_mismatched_record_is_kept_aside_when_re_run(tmp_path):
    _first_attempt(tmp_path, _writing_launcher(seed=99))
    outcome = run_campaign(_campaign(), _writing_launcher(), RunnerOptions(
        output_dir=str(tmp_path)))
    assert len(outcome.records) == 1
    assert len(_superseded(tmp_path)) == 3


def test_a_resumed_run_moves_nothing(tmp_path):
    _first_attempt(tmp_path)
    run_campaign(_campaign(), _writing_launcher(), RunnerOptions(
        output_dir=str(tmp_path)))
    assert _superseded(tmp_path) == []


def test_a_dry_run_moves_nothing(tmp_path):
    _first_attempt(tmp_path)
    run_campaign(_campaign(), _writing_launcher(), RunnerOptions(
        output_dir=str(tmp_path), resume=False, dry_run=True))
    assert _superseded(tmp_path) == []


def test_a_first_attempt_creates_nothing_extra(tmp_path):
    run_campaign(_campaign(), _writing_launcher(), RunnerOptions(
        output_dir=str(tmp_path)))
    assert _superseded(tmp_path) == []


def test_repeated_re_runs_never_overwrite_a_kept_attempt(tmp_path):
    _first_attempt(tmp_path)
    for _ in range(3):
        run_campaign(_campaign(), _writing_launcher(), RunnerOptions(
            output_dir=str(tmp_path), resume=False))
    output, _, config = _paths(tmp_path)
    kept = _superseded(tmp_path)
    assert len([n for n in kept
                if n.startswith(os.path.basename(output) + ".")]) == 3
    assert len([n for n in kept
                if n.startswith(os.path.basename(config) + ".")]) == 3


def test_a_previous_attempt_that_cannot_be_moved_fails_that_run(
        tmp_path, monkeypatch):
    """E.g. a log still held open by a live bridge on Windows: launching
    with the old record still in place is exactly what this prevents."""
    _first_attempt(tmp_path)
    launched = []

    def refuse(src, dst):
        raise PermissionError("in use")

    monkeypatch.setattr(os, "replace", refuse)
    outcome = run_campaign(_campaign(),
                           lambda run, c, p: launched.append(run),
                           RunnerOptions(output_dir=str(tmp_path),
                                         resume=False))
    assert launched == []
    assert outcome.outcomes[0].status == "failed"
    assert "previous attempt" in outcome.outcomes[0].detail
