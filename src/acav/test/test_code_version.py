"""Every run records the code that produced it.

The first E6 campaigns were run from an unmerged branch, and nothing in
their records says which commit -- the paper's methods section could only
have said so from memory. These tests pin the recording path end to end.
"""

import json
import os
import subprocess

import pytest

from acav import code_version
from acav.campaign import build_e6_latency
from acav.campaign_cli import code_version_summary
from acav.campaign_runner import write_manifest, write_run_config
from acav.metrics import RunResult
from acav.run_record import Provenance, RunRecord, read_record, write_record


def _git(repo, *args):
    subprocess.run(["git", "-C", str(repo), *args], check=True,
                   capture_output=True)


@pytest.fixture
def repo(tmp_path):
    root = tmp_path / "repo"
    root.mkdir()
    _git(root, "init", "-q")
    _git(root, "config", "user.email", "t@example.org")
    _git(root, "config", "user.name", "t")
    (root / "a.txt").write_text("one\n")
    _git(root, "add", "a.txt")
    _git(root, "commit", "-q", "-m", "first")
    return root


class TestDescribeCheckout:
    def test_a_clean_checkout_is_its_commit(self, repo):
        version = code_version.describe_checkout(str(repo))
        assert code_version.is_clean(version)

    def test_a_modified_tracked_file_is_dirty(self, repo):
        (repo / "a.txt").write_text("two\n")
        version = code_version.describe_checkout(str(repo))
        assert version.endswith("+dirty")
        assert not code_version.is_clean(version)

    def test_untracked_files_do_not_make_it_dirty(self, repo):
        (repo / "notes.md").write_text("local\n")
        assert code_version.is_clean(code_version.describe_checkout(str(repo)))

    def test_outside_a_checkout_is_none(self, tmp_path):
        assert code_version.describe_checkout(str(tmp_path)) is None


class TestDescribe:
    def test_the_environment_wins_by_default(self, monkeypatch):
        monkeypatch.setenv(code_version.ENV_VAR, "abc+dirty")
        assert code_version.describe() == "abc+dirty"

    def test_the_orchestrator_ignores_a_stale_environment(self, monkeypatch):
        monkeypatch.setenv(code_version.ENV_VAR, "stale")
        assert code_version.describe(prefer_env=False) != "stale"

    def test_this_repository_resolves(self, monkeypatch):
        monkeypatch.delenv(code_version.ENV_VAR, raising=False)
        version = code_version.describe()
        assert not version.startswith("unknown"), version


class TestRecorded:
    def test_provenance_round_trips(self, tmp_path):
        path = str(tmp_path / "r.json")
        write_record(path, RunRecord(
            result=RunResult(run_id="r", scheduler="dynamics_aware"),
            experiment="E6",
            provenance=Provenance(backend_type="com", code_version="f" * 40)))
        assert read_record(path).provenance.code_version == "f" * 40

    def test_an_older_record_without_it_still_loads(self, tmp_path):
        path = str(tmp_path / "old.json")
        write_record(path, RunRecord(
            result=RunResult(run_id="r", scheduler="dynamics_aware"),
            experiment="E6", provenance=Provenance(backend_type="com")))
        raw = json.load(open(path))
        del raw["provenance"]["code_version"]
        json.dump(raw, open(path, "w"))
        assert read_record(path).provenance.code_version == ""

    def test_run_config_and_manifest_carry_it(self, tmp_path, monkeypatch):
        monkeypatch.setenv(code_version.ENV_VAR, "c" * 40)
        campaign = build_e6_latency(repeats=1)
        run = campaign.runs[0]
        config = write_run_config(run, str(tmp_path / run.output_name))
        assert json.load(open(config))["code_version"] == "c" * 40
        manifest = write_manifest(campaign, str(tmp_path))
        assert json.load(open(manifest))["code_version"] == "c" * 40


def _record(run_id, version):
    return RunRecord(result=RunResult(run_id=run_id, scheduler="dynamics_aware"),
                     experiment="E6",
                     provenance=Provenance(backend_type="com",
                                           code_version=version))


class TestReportSummary:
    def test_one_clean_commit_is_attributable(self):
        summary = code_version_summary([_record("a", "a" * 40),
                                        _record("b", "a" * 40)])
        assert summary["attributable"]

    @pytest.mark.parametrize("versions", [
        ("a" * 40, "b" * 40),            # two commits
        ("a" * 40 + "+dirty",),          # modified code
        ("",),                           # recorded before this existed
    ])
    def test_anything_else_is_not(self, versions):
        records = [_record(str(i), v) for i, v in enumerate(versions)]
        summary = code_version_summary(records)
        assert not summary["attributable"]
        if "" in versions:
            assert "not recorded" in summary["versions"]


class TestSchedulerProvenance:
    """What the scheduler actually writes, through its real _write_output."""

    def _write(self, tmp_path, monkeypatch, scenario_path=""):
        from types import SimpleNamespace
        from acav.scheduler_node import SchedulerNode
        monkeypatch.setenv(code_version.ENV_VAR, "d" * 40)
        node = SchedulerNode.__new__(SchedulerNode)
        out = str(tmp_path / "run.json")
        node._options = SimpleNamespace(
            output_json=out, experiment="E6", scenario_path=scenario_path,
            gains_json="{}", fault_json="null", attitude_provider="")
        node._backend_type, node._stk_version = "com", "12.6.0"
        node._execution_mode, node._started_utc = "lockstep", ""
        node._readback_provider, node._readback_forced = "", None
        node._write_output(RunResult(run_id="r", scheduler="dynamics_aware"))
        return read_record(out).provenance

    def test_the_default_scenario_is_fingerprinted(self, tmp_path, monkeypatch):
        # Every campaign so far ran with no explicit scenario path, and
        # every record's scenario_sha256 came out empty.
        from acav.run_record import scenario_digest
        from acav.spec import AcavSpec
        provenance = self._write(tmp_path, monkeypatch)
        assert provenance.scenario_sha256 == scenario_digest(AcavSpec.locate(None))
        assert provenance.code_version == "d" * 40
