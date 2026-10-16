"""Unattended campaigns: run_acav_campaign --twin-launch matlab-batch.

The campaign passes the launch mode to every run (each run's command
gets ``--twin-launch matlab-batch``) and records it in campaign.json.
A run whose summary.json records a failed step -- a MATLAB session that
exited early, timed out or stopped uncleanly among them -- is not
resumed over: it is re-run, and its bundle is kept as
``<name>.superseded-<time>``. Before each automated run, any MATLAB or
libmwros2server process left by the previous run (found from the PIDs
that run recorded, and the processes those started) is killed and the
cleanup recorded in twin_cleanup.jsonl. Manual campaigns are unchanged.
"""

import glob
import json
import os

import pytest

from acav import twin_launcher as tl
from acav.campaign import Campaign, RunSpec
from acav.campaign_runner import (
    CommandLauncher,
    RunnerOptions,
    run_campaign,
    write_manifest,
)

from test_campaign import _writing_launcher   # noqa: I100 - shared helper


def _campaign(n=3):
    return Campaign("c", "E6", "", runs=tuple(
        RunSpec(experiment="E6", seed=k) for k in range(1, n + 1)))


def _summary(path, *, failed_step=None, launcher_pid=None,
             matlab_pid=None):
    logs = path[:-len(".json")] + ".logs"
    os.makedirs(logs, exist_ok=True)
    steps = [{"step": "twin_acking", "ok": True, "detail": ""},
             {"step": "matlab_session", "ok": failed_step is None,
              "detail": "clean_stop" if failed_step is None else
              failed_step}]
    with open(os.path.join(logs, "summary.json"), "w",
              encoding="utf-8") as handle:
        json.dump({"run_id": os.path.basename(path)[:-5],
                   "twin_launch": "matlab-batch", "steps": steps,
                   "matlab_session_result": {"how": "clean_stop",
                                             "exit_code": 0,
                                             "pid": launcher_pid}},
                  handle)
    if matlab_pid is not None:
        with open(os.path.join(logs, tl.MATLAB_SESSION_FILE), "w",
                  encoding="utf-8") as handle:
            json.dump({"pid": matlab_pid}, handle)


def _automated_launcher(fail_once=None, pids=None):
    """Writes a record and an automated summary per run; ``fail_once``
    names a seed whose first attempt records a failed MATLAB session
    and exits non-zero, as the runner does."""
    calls = []
    write = _writing_launcher()

    def launch(run, campaign, path):
        calls.append(run.run_id)
        write(run, campaign, path)
        failed = fail_once == run.seed and calls.count(run.run_id) == 1
        launcher_pid, matlab_pid = (pids or {}).get(run.seed, (None, None))
        _summary(path, failed_step="early_exit" if failed else None,
                 launcher_pid=launcher_pid, matlab_pid=matlab_pid)
        if failed:
            raise RuntimeError("launch command exited 1")
    launch.calls = calls
    return launch


class TestCommand:
    def _argv(self, monkeypatch, launcher):
        seen = []
        monkeypatch.setattr(
            "acav.campaign_runner.subprocess.run",
            lambda argv, **k: seen.append(argv) or type(
                "C", (), {"returncode": 0})())
        launcher(RunSpec(experiment="E6", seed=1), _campaign(1), "o.json")
        return seen[0]

    def test_automated_runs_get_the_option(self, monkeypatch):
        argv = self._argv(monkeypatch, CommandLauncher(
            ["python", "run_acav_run.py", "--run-id", "{run_id}"],
            twin_launch="matlab-batch"))
        assert argv[-2:] == ["--twin-launch", "matlab-batch"]

    def test_an_explicit_placeholder_is_used_instead(self, monkeypatch):
        argv = self._argv(monkeypatch, CommandLauncher(
            ["python", "x.py", "--twin-launch", "{twin_launch}"],
            twin_launch="matlab-batch"))
        assert argv == ["python", "x.py", "--twin-launch", "matlab-batch"]

    def test_manual_commands_are_unchanged(self, monkeypatch):
        template = ["python", "run_acav_run.py", "--run-id", "{run_id}"]
        assert self._argv(monkeypatch, CommandLauncher(template)) == \
            self._argv(monkeypatch, CommandLauncher(
                template, twin_launch="manual"))
        assert self._argv(monkeypatch, CommandLauncher(template)) == \
            ["python", "run_acav_run.py", "--run-id", "E6-dynamics_aware-"
             "lockstep-seed1"]


class TestCampaign:
    def test_three_automated_runs(self, tmp_path):
        launcher = _automated_launcher()
        options = RunnerOptions(output_dir=str(tmp_path),
                                twin_launch="matlab-batch",
                                find_processes=lambda pids: [],
                                kill_tree=lambda pid: None)
        outcome = run_campaign(_campaign(), launcher, options)
        assert outcome.complete and len(outcome.records) == 3
        manifest = json.load(open(tmp_path / "campaign.json"))
        assert manifest["twin_launch"] == "matlab-batch"

    def test_a_failed_matlab_session_is_retried_on_resume(self, tmp_path):
        launcher = _automated_launcher(fail_once=2)
        options = RunnerOptions(output_dir=str(tmp_path),
                                twin_launch="matlab-batch",
                                find_processes=lambda pids: [],
                                kill_tree=lambda pid: None)
        first = run_campaign(_campaign(), launcher, options)
        assert [o.status for o in first.outcomes] == [
            "completed", "failed", "completed"]
        second = run_campaign(_campaign(), launcher, options)
        assert [o.status for o in second.outcomes] == [
            "skipped", "completed", "skipped"]
        assert launcher.calls.count("E6-dynamics_aware-lockstep-seed2") == 2
        kept = glob.glob(str(tmp_path / "*seed2*.superseded-*"))
        assert kept, "the failed attempt's bundle is kept"
        assert len(second.records) == 3

    def test_a_leftover_twin_is_killed_before_the_next_run(self, tmp_path):
        launcher = _automated_launcher(pids={1: (500, 501), 2: (600, 601)})
        alive = {501: "MATLAB.exe", 777: "libmwros2server.exe",
                 999: "notepad.exe"}
        asked, killed = [], []

        def find(pids):
            asked.append(sorted(pids))
            # 777 was started by 501; 999 is unrelated.
            return [(p, alive[p]) for p in (501, 777) if 501 in pids]

        options = RunnerOptions(output_dir=str(tmp_path),
                                twin_launch="matlab-batch",
                                find_processes=find,
                                kill_tree=killed.append)
        run_campaign(_campaign(), launcher, options)
        assert asked[0] == [500, 501]          # before run 2: run 1's PIDs
        assert killed == [501, 777]
        lines = (tmp_path / "twin_cleanup.jsonl").read_text().splitlines()
        entry = json.loads(lines[0])
        assert entry["after_run"] == "E6-dynamics_aware-lockstep-seed1"
        assert entry["killed"] == [[501, "MATLAB.exe"],
                                   [777, "libmwros2server.exe"]]

    def test_only_twin_processes_are_killed(self, tmp_path):
        launcher = _automated_launcher(pids={1: (500, 501)})
        killed = []
        options = RunnerOptions(
            output_dir=str(tmp_path), twin_launch="matlab-batch",
            find_processes=lambda pids: [(501, "notepad.exe")],
            kill_tree=killed.append)
        run_campaign(_campaign(2), launcher, options)
        assert killed == []

    def test_manual_campaigns_never_look_for_processes(self, tmp_path):
        def fail(*a):
            raise AssertionError("manual campaigns do not clean up")

        options = RunnerOptions(output_dir=str(tmp_path),
                                find_processes=fail, kill_tree=fail)
        assert run_campaign(_campaign(), _writing_launcher(),
                            options).complete
        assert not (tmp_path / "twin_cleanup.jsonl").exists()


class TestManualManifest:
    def test_the_manifest_is_unchanged(self, tmp_path):
        write_manifest(_campaign(), str(tmp_path))
        manual = json.load(open(tmp_path / "campaign.json"))
        assert "twin_launch" not in manual
        expected = _campaign().to_dict()
        manual.pop("code_version")
        assert manual == expected


class TestReport:
    def _directory(self, tmp_path, modes):
        from test_campaign import _record, covering
        from acav.run_record import write_record

        for seed, mode in enumerate(modes):
            name = f"E6-dynamics_aware-lockstep-seed{seed}"
            write_record(str(tmp_path / f"{name}.json"),
                         covering(_record(name, seed=seed)))
            logs = tmp_path / f"{name}.logs"
            logs.mkdir()
            summary = {"run_id": name}
            if mode:
                summary["twin_launch"] = mode
            (logs / "summary.json").write_text(json.dumps(summary),
                                               encoding="utf-8")

    def _report(self, tmp_path):
        from acav.campaign_cli import main
        return main(["report", "--experiment", "E6", "--repeats", "2",
                     "--no-figures", "--output-dir", str(tmp_path)])

    def test_mixed_launch_modes_are_refused(self, tmp_path, caplog):
        self._directory(tmp_path, [None, "matlab-batch"])
        assert self._report(tmp_path) == 1
        assert "twin launch" in caplog.text

    @pytest.mark.parametrize("modes", [[None, None],
                                       ["matlab-batch", "matlab-batch"]])
    def test_one_launch_mode_is_reported(self, tmp_path, caplog, modes):
        self._directory(tmp_path, modes)
        self._report(tmp_path)
        assert "twin launch" not in caplog.text

    def test_the_cli_option(self):
        from acav.campaign_cli import _parser
        args = ["run", "--experiment", "E6", "--output-dir", "x"]
        assert _parser().parse_args(args).twin_launch == "manual"
        assert _parser().parse_args(
            args + ["--twin-launch", "matlab-batch"]).twin_launch == \
            "matlab-batch"
