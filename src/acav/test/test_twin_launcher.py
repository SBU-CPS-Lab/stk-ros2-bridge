"""The runner can start and stop the MATLAB twin itself (--twin-launch).

``manual`` (the default) is unchanged: the runner prompts, a person
builds the model and presses Play, and stops it after the run.
``matlab-batch`` starts ``matlab -batch "run_twin_session(...)"`` as a
child process (acav.twin_launcher.MatlabBatchTwin): the driver builds
the twin for this run's config, starts the simulation, and stops it and
closes the model when the runner writes the stop file. The ack wait,
the v3 identity check and the free-run start gate stay authoritative:
the runner still refuses to start the scheduler without them. If the
process does not exit after the stop file, the runner kills the process
tree it started -- by its own PID, never another MATLAB instance.
"""

import json
import os

import pytest

import acav.deploy_run_cli as deploy_run_cli
from acav import twin_launcher as tl
from acav.deploy_run_cli import ConnectResult, Deps, execute_run
from acav.run_record import Provenance, RunRecord, read_record, write_record
from acav.metrics import RunResult

from test_deploy_run_cli import (  # noqa: I100, F401 - shared helpers
    FakeProcess, _config, offline_graph)


# -- the launcher ---------------------------------------------------------

class _Process:
    """A MATLAB process that exits when told to (or never)."""

    def __init__(self, pid=4242, exits_after_polls=None, returncode=0):
        self.pid = pid
        self._left = exits_after_polls
        self.returncode = returncode
        self.polls = 0

    def poll(self):
        self.polls += 1
        if self._left is None:
            return None
        if self._left <= 0:
            return self.returncode
        self._left -= 1
        return None


class _Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, seconds):
        self.t += seconds


def _launcher(tmp_path, process):
    clock = _Clock()
    launched, killed = [], []

    def popen(argv, **kwargs):
        launched.append((argv, kwargs))
        return process

    twin = tl.MatlabBatchTwin(
        str(tmp_path / "r.runconfig.json"), str(tmp_path / "logs"),
        popen=popen, kill_tree=killed.append, sleep=clock.sleep,
        clock=clock, matlab_dir=str(tmp_path / "stk_matlab"))
    return twin, launched, killed, clock


class TestLauncher:
    def test_the_command(self, tmp_path):
        twin, launched, _, _ = _launcher(tmp_path, _Process())
        os.makedirs(tmp_path / "logs")
        (tmp_path / "logs" / tl.STOP_FILE).write_text("stale")
        twin.start()
        argv, kwargs = launched[0]
        stop = str(tmp_path / "logs" / tl.STOP_FILE)
        assert argv == [
            "matlab", "-batch",
            f"run_twin_session('{tmp_path / 'r.runconfig.json'}', '{stop}')",
            "-logfile", str(tmp_path / "logs" / tl.MATLAB_LOG_FILE)]
        assert kwargs["cwd"] == str(tmp_path / "stk_matlab")
        assert not os.path.exists(stop), "a stale stop file is removed"

    def test_a_quote_in_a_path_is_escaped_for_matlab(self, tmp_path):
        config = os.path.join(str(tmp_path), "it's", "r.json")
        twin = tl.MatlabBatchTwin(config, str(tmp_path / "logs"),
                                  popen=lambda *a, **k: None)
        # Absolute and normalised (os.path.abspath), quote doubled.
        assert "'" + config.replace("'", "''") + "'" in twin.argv[2]

    def test_a_stop_file_stop(self, tmp_path):
        twin, _, killed, _ = _launcher(tmp_path, _Process(exits_after_polls=3))
        twin.start()
        result = twin.stop(timeout_s=30.0)
        assert os.path.exists(twin.stop_file)
        assert result.how == "clean_stop" and result.exit_code == 0
        assert killed == []

    def test_a_nonzero_exit_after_the_stop_is_not_clean(self, tmp_path):
        twin, _, _, _ = _launcher(
            tmp_path, _Process(exits_after_polls=1, returncode=1))
        twin.start()
        result = twin.stop(timeout_s=30.0)
        assert result.how == "error_exit" and result.exit_code == 1

    def test_an_early_exit(self, tmp_path):
        twin, _, killed, _ = _launcher(
            tmp_path, _Process(exits_after_polls=0, returncode=1))
        twin.start()
        assert twin.exit_code() == 1
        result = twin.stop(timeout_s=30.0)
        assert result.how == "early_exit" and result.exit_code == 1
        assert killed == []

    def test_a_timeout_kills_only_its_own_tree(self, tmp_path):
        process = _Process(pid=4242)                     # never exits
        twin, _, killed, clock = _launcher(tmp_path, process)
        twin.start()
        result = twin.stop(timeout_s=30.0)
        assert killed == [4242]
        assert result.how == "timeout_killed"
        assert clock.t >= 30.0

    def test_stop_is_idempotent(self, tmp_path):
        twin, _, _, _ = _launcher(tmp_path, _Process(exits_after_polls=0))
        twin.start()
        first = twin.stop(timeout_s=30.0)
        assert twin.stop(timeout_s=30.0) is first

    def test_never_started(self, tmp_path):
        twin, _, _, _ = _launcher(tmp_path, _Process())
        assert twin.stop(timeout_s=1.0).how == "not_started"

    def test_kill_tree_names_only_that_pid(self, monkeypatch):
        calls = []
        monkeypatch.setattr(tl.subprocess, "run",
                            lambda argv, **k: calls.append(argv))
        tl.kill_process_tree(4242)
        assert calls == [["taskkill", "/PID", "4242", "/T", "/F"]]

    def test_the_automated_ack_timeout(self):
        assert tl.AUTOMATED_TWIN_ACK_TIMEOUT_S == max(
            2 * tl.MEASURED_STARTUP_BUILD_S, 180.0)


# -- the runner -------------------------------------------------------------

class _FakeTwin:
    def __init__(self, log_dir, how="clean_stop", exit_code=0,
                 early=None, version="23.2.0.2485118 (R2023b)"):
        self.log_dir = log_dir
        self.argv = ["matlab", "-batch", "run_twin_session(...)"]
        self.how, self.code, self.early = how, exit_code, early
        self.version = version
        self.started = self.stopped = 0

    def start(self):
        self.started += 1

    def exit_code(self):
        return self.early

    def stop(self, timeout_s=None):
        self.stopped += 1
        with open(os.path.join(self.log_dir, tl.MATLAB_SESSION_FILE), "w",
                  encoding="utf-8") as handle:
            json.dump({"matlab_version": self.version,
                       "exit": "clean_stop" if self.how == "clean_stop"
                       else "error"}, handle)
        return tl.SessionResult(self.how, self.code, "")


def _run(tmp_path, twin_ok=True, **twin):
    config = _config(tmp_path, twin_launch="matlab-batch")
    prompts, waits, twins = [], [], []

    def launch(run_config, log_dir):
        twins.append(_FakeTwin(log_dir, **twin))
        return twins[-1]

    def scheduler_or_bridge(argv, **kwargs):
        if "acav.scheduler_node" in argv:
            write_record(config.output, RunRecord(
                result=RunResult(run_id=config.run_id,
                                 scheduler="dynamics_aware"),
                provenance=Provenance(backend_type="com",
                                      execution_mode="lockstep")))
        return FakeProcess()

    deps = Deps(popen=scheduler_or_bridge,
                wait_for_bridge_connected=lambda *a: ConnectResult(True,
                                                                   "com"),
                wait_for_twin_ack=lambda prefix, timeout, *rest: (
                    waits.append(timeout) or twin_ok),
                prompt=prompts.append, launch_twin=launch)
    code = execute_run(config, deps)
    summary = json.load(open(os.path.join(config.resolved_log_dir(),
                                          "summary.json")))
    return code, summary, prompts, waits, twins, config


class TestRunnerAutomated:
    def test_a_clean_automated_run(self, tmp_path):
        code, summary, prompts, waits, twins, config = _run(tmp_path)
        assert code == 0
        assert prompts == []                       # nobody is asked
        assert waits == [tl.AUTOMATED_TWIN_ACK_TIMEOUT_S]
        assert summary["twin_launch"] == "matlab-batch"
        assert summary["twin_ack_timeout_s"] == \
            tl.AUTOMATED_TWIN_ACK_TIMEOUT_S
        assert summary["matlab_session"]["exit"] == "clean_stop"
        assert summary["matlab_session_result"]["how"] == "clean_stop"
        assert summary["matlab_version"] == "23.2.0.2485118 (R2023b)"
        steps = [s["step"] for s in summary["steps"]]
        assert steps.index("twin_launch") < steps.index("twin_acking")
        assert steps.index("scheduler_exit") < steps.index("matlab_session")
        assert twins[0].started == 1 and twins[0].stopped == 1
        assert read_record(config.output).provenance.matlab_version == \
            "23.2.0.2485118 (R2023b)"

    def test_the_identity_check_stays_authoritative(self, tmp_path,
                                                     monkeypatch):
        config = _config(tmp_path, twin_launch="matlab-batch")
        twins = []
        deps = Deps(popen=lambda *a, **k: FakeProcess(),
                    wait_for_bridge_connected=lambda *a: ConnectResult(
                        True, "com"),
                    wait_for_twin_ack=lambda *a: True,
                    confirm_twin_identity=lambda *a: ("v3 x", ["stale"]),
                    prompt=lambda m: None,
                    launch_twin=lambda rc, d: twins.append(_FakeTwin(d))
                    or twins[-1])
        assert execute_run(config, deps) == deploy_run_cli.EXIT_STEP_FAILED
        assert twins[0].stopped == 1                 # still stopped

    def test_no_ack_stops_the_twin_and_fails(self, tmp_path):
        code, summary, _, _, twins, _ = _run(
            tmp_path, twin_ok=False, how="early_exit", exit_code=1, early=1)
        assert code == deploy_run_cli.EXIT_STEP_FAILED
        acking = [s for s in summary["steps"] if s["step"] == "twin_acking"]
        assert not acking[0]["ok"] and "exited" in acking[0]["detail"]
        assert twins[0].stopped == 1
        session = [s for s in summary["steps"]
                   if s["step"] == "matlab_session"]
        assert session and not session[0]["ok"]

    def test_an_unclean_stop_fails_the_run(self, tmp_path):
        code, summary, _, _, _, _ = _run(tmp_path, how="timeout_killed",
                                         exit_code=1)
        assert code == deploy_run_cli.EXIT_STEP_FAILED
        assert summary["matlab_session_result"]["how"] == "timeout_killed"


class TestManualIsUnchanged:
    """Byte for byte what the runner did before --twin-launch existed,
    plus console_quickedit (P9e), errors (the waits' own exceptions) and
    the bridge's startup record (bridge_*) and which STK the run used
    (stk_process, allow_com_started_stk), which every run now records."""

    KEYS = ['allow_com_started_stk',
            'bridge_connect_s', 'bridge_exit_code', 'bridge_last_phase',
            'bridge_phase_durations', 'bridge_startup',
            'code_version', 'console_quickedit', 'diagnostic_label',
            'equivalence_diagnostic', 'errors',
            'execution_mode', 'experiment', 'finished_at', 'graphics_end',
            'graphics_mode', 'graphics_mode_required', 'graphics_samples',
            'graphics_startup', 'orbit_batch_samples', 'output_exists',
            'real_time_factor', 'run_config', 'run_config_run_id_verified',
            'run_id', 'sensing_mode', 'started_at', 'steps', 'stk_process',
            'twin_ack_wait',
            'twin_identity', 'twin_identity_problems']
    STEPS = ['idle_check', 'bridge_connected', 'stk_process', 'stk_graphics',
             'twin_acking', 'twin_identity', 'scheduler_exit',
             'stk_graphics_end', 'topic_capture']

    def test_the_default_is_manual(self):
        assert deploy_run_cli.RunConfig.__dataclass_fields__[
            "twin_launch"].default == "manual"
        parser = deploy_run_cli._parser()
        args = ["--run-id", "p", "--output", "p.json",
                "--bridge-config", "b.yaml"]
        assert parser.parse_args(args).twin_launch == "manual"
        assert parser.parse_args(
            args + ["--twin-launch", "matlab-batch"]).twin_launch == \
            "matlab-batch"
        with pytest.raises(SystemExit):
            parser.parse_args(args + ["--twin-launch", "engine"])

    def test_prompts_steps_and_summary_keys(self, tmp_path):
        prompts, waits = [], []

        def no_launch(*a):
            raise AssertionError("manual mode must not launch MATLAB")

        deps = Deps(popen=lambda *a, **k: FakeProcess(),
                    wait_for_bridge_connected=lambda *a: ConnectResult(
                        True, "com"),
                    wait_for_twin_ack=lambda prefix, timeout, *rest: (
                        waits.append(timeout) or True),
                    prompt=prompts.append, launch_twin=no_launch)
        config = _config(tmp_path)
        assert execute_run(config, deps) == 0
        summary = json.load(open(tmp_path / "run.logs" / "summary.json"))
        assert sorted(summary) == self.KEYS
        assert [s["step"] for s in summary["steps"]] == self.STEPS
        assert waits == [config.twin_ack_timeout_s]
        run_config = summary["run_config"]
        assert prompts == [
            "\n=== Start the MATLAB/Simulink twin now ===\n"
            "  matlab -batch \"cd('src/stk_matlab'); setup_stk_ros; "
            f"build_acav_twin('RunConfigPath', '{run_config}')\"\n"
            "then open the generated acav_twin.slx and press Play.\n"
            "Press Enter here once it is running (this script will then "
            "WAIT for its barrier ack, not just trust the keypress): ",
            "\n=== Scheduler finished. Stop the Simulink twin now (the "
            "model has no automatic stop) and press Enter to continue: "]


def test_matlab_version_in_provenance_round_trips(tmp_path):
    record = RunRecord(result=RunResult(run_id="r", scheduler="x"),
                       provenance=Provenance(matlab_version="23.2"))
    write_record(str(tmp_path / "r.json"), record)
    assert read_record(str(tmp_path / "r.json")).provenance.matlab_version \
        == "23.2"
    old = record.to_dict()
    old["provenance"].pop("matlab_version")
    assert Provenance.from_dict(old["provenance"]).matlab_version == ""
