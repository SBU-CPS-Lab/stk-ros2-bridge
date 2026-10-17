"""The twin-ack wait stops as soon as MATLAB has exited.

The d97800c incident (results/_archive/incidents/E2_P10_seed1_d97800c):
MATLAB exited with an error about 112 s after launch, and the runner
went on waiting for an ack until its 252.6 s timeout. The bridge wait
already stops when the bridge exits (3d143bc); the twin-ack wait now
does the same with the automated twin's exit code, records why it
stopped (summary.json "twin_ack_wait") and, when MATLAB exited, the exit
code in summary.json "errors". The timeout is unchanged otherwise.
"""

import json
import os
import types

from acav import twin_launcher as tl
from acav.deploy_run_cli import ConnectResult, Deps, execute_run
from acav.metrics import RunResult
from acav.run_record import Provenance, RunRecord, write_record
import acav.deploy_run_cli as deploy_run_cli

from test_deploy_run_cli import FakeProcess, _config  # noqa: I100
from test_ros_wait_contexts import FakeRos  # noqa: I100

# --- the ROS wait ------------------------------------------------------------

def test_the_ros_wait_ends_as_soon_as_matlab_exits(monkeypatch):
    fake = FakeRos(monkeypatch)
    codes = iter([None, None, 1])
    assert deploy_run_cli._wait_for_twin_ack_ros(
        "/stk", 252.6, lambda: next(codes)) is False
    assert fake.now < 5.0


def test_the_ros_wait_still_sees_an_ack_while_matlab_runs(monkeypatch):
    def ack(fake):
        fake.deliver("/stk/scenario_time", types.SimpleNamespace(step_index=3))
        fake.deliver("/stk/lockstep/ack", types.SimpleNamespace(
            data="matlab_twin:3"))

    FakeRos(monkeypatch, on_spin=ack)
    assert deploy_run_cli._wait_for_twin_ack_ros("/stk", 252.6,
                                                 lambda: None) is True


def test_the_ros_wait_without_an_exit_check_times_out_as_before(monkeypatch):
    fake = FakeRos(monkeypatch)
    assert deploy_run_cli._wait_for_twin_ack_ros("/stk", 3.0) is False
    assert fake.now >= 3.0


# --- the runner --------------------------------------------------------------

class _Twin:
    def __init__(self, log_dir, early):
        self.log_dir = log_dir
        self.argv = ["matlab", "-batch", "run_twin_session(...)"]
        self.early = early

    def start(self):
        pass

    def exit_code(self):
        return self.early

    def stop(self, timeout_s=None):
        with open(os.path.join(self.log_dir, tl.MATLAB_SESSION_FILE), "w",
                  encoding="utf-8") as handle:
            json.dump({"matlab_version": "23.2", "exit":
                       "error" if self.early else "clean_stop"}, handle)
        return tl.SessionResult("early_exit" if self.early else "clean_stop",
                                self.early or 0, "")


def _run(tmp_path, early, acks, twin_launch="matlab-batch"):
    config = _config(tmp_path, twin_launch=twin_launch)
    seen = {}

    def scheduler_or_bridge(argv, **kwargs):
        if "acav.scheduler_node" in argv:
            write_record(config.output, RunRecord(
                result=RunResult(run_id=config.run_id,
                                 scheduler="dynamics_aware"),
                provenance=Provenance(backend_type="com",
                                      execution_mode="lockstep")))
        return FakeProcess()

    def wait(prefix, timeout_s, twin_exit_code=None):
        seen["exit_check"] = twin_exit_code
        if twin_exit_code is not None and twin_exit_code() is not None:
            return False
        return acks

    recorder = types.SimpleNamespace(close=lambda: None, error=None)
    deps = Deps(popen=scheduler_or_bridge,
                check_idle=lambda prefix: None,
                start_recorder=lambda *a: recorder,
                wait_for_bridge_connected=lambda *a: ConnectResult(True,
                                                                   "com"),
                wait_for_twin_ack=wait, prompt=lambda message: None,
                launch_twin=lambda run_config, log_dir: _Twin(log_dir, early))
    code = execute_run(config, deps)
    summary = json.load(open(os.path.join(config.resolved_log_dir(),
                                          "summary.json"), encoding="utf-8"))
    return code, summary, seen


def _step(summary, name):
    return [s for s in summary["steps"] if s["step"] == name][0]


def test_matlab_exiting_ends_the_wait_and_is_recorded(tmp_path):
    code, summary, seen = _run(tmp_path, early=1, acks=True)
    assert code == 1
    assert seen["exit_check"] is not None
    step = _step(summary, "twin_acking")
    assert step["ok"] is False
    assert "MATLAB exited (code 1) before the twin acked" in step["detail"]
    wait = summary["twin_ack_wait"]
    assert wait["stopped"] == "matlab_exited"
    assert wait["exit_code"] == 1
    assert wait["timeout_s"] == tl.AUTOMATED_TWIN_ACK_TIMEOUT_S
    assert wait["waited_s"] >= 0.0
    error, = summary["errors"]
    assert error["step"] == "twin_acking"
    assert error["type"] == "MatlabExited"
    assert error["exit_code"] == 1
    assert "exited (code 1)" in error["message"]


def test_a_running_twin_that_acks_is_unchanged(tmp_path):
    code, summary, seen = _run(tmp_path, early=None, acks=True)
    assert code == 0
    assert summary["twin_ack_wait"]["stopped"] == "acked"
    assert summary["twin_ack_wait"]["exit_code"] is None
    assert summary["errors"] == []


def test_a_running_twin_that_never_acks_times_out_as_before(tmp_path):
    code, summary, seen = _run(tmp_path, early=None, acks=False)
    assert code == 1
    assert summary["twin_ack_wait"]["stopped"] == "timeout"
    assert summary["errors"] == []


def test_a_manual_twin_has_no_exit_check(tmp_path):
    code, summary, seen = _run(tmp_path, early=None, acks=True,
                               twin_launch="manual")
    assert code == 0
    assert seen["exit_check"] is None
    assert summary["twin_ack_wait"]["stopped"] == "acked"
