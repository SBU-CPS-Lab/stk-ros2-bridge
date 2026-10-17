"""Before MATLAB starts: leftover twin processes are recorded, and a
leftover ROS 2 server is waited out or refused.

In the live session on d0bf796 an E1 run's twin could not create its
ROS 2 node ("Server failed to start. Details: Transport stopped.",
results\\LIVE_E6_d0bf796, the quaternion_order attempt). Its cause is not
established. Before launching MATLAB (--twin-launch matlab-batch) the
runner now records every running matlab.exe and libmwros2server.exe in
summary.json (step twin_leftovers); if a libmwros2server.exe is still
running it waits up to 60 s for it to end, then refuses the run with a
clear message. A matlab.exe alone is recorded, not refused: it may be
the operator's own session.
"""

import json
import os

import pytest

from acav.deploy_run_cli import ConnectResult, Deps, execute_run
from acav.metrics import RunResult
from acav.run_record import Provenance, RunRecord, write_record

from test_deploy_run_cli import FakeProcess, _config  # noqa: I100
from test_twin_ack_exit import _Twin  # noqa: I100



def _run(tmp_path, polls, twin_launch="matlab-batch"):
    """polls: what each call to the process finder returns, in order
    (the last repeats)."""
    config = _config(tmp_path, twin_launch=twin_launch)
    calls, slept, launched = [], [], []

    def finder():
        calls.append(1)
        return polls[min(len(calls), len(polls)) - 1]

    def scheduler_or_bridge(argv, **kwargs):
        if "acav.scheduler_node" in argv:
            write_record(config.output, RunRecord(
                result=RunResult(run_id=config.run_id,
                                 scheduler="dynamics_aware"),
                provenance=Provenance(backend_type="com",
                                      execution_mode="lockstep")))
        return FakeProcess()

    def launch(run_config, log_dir):
        launched.append(log_dir)
        return _Twin(log_dir, None)

    from types import SimpleNamespace
    recorder = SimpleNamespace(close=lambda: None, error=None)
    deps = Deps(popen=scheduler_or_bridge, check_idle=lambda prefix: None,
                start_recorder=lambda *a: recorder,
                wait_for_bridge_connected=lambda *a: ConnectResult(True,
                                                                   "com"),
                wait_for_twin_ack=lambda *a: True,
                prompt=lambda message: None, launch_twin=launch,
                find_twin_processes=finder, sleep=slept.append)
    code = execute_run(config, deps)
    summary = json.load(open(os.path.join(config.resolved_log_dir(),
                                          "summary.json"), encoding="utf-8"))
    return code, summary, calls, slept, launched


def _step(summary):
    found = [s for s in summary["steps"] if s["step"] == "twin_leftovers"]
    return found[0] if found else None


def test_nothing_left_is_recorded_and_the_run_goes_on(tmp_path):
    code, summary, _, slept, launched = _run(tmp_path, [[]])
    assert code == 0 and launched and slept == []
    assert summary["leftover_twin_processes"] == []
    assert _step(summary)["ok"] is True


def test_a_leftover_matlab_is_recorded_not_refused(tmp_path):
    code, summary, _, slept, launched = _run(tmp_path,
                                             [[(5100, "MATLAB.exe")]])
    assert code == 0 and launched and slept == []
    assert summary["leftover_twin_processes"] == [[5100, "MATLAB.exe"]]


def test_a_ros_server_that_ends_is_waited_for(tmp_path):
    server = [(7200, "libmwros2server.exe")]
    code, summary, calls, slept, launched = _run(tmp_path,
                                                 [server, server, []])
    assert code == 0 and launched
    assert summary["leftover_twin_processes"] == [[7200,
                                                   "libmwros2server.exe"]]
    assert sum(slept) == pytest.approx(4.0)
    assert "ended after" in _step(summary)["detail"]


def test_a_ros_server_that_stays_refuses_the_run(tmp_path):
    server = [(7200, "libmwros2server.exe")]
    code, summary, _, slept, launched = _run(tmp_path, [server])
    assert code == 1
    assert launched == []
    assert sum(slept) == pytest.approx(60.0)
    step = _step(summary)
    assert step["ok"] is False
    assert "libmwros2server.exe (pid 7200) still running after 60 s" in \
        step["detail"]


def test_a_manual_twin_is_not_checked(tmp_path):
    code, summary, calls, _, _ = _run(tmp_path, [[(1, "matlab.exe")]],
                                      twin_launch="manual")
    assert calls == [] and _step(summary) is None


def test_the_default_finder_query_has_no_double_quotes():
    import inspect

    from acav import twin_launcher
    source = inspect.getsource(twin_launcher.twin_processes_running)
    assert "matlab.exe" in source and "libmwros2server.exe" in source
    script = twin_launcher.TWIN_PROCESS_QUERY
    assert '"' not in script
