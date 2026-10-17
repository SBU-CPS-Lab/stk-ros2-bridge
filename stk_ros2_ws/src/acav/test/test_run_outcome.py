"""Honest outcomes for a run that left no record.

Live session on d0bf796 (results\\LIVE_E6_d0bf796) showed three ways an
E1 guard-on run can end without a record, only one of which is the
guard detecting the mismatch:

  interrupted              KeyboardInterrupt in summary.json errors: no
                           result; never a pass, never a fail-stop
  infrastructure_failure   the twin's ROS node/server did not start
                           ("Server failed to start", the simulation
                           never "running": waitFor "not reached"): not
                           a detection; to be retried
  guard_fail_stop          the twin stopped on its own guard
                           (adcs:simulink_step:AttitudeMismatch), with
                           the reason printed
  fail_stop                anything else that ended by itself

The fixtures are minimal copies of fields from the three real
summary.json files (sources named on each); results are not read here.
"""

import json

import pytest

from acav.campaign import build_e1_matrix
from acav.campaign_cli import main

from test_e1_mismatch import _write  # noqa: I100


# E1-...-mismatchTimeOffset.logs.superseded-20261017T192525/summary.json
INTERRUPTED = {
    "steps": [{"step": "stk_graphics", "ok": True, "detail": ""},
              {"step": "twin_launch", "ok": True, "detail": "matlab -batch"},
              {"step": "matlab_session", "ok": False,
               "detail": "error_exit, exit code 1"}],
    "errors": [{"step": "run", "type": "KeyboardInterrupt", "message": "",
                "traceback": "...\nKeyboardInterrupt\n"}],
    "matlab_session": {"running_at": "", "exit": "", "error": "",
                       "simulink_diagnostics": ""},
}
# E1-...-mismatchQuaternionOrder.logs/summary.json (latest attempt)
INFRASTRUCTURE = {
    "steps": [{"step": "bridge_connected", "ok": True, "detail": "com"},
              {"step": "twin_acking", "ok": False, "detail": "no ack"},
              {"step": "matlab_session", "ok": False,
               "detail": "error_exit, exit code 1"}],
    "errors": [],
    "twin_ack_wait": {"stopped": "timeout", "exit_code": None,
                      "waited_s": 253.2, "timeout_s": 252.6},
    "matlab_session": {
        "running_at": "", "exit": "error",
        "error": "Error using run_twin_session>waitFor\nnot reached within "
                 "120 s\n\nError in run_twin_session (line 51)",
        "simulink_diagnostics":
            "Error: MATLAB System block 'acav_twin/AttitudeSub/SourceBlock' "
            "error occurred when invoking 'setupImpl' method ...\r\n"
            "Caused by:\r\n    Error creating the ROS 2 node "
            "acav_twin_81473.\r\n        Server failed to start. Details: "
            "Transport stopped."},
}
# E1-...-mismatchQuaternionOrder.logs.superseded-20261017T191957/summary.json
GUARD = {
    "steps": [{"step": "twin_identity", "ok": True, "detail": "v3"},
              {"step": "bridge_alive", "ok": False, "detail": "exited"},
              {"step": "matlab_session", "ok": False,
               "detail": "error_exit, exit code 1"}],
    "errors": [],
    "twin_ack_wait": {"stopped": "acked", "exit_code": None,
                      "waited_s": 108.8, "timeout_s": 252.6},
    "matlab_session": {
        "running_at": "2026-10-17T18:40:26.608", "exit": "error",
        "error": "Error using run_twin_session\nthe simulation stopped by "
                 "itself (status stopped): Error: STK attitude does not "
                 "match the previous achieved command (quaternion "
                 "residual\n1.41421). Check timestamps and reference "
                 "frames.\r\nError in simulink_step.m (line 239)\r\n"
                 "            error('adcs:simulink_step:AttitudeMismatch', "
                 "...",
        "simulink_diagnostics":
            "Error: STK attitude does not match the previous achieved "
            "command (quaternion residual 1.41421). Check timestamps and "
            "reference frames.\r\nError in simulink_step.m (line 239)\r\n"
            "            error('adcs:simulink_step:AttitudeMismatch', ..."},
}
# E1-...-mismatchTimeOffset.logs/summary.json (latest attempt)
ENDED_BY_ITSELF = {
    "steps": [{"step": "bridge_connected", "ok": True, "detail": "com"},
              {"step": "twin_acking", "ok": False, "detail": "no ack"},
              {"step": "matlab_session", "ok": True,
               "detail": "clean_stop, exit code 0"}],
    "errors": [],
    "twin_ack_wait": {"stopped": "timeout", "exit_code": None,
                      "waited_s": 252.7, "timeout_s": 252.6},
    "matlab_session": {"running_at": "2026-10-17T19:28:01.456",
                       "exit": "clean_stop", "error": "",
                       "simulink_diagnostics": ""},
}


@pytest.mark.parametrize("summary, kind", [
    (INTERRUPTED, "interrupted"),
    (INFRASTRUCTURE, "infrastructure_failure"),
    (GUARD, "guard_fail_stop"),
    (ENDED_BY_ITSELF, "fail_stop"),
])
def test_classify(summary, kind):
    from acav.run_outcome import classify
    assert classify(summary)["kind"] == kind


def test_the_guard_reason_is_the_twins_own():
    from acav.run_outcome import classify
    reason = classify(GUARD)["reason"]
    assert reason.startswith("adcs:simulink_step:AttitudeMismatch: STK "
                             "attitude does not match the previous achieved "
                             "command (quaternion residual 1.41421)")


def test_the_infrastructure_reason_names_the_server():
    from acav.run_outcome import classify
    assert "Server failed to start" in classify(INFRASTRUCTURE)["reason"]


def _folder(tmp_path, quaternion, offset):
    control, q_run, t_run = build_e1_matrix(
        ("quaternion_order", "time_offset"), 1).runs
    _write(tmp_path, control)
    for run, summary in ((q_run, quaternion), (t_run, offset)):
        logs = tmp_path / f"{run.run_id}.logs"
        logs.mkdir()
        (logs / "summary.json").write_text(json.dumps(summary),
                                           encoding="utf-8")
    return control, q_run, t_run


def _report(tmp_path, capsys):
    code = main(["report", "--experiment", "E1", "--e1-types",
                 "quaternion_order,time_offset", "--no-figures",
                 "--output-dir", str(tmp_path)])
    out = capsys.readouterr().out
    return code, json.loads(out[:out.rindex("}") + 1])


def test_report_lists_an_infrastructure_failure_apart(tmp_path, capsys):
    _, q_run, _ = _folder(tmp_path, INFRASTRUCTURE, ENDED_BY_ITSELF)
    code, report = _report(tmp_path, capsys)
    arm = report["arms"]["quaternion_order/on"]
    assert arm["fail_stops"] == []
    failure, = arm["infrastructure_failures"]
    assert failure["run_id"] == q_run.run_id
    assert "Server failed to start" in failure["reason"]
    assert report["runs_to_retry"] == [q_run.run_id]
    assert code == 1


def test_report_names_the_guard_reason(tmp_path, capsys):
    _folder(tmp_path, GUARD, ENDED_BY_ITSELF)
    code, report = _report(tmp_path, capsys)
    stop, = report["arms"]["quaternion_order/on"]["fail_stops"]
    assert stop["outcome"] == "guard_fail_stop"
    assert "AttitudeMismatch" in stop["reason"]
    offset, = report["arms"]["time_offset/on"]["fail_stops"]
    assert offset["cause"].startswith(("confirmed",
                                       "timeout; cause not independently"))
    assert code == 0


def test_an_interrupted_run_is_no_result(tmp_path, capsys):
    _, _, t_run = _folder(tmp_path, GUARD, INTERRUPTED)
    code, report = _report(tmp_path, capsys)
    arm = report["arms"]["time_offset/on"]
    assert arm["fail_stops"] == []
    assert arm["interrupted"] == [t_run.run_id]
    assert "cause" not in json.dumps(arm)
    assert t_run.run_id in report["runs_to_retry"]
    assert code == 1
