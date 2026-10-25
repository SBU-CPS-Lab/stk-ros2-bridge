"""A time_offset fail-stop is "confirmed" only on the run's own evidence.

A slow MATLAB start also ends the twin-ack wait in a timeout
(one run needed 152.2 s to ack, 100 s inside the 252.6 s bound).
So the E1 report labels a time_offset fail-stop "confirmed" only when
what the run already recorded shows the twin was alive and never became
ready, and the control arm acked within the bound:

  * summary.json twin_ack_wait: stopped "timeout", no exit code (MATLAB
    was alive at the timeout);
  * summary.json matlab_session.running_at set (the simulation started);
  * the bridge_connected step passed;
  * topics_summary.json: 0 /acav/twin_status and 0 /stk/cmd/attitude
    messages while the bridge published attitude samples (the twin
    publishes only once its telemetry is valid);
  * the control arm's summary.json twin_ack_wait.waited_s, below the
    bound.

Otherwise: "timeout; cause not independently confirmed (control arm
acked in X s)", with what was missing. Records and summaries here are
synthetic test fixtures.
"""

import json

import pytest

from acav.campaign import build_e1_matrix
from acav.campaign_cli import main

from test_e1_mismatch import _write  # noqa: I100

TYPES = ("time_offset",)


def _summary(tmp_path, run_id, **fields):
    logs = tmp_path / f"{run_id}.logs"
    logs.mkdir(exist_ok=True)
    (logs / "summary.json").write_text(json.dumps(dict(
        {"run_id": run_id}, **fields)), encoding="utf-8")
    return logs


def _setup(tmp_path, *, control_ack=108.0, stopped="timeout", exit_code=None,
           running_at="2026-10-17T10:00:00.000", bridge_ok=True,
           twin_status=0, commands=0, attitude=2500, topics=True):
    control, offset = build_e1_matrix(TYPES, 1).runs
    _write(tmp_path, control)
    if control_ack is not None:
        _summary(tmp_path, control.run_id, twin_ack_wait={
            "stopped": "acked", "exit_code": None,
            "waited_s": control_ack, "timeout_s": 252.6})
    else:
        _summary(tmp_path, control.run_id)
    logs = _summary(
        tmp_path, offset.run_id,
        steps=[{"step": "bridge_connected", "ok": bridge_ok, "detail": ""},
               {"step": "twin_acking", "ok": False, "detail": "no ack"}],
        twin_ack_wait={"stopped": stopped, "exit_code": exit_code,
                       "waited_s": 252.6, "timeout_s": 252.6},
        matlab_session={"running_at": running_at})
    if topics:
        (logs / "topics_summary.json").write_text(json.dumps({
            "received_messages": {"/stk/status": 300,
                                  "/stk/cmd/attitude": commands,
                                  "/acav/twin_status": twin_status,
                                  "/stk/sss_1/attitude": attitude,
                                  "/stk/sss_1/angular_rate": attitude},
            "error": None}), encoding="utf-8")
    return offset


def _cause(tmp_path, capsys):
    main(["report", "--experiment", "E1", "--e1-types", "time_offset",
          "--no-figures", "--output-dir", str(tmp_path)])
    out = capsys.readouterr().out
    report = json.loads(out[:out.rindex("}") + 1])
    stop, = report["arms"]["time_offset/on"]["fail_stops"]
    return stop["cause"]


def test_confirmed_on_the_runs_own_evidence(tmp_path, capsys):
    _setup(tmp_path)
    assert _cause(tmp_path, capsys) == (
        "confirmed: MATLAB alive at the 252.6 s timeout and its simulation "
        "running, 0 twin statuses and 0 attitude commands while the bridge "
        "published 2500 attitude samples; control arm acked in 108.0 s")


@pytest.mark.parametrize("change, missing", [
    ({"stopped": "matlab_exited", "exit_code": 1}, "MATLAB alive at timeout"),
    ({"running_at": ""}, "simulation running"),
    ({"bridge_ok": False}, "bridge connected"),
    ({"twin_status": 3}, "no twin status"),
    ({"commands": 2}, "no twin command"),
    ({"attitude": 0}, "bridge published attitude"),
    ({"topics": False}, "topics_summary.json"),
])
def test_anything_missing_leaves_the_cause_unconfirmed(tmp_path, capsys,
                                                       change, missing):
    _setup(tmp_path, **change)
    cause = _cause(tmp_path, capsys)
    assert cause.startswith("timeout; cause not independently confirmed "
                            "(control arm acked in 108.0 s)")
    assert missing in cause


def test_without_a_control_ack_time_it_says_so(tmp_path, capsys):
    _setup(tmp_path, control_ack=None)
    assert _cause(tmp_path, capsys).startswith(
        "timeout; cause not independently confirmed (control arm ack time "
        "not recorded)")


def test_a_control_slower_than_the_bound_does_not_confirm(tmp_path, capsys):
    _setup(tmp_path, control_ack=260.0)
    cause = _cause(tmp_path, capsys)
    assert cause.startswith("timeout; cause not independently confirmed")
    assert "control acked within the bound" in cause
