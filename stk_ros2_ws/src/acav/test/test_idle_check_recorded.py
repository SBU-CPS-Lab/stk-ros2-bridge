"""An idle-check refusal is a recorded, failed step -- not just a raise.

On 2026-10-15 (E6-nc3-stale-model) the check refused a run because the
previous Simulink twin was still publishing, but run.log ended after the
sensing-mode line and summary.json had no steps: the reason was only in
the console. It is now logged as `idle_check: <reason>` at ERROR and
recorded as a failed step, and the run returns 1 without launching.
"""

import json
from unittest.mock import Mock

from acav.deploy_run_cli import Deps, execute_run

from test_deploy_run_cli import _config  # noqa: I100 - shared helper

REASON = ("Another run is active on /stk/lockstep/ack; stop its bridge, "
          "scheduler and twin first")


def _busy(prefix):
    raise RuntimeError(REASON)


def _run(tmp_path, check_idle):
    popen = Mock()
    code = execute_run(_config(tmp_path),
                       Deps(check_idle=check_idle, popen=popen))
    logs = tmp_path / "run.logs"
    summary = json.loads((logs / "summary.json").read_text(encoding="utf-8"))
    log = (logs / "run.log").read_text(encoding="utf-8")
    return code, popen, summary, log


def test_a_busy_graph_is_logged_and_recorded(tmp_path):
    code, popen, summary, log = _run(tmp_path, _busy)
    assert code == 1
    popen.assert_not_called()
    assert summary["steps"] == [
        {"step": "idle_check", "ok": False, "detail": REASON}]
    assert f"ERROR idle_check: {REASON}" in log


def test_a_failing_check_itself_is_recorded_too(tmp_path):
    def broken(prefix):
        raise OSError("rclpy could not start")

    code, popen, summary, log = _run(tmp_path, broken)
    assert code == 1
    popen.assert_not_called()
    assert summary["steps"][0]["step"] == "idle_check"
    assert "rclpy could not start" in summary["steps"][0]["detail"]


def test_an_idle_graph_is_recorded_as_a_passed_step(tmp_path):
    from acav.deploy_run_cli import ConnectResult

    from test_deploy_run_cli import FakeProcess  # noqa: I100

    from types import SimpleNamespace

    recorder = SimpleNamespace(close=lambda: None, error=None)
    deps = Deps(check_idle=lambda prefix: None,
                start_recorder=lambda *a: recorder,
                popen=lambda *a, **k: FakeProcess(),
                wait_for_bridge_connected=lambda *a: ConnectResult(True, "com"),
                wait_for_twin_ack=lambda *a: True, prompt=lambda m: None)
    assert execute_run(_config(tmp_path), deps) == 0
    summary = json.loads((tmp_path / "run.logs" / "summary.json")
                         .read_text(encoding="utf-8"))
    assert summary["steps"][0] == {"step": "idle_check", "ok": True,
                                   "detail": ""}
