"""The runner's bridge-connect wait: bounded, fail-fast, and recorded.

A free-run run can fail bridge_connected with "no BridgeStatus received
at all" while the bridge is still inside convention validation. Launch to
CONNECTED took 43.7-76.1 s over 40 runs, so:
  * the bound is 180 s (about 2.4x the slowest of those 40);
  * the wait ends at once if the bridge process exits, with its code;
  * the bridge writes bridge_startup_phases.json into the run's logs
    folder (stk_bridge core/startup_phases.py), and summary.json carries
    bridge_connect_s, bridge_last_phase, bridge_phase_durations,
    bridge_startup (the whole file) and bridge_exit_code;
  * a failed wait's detail names the phase the bridge was in.
"""

import json
import os
from types import SimpleNamespace

import pytest

import acav.deploy_run_cli as deploy_run_cli
from acav.deploy_run_cli import (
    BRIDGE_CONNECT_TIMEOUT_S,
    STARTUP_PHASES_FILE,
    ConnectResult,
    Deps,
    RunConfig,
    _parser,
    execute_run,
    write_bridge_config,
)

from test_deploy_run_cli import FakeProcess, _config  # noqa: I100
from test_ros_wait_contexts import FakeRos  # noqa: I100

PHASES = {
    "started_at": "2026-10-17T09:42:30.000", "finished": False,
    "outcome": "", "error": "",
    "current": "validation/quaternion_ordering",
    "current_since": "2026-10-17T09:43:57.500",
    "last_phase": "validation/quaternion_ordering",
    "phases": [
        {"phase": "connect", "started_at": "a", "ended_at": "b",
         "s": 1.5, "ok": True},
        {"phase": "frames", "started_at": "c", "ended_at": "d",
         "s": 21.0, "ok": True},
    ],
}


def test_the_bound_is_180_s_by_default():
    assert BRIDGE_CONNECT_TIMEOUT_S == 180.0
    import dataclasses

    fields = {f.name: f for f in dataclasses.fields(RunConfig)}
    assert fields["bridge_connect_timeout_s"].default == 180.0
    args = _parser().parse_args(["--run-id", "r", "--output", "o.json",
                                 "--bridge-config", "b.yaml"])
    assert args.bridge_connect_timeout_s == 180.0


def test_the_bridge_is_told_where_to_write_its_phases(tmp_path):
    import logging

    config = _config(tmp_path)
    (tmp_path / "logs").mkdir()
    path = write_bridge_config(config, str(tmp_path / "logs"),
                               logging.getLogger("t"))
    import yaml
    settings = yaml.safe_load(open(path, encoding="utf-8"))
    assert settings["startup_phases_file"] == os.path.join(
        os.path.abspath(str(tmp_path / "logs")), STARTUP_PHASES_FILE)
    assert STARTUP_PHASES_FILE == "bridge_startup_phases.json"


class ExitedProcess(FakeProcess):
    def __init__(self, code):
        super().__init__(returncode=code)
        self.code = code

    def poll(self):
        return self.code


def _deps(**overrides):
    recorder = SimpleNamespace(close=lambda: None, error=None)
    values = dict(check_idle=lambda prefix: None,
                  start_recorder=lambda *a: recorder,
                  popen=lambda *a, **k: FakeProcess(),
                  wait_for_bridge_connected=lambda *a: ConnectResult(True,
                                                                     "com"),
                  wait_for_twin_ack=lambda *a: True,
                  prompt=lambda message: None)
    values.update(overrides)
    return Deps(**values)


def _summary(tmp_path):
    return json.loads((tmp_path / "run.logs" / "summary.json")
                      .read_text(encoding="utf-8"))


def _write_phases(tmp_path, phases):
    path = tmp_path / "run.logs" / STARTUP_PHASES_FILE
    path.write_text(json.dumps(phases), encoding="utf-8")


def test_a_connected_run_records_its_startup(tmp_path):
    seen = {}

    def connected(prefix, timeout_s, domain, exit_code):
        seen["timeout_s"] = timeout_s
        seen["exit_code"] = exit_code()
        done = dict(PHASES, finished=True, outcome="connected", current="",
                    current_since="", last_phase="interfaces")
        _write_phases(tmp_path, done)
        return ConnectResult(True, "com")

    config = _config(tmp_path, bridge_connect_timeout_s=180.0)
    assert execute_run(config, _deps(
        wait_for_bridge_connected=connected)) == 0
    assert seen == {"timeout_s": 180.0, "exit_code": None}
    summary = _summary(tmp_path)
    assert summary["bridge_connect_s"] >= 0.0
    assert summary["bridge_last_phase"] == "interfaces"
    assert summary["bridge_phase_durations"] == {"connect": 1.5,
                                                 "frames": 21.0}
    assert summary["bridge_startup"]["outcome"] == "connected"
    assert summary["bridge_exit_code"] is None


def test_a_timed_out_wait_names_the_phase_the_bridge_was_in(tmp_path):
    def stalled(prefix, timeout_s, domain, exit_code):
        _write_phases(tmp_path, PHASES)
        return ConnectResult(False, "", "no BridgeStatus received at all")

    assert execute_run(_config(tmp_path), _deps(
        wait_for_bridge_connected=stalled)) == 1
    summary = _summary(tmp_path)
    step = [s for s in summary["steps"]
            if s["step"] == "bridge_connected"][0]
    assert step["ok"] is False
    assert "no BridgeStatus received at all" in step["detail"]
    assert ("bridge startup phase: validation/quaternion_ordering since "
            "2026-10-17T09:43:57.500") in step["detail"]
    assert summary["bridge_connect_s"] is None
    assert summary["bridge_last_phase"] == "validation/quaternion_ordering"
    assert summary["bridge_exit_code"] is None


def test_a_bridge_that_exited_is_recorded_with_its_code(tmp_path):
    def exited(prefix, timeout_s, domain, exit_code):
        return ConnectResult(False, "", f"the bridge exited (code "
                                        f"{exit_code()}) before it "
                                        f"reported CONNECTED")

    assert execute_run(_config(tmp_path), _deps(
        popen=lambda *a, **k: ExitedProcess(3),
        wait_for_bridge_connected=exited)) == 1
    summary = _summary(tmp_path)
    assert summary["bridge_exit_code"] == 3
    assert summary["bridge_startup"] is None
    step = [s for s in summary["steps"]
            if s["step"] == "bridge_connected"][0]
    assert "exited (code 3)" in step["detail"]
    assert "no bridge_startup_phases.json" in step["detail"]


# --- the ROS wait itself -----------------------------------------------------

def test_the_ros_wait_ends_as_soon_as_the_bridge_exits(monkeypatch):
    fake = FakeRos(monkeypatch)
    codes = iter([None, None, 2])
    result = deploy_run_cli._wait_for_bridge_connected_ros(
        "/stk", 180.0, "0", lambda: next(codes))
    assert not result.connected
    assert result.detail == ("the bridge exited (code 2) before it "
                             "reported CONNECTED")
    assert fake.now < 5.0


def test_the_ros_wait_names_starting_on_timeout(monkeypatch):
    def starting(fake):
        fake.deliver("/stk/status", fake.BridgeStatus(state=0))

    FakeRos(monkeypatch, on_spin=starting)
    result = deploy_run_cli._wait_for_bridge_connected_ros(
        "/stk", 3.0, "0", lambda: None)
    assert not result.connected
    assert result.detail == "timed out; last state=0 (STARTING)"


def test_the_ros_wait_still_works_without_an_exit_check(monkeypatch):
    def connect(fake):
        fake.deliver("/stk/status", fake.BridgeStatus(state=1))

    FakeRos(monkeypatch, on_spin=connect)
    assert deploy_run_cli._wait_for_bridge_connected_ros(
        "/stk", 3.0, "0").connected


@pytest.mark.parametrize("phases, expected", [
    (None, None),
    ({"phases": [], "last_phase": "", "current": ""}, None),
])
def test_an_empty_or_missing_phases_file_gives_no_last_phase(
        phases, expected):
    assert deploy_run_cli._bridge_startup_fields(phases)[
        "bridge_last_phase"] == expected
