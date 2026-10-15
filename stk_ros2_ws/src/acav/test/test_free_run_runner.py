"""A free-run run's twin sub-step statistics reach summary.json.

The twin (adcs.simulink_step, execution_mode free_run) writes n > 1
occurrences, the largest n and the most negative command-stamp offset
to <run>.logs/twin_substeps.json whenever they change. After the
scheduler exits, the runner copies them into summary.json as
"twin_substeps" for a free-run run; null means no such event was
recorded. A lockstep run does not carry the key.
"""

import json

import pytest

import acav.deploy_run_cli as deploy_run_cli
from acav.deploy_run_cli import ConnectResult, Deps, execute_run

from test_deploy_run_cli import FakeProcess, _config  # noqa: I100

STATS = {"samples": 812, "steps": 7200, "events": 403, "max_n": 3,
         "min_stamp_offset_s": -0.0994}


@pytest.fixture(autouse=True)
def offline(monkeypatch):
    from types import SimpleNamespace
    import acav.topic_recorder
    monkeypatch.setattr(deploy_run_cli, "_check_graph_idle", lambda p: None)
    monkeypatch.setattr(acav.topic_recorder, "TopicRecorder",
                        lambda *a: SimpleNamespace(close=lambda: None,
                                                   error=None))


def _run(tmp_path, mode, stats=None):
    logs = tmp_path / "run.logs"

    def popen(argv, **kwargs):
        if "acav.scheduler_node" in argv:
            # The bridge's start gate opened (see the tests below).
            (logs / "startup_gate.json").write_text(json.dumps(GATE),
                                                    encoding="utf-8")
            if stats is not None:
                (logs / "twin_substeps.json").write_text(
                    json.dumps(stats), encoding="utf-8")
        return FakeProcess()

    deps = Deps(popen=popen,
                wait_for_bridge_connected=lambda *a: ConnectResult(True,
                                                                   "com"),
                wait_for_twin_ack=lambda *a: True, prompt=lambda m: None)
    assert execute_run(_config(tmp_path, execution_mode=mode), deps) == 0
    return json.loads((logs / "summary.json").read_text(encoding="utf-8"))


def test_free_run_statistics_are_recorded(tmp_path):
    assert _run(tmp_path, "free_run", STATS)["twin_substeps"] == STATS


def test_free_run_without_events_records_null(tmp_path):
    summary = _run(tmp_path, "free_run")
    assert "twin_substeps" in summary and summary["twin_substeps"] is None


def test_lockstep_has_no_substep_key(tmp_path):
    assert "twin_substeps" not in _run(tmp_path, "lockstep", STATS)


def test_the_execution_mode_is_in_the_summary(tmp_path):
    assert _run(tmp_path, "free_run")["execution_mode"] == "free_run"
    other = tmp_path / "b"
    other.mkdir()
    assert _run(other, "lockstep")["execution_mode"] == "lockstep"


GATE = {"opened": True, "wait_s": 94.2,
        "participants": ["acav_scheduler", "matlab_twin"],
        "opened_utc": "2026-10-15T10:00:00Z"}


def _run_with_gate(tmp_path, mode, gate):
    logs = tmp_path / "run.logs"

    def popen(argv, **kwargs):
        if "acav.scheduler_node" in argv and gate is not None:
            (logs / "startup_gate.json").write_text(json.dumps(gate),
                                                    encoding="utf-8")
        return FakeProcess()

    deps = Deps(popen=popen,
                wait_for_bridge_connected=lambda *a: ConnectResult(True,
                                                                   "com"),
                wait_for_twin_ack=lambda *a: True, prompt=lambda m: None)
    code = execute_run(_config(tmp_path, execution_mode=mode), deps)
    summary = json.loads((logs / "summary.json").read_text(encoding="utf-8"))
    return code, summary


def test_the_free_run_start_gate_is_recorded(tmp_path):
    code, summary = _run_with_gate(tmp_path, "free_run", GATE)
    assert code == 0
    assert summary["free_run_start_gate"] == GATE


def test_a_free_run_gate_that_never_opened_fails_the_run(tmp_path):
    code, summary = _run_with_gate(tmp_path, "free_run", None)
    assert code == 1
    assert summary["free_run_start_gate"] is None
    steps = {s["step"]: s for s in summary["steps"]}
    assert steps["free_run_start_gate"]["ok"] is False


def test_lockstep_has_no_gate_key(tmp_path):
    _, summary = _run_with_gate(tmp_path, "lockstep", GATE)
    assert "free_run_start_gate" not in summary
