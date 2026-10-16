"""The run orchestrator refuses a twin that loaded the wrong config or
runs other code, and records the identity of the one it accepts.

Between twin_acking and starting the scheduler, run_acav_run.py reads
the twin's identity (/acav/twin_identity, see acav/twin_identity.py)
and checks it against this run's config file and this checkout's code.
Any mismatch fails the run before the scheduler starts -- the bridge
cannot advance without acav_scheduler, so nothing runs. The accepted
identity goes to summary.json and, through the scheduler, into the run
record's provenance (not part of the determinism signature).
"""

import hashlib
import json

import pytest

import acav.deploy_run_cli as deploy_run_cli
from acav import twin_identity as ti
from acav.deploy_run_cli import (
    ConnectResult,
    Deps,
    _confirm_twin_identity,
    execute_run,
    scheduler_argv,
)
from acav.metrics import RunResult, determinism_signature
from acav.run_record import Provenance, RunRecord

from test_deploy_run_cli import FakeProcess, _config  # noqa: I100


@pytest.fixture(autouse=True)
def offline(monkeypatch):
    from types import SimpleNamespace
    import acav.topic_recorder
    monkeypatch.setattr(deploy_run_cli, "_check_graph_idle", lambda p: None)
    monkeypatch.setattr(acav.topic_recorder, "TopicRecorder",
                        lambda *a: SimpleNamespace(close=lambda: None,
                                                   error=None))


def _identity_for(run_config, config_sha=None, build=None):
    code = ti.code_fingerprint(ti.source_root())[:ti.FINGERPRINT_CHARS]
    config_sha = config_sha or ti.file_sha256(run_config)
    return f"v3 {config_sha} {build or code} {code} 100Hz"


@pytest.fixture
def run_config(tmp_path):
    path = tmp_path / "E6-seed1.runconfig.json"
    path.write_text('{"run_id": "E6-seed1"}\n', encoding="utf-8")
    return str(path)


class TestConfirm:
    def test_a_matching_twin(self, run_config):
        text = _identity_for(run_config)
        assert _confirm_twin_identity(
            run_config, 1.0, wait=lambda timeout_s: text) == (text, [])

    def test_no_identity_received(self, run_config):
        text, problems = _confirm_twin_identity(
            run_config, 1.0, wait=lambda timeout_s: None)
        assert text == ""
        assert "no twin identity" in problems[0]
        assert "build_acav_twin" in problems[0]

    def test_the_previous_runs_model(self, run_config):
        previous = hashlib.sha256(b'{"run_id": "E6-seed0"}\n').hexdigest()
        text = _identity_for(run_config, config_sha=previous)
        _, problems = _confirm_twin_identity(run_config, 1.0,
                                             wait=lambda timeout_s: text)
        assert "different run configuration" in problems[0]

    def test_a_stale_model(self, run_config):
        text = _identity_for(run_config, build="0" * 16)
        _, problems = _confirm_twin_identity(run_config, 1.0,
                                             wait=lambda timeout_s: text)
        assert any("rebuild" in p for p in problems)


def _run(tmp_path, confirm, run_config):
    launched = []

    def popen(argv, **kwargs):
        launched.append(argv)
        return FakeProcess()

    deps = Deps(popen=popen,
                wait_for_bridge_connected=lambda *a: ConnectResult(True,
                                                                   "com"),
                wait_for_twin_ack=lambda *a: True, prompt=lambda m: None,
                confirm_twin_identity=confirm)
    code = execute_run(_config(tmp_path, run_config=run_config), deps)
    summary = json.loads((tmp_path / "run.logs" / "summary.json")
                         .read_text(encoding="utf-8"))
    log = (tmp_path / "run.logs" / "run.log").read_text(encoding="utf-8")
    return code, launched, summary, log


class TestExecuteRun:
    def test_a_refused_twin_never_starts_the_scheduler(self, tmp_path,
                                                       run_config):
        code, launched, summary, log = _run(
            tmp_path,
            lambda path, timeout_s: ("v1 x", ["the twin loaded a "
                                              "different run configuration"]),
            run_config)
        assert code == 1
        assert len(launched) == 1                       # the bridge only
        assert "acav.scheduler_node" not in " ".join(launched[0])
        assert summary["twin_identity"] == "v1 x"
        assert summary["twin_identity_problems"] == [
            "the twin loaded a different run configuration"]
        assert "twin_identity" in log and "different run configuration" in log

    def test_an_accepted_twin_is_recorded_and_passed_on(self, tmp_path,
                                                        run_config):
        text = _identity_for(run_config)
        seen = []

        def confirm(path, timeout_s):
            seen.append(path)
            return text, []

        code, launched, summary, _ = _run(tmp_path, confirm, run_config)
        assert code == 0
        assert seen == [run_config]
        assert summary["twin_identity"] == text
        assert summary["twin_identity_problems"] == []
        scheduler = launched[1]
        assert f"twin_identity:={json.dumps(text)}" in scheduler

    def test_the_real_default_is_the_ros_backed_check(self):
        assert Deps.__dataclass_fields__["confirm_twin_identity"].default \
            is None
        import inspect
        source = inspect.getsource(deploy_run_cli._wait_for_twin_identity_ros)
        assert "from stk_bridge.interfaces.qos import command_qos" in source
        assert "IDENTITY_TOPIC" in source

    def test_scheduler_argv_without_an_identity(self, tmp_path):
        argv = scheduler_argv(_config(tmp_path))
        assert f"twin_identity:={json.dumps('')}" in argv


class TestRecorded:
    def test_provenance_round_trip_and_old_records(self):
        provenance = Provenance(twin_identity="v1 abc")
        assert Provenance.from_dict(provenance.to_dict()).twin_identity == \
            "v1 abc"
        assert Provenance.from_dict({}).twin_identity == ""

    def test_the_identity_is_not_part_of_the_signature(self):
        result = RunResult(run_id="r", scheduler="dynamics_aware")
        with_identity = RunRecord(result=result,
                                  provenance=Provenance(twin_identity="v1"))
        without = RunRecord(result=result, provenance=Provenance())
        assert with_identity.signature == without.signature == \
            determinism_signature(result)

    def test_the_scheduler_writes_it(self, tmp_path):
        from types import SimpleNamespace
        from acav.run_record import read_record
        from acav.scheduler_node import SchedulerNode
        node = SchedulerNode.__new__(SchedulerNode)
        out = str(tmp_path / "run.json")
        node._options = SimpleNamespace(
            output_json=out, experiment="E6", scenario_path="",
            gains_json="{}", fault_json="null", attitude_provider="",
            twin_identity="v1 accepted")
        node._backend_type, node._stk_version = "com", "12.6.0"
        node._execution_mode, node._started_utc = "lockstep", ""
        node._readback_provider, node._readback_forced = "", None
        node._write_output(RunResult(run_id="r", scheduler="dynamics_aware"))
        assert read_record(out).provenance.twin_identity == "v1 accepted"
