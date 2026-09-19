"""Offline tests for the deploy-machine run orchestrator.

No ROS, no subprocess, no live STK, no human at a keyboard: every piece
that touches any of those is injected via ``Deps`` (see
``acav/deploy_run_cli.py``'s own docstring for why), so what is tested
here is the state machine -- bridge up, then twin confirmed, then the
scheduler run, then teardown -- and that a failure at any step is
reported honestly rather than treated as success.
"""

import inspect
import json
import os

import pytest

import acav.deploy_run_cli as deploy_run_cli
from acav.deploy_run_cli import (
    ConnectResult,
    Deps,
    RunConfig,
    bridge_launch_argv,
    execute_run,
    scheduler_argv,
)


class FakeProcess:
    def __init__(self, returncode=0):
        self._returncode = returncode
        self.terminated = False
        self.killed = False
        self.waited = False

    def wait(self, timeout=None):
        self.waited = True
        return self._returncode

    def poll(self):
        return None if not self.terminated else self._returncode

    def terminate(self):
        self.terminated = True

    def kill(self):
        self.killed = True


def _config(tmp_path, **overrides):
    defaults = dict(
        run_id="E6-seed1", experiment="E6", scheduler="dynamics_aware",
        execution_mode="lockstep", seed="1",
        output=str(tmp_path / "run.json"), scenario="",
        backend="com", attitude_provider="", run_config="",
        gains_json="{}", fault_json="null",
        bridge_config=str(tmp_path / "bridge.yaml"),
        bridge_connect_timeout_s=1.0, twin_ack_timeout_s=1.0,
    )
    defaults.update(overrides)
    return RunConfig(**defaults)


class TestArgvBuilding:
    def test_bridge_launch_names_the_config_file(self, tmp_path):
        config = _config(tmp_path)
        argv = bridge_launch_argv(config)
        assert argv[:3] == ["ros2", "launch", "stk_bridge"]
        assert f"config_file:={config.bridge_config}" in argv

    def test_scheduler_argv_carries_every_run_identifying_field(self, tmp_path):
        config = _config(tmp_path, seed="42")
        argv = scheduler_argv(config)
        joined = " ".join(argv)
        for expected in ("scheduler_type:=dynamics_aware",
                        "output_json:=", "run_id:=E6-seed1",
                        "experiment:=E6", "seed:=42"):
            assert expected in joined

    def test_scheduler_argv_omits_seed_param_when_unset(self, tmp_path):
        config = _config(tmp_path, seed="")
        argv = scheduler_argv(config)
        assert "seed:=" not in " ".join(argv)


class TestExecuteRunHappyPath:
    def test_a_clean_run_returns_ok_and_writes_a_summary(self, tmp_path):
        config = _config(tmp_path)
        bridge_proc = FakeProcess()
        scheduler_proc = FakeProcess(returncode=0)
        procs = iter([bridge_proc, scheduler_proc])

        deps = Deps(
            popen=lambda *a, **k: next(procs),
            wait_for_bridge_connected=lambda *a, **k: ConnectResult(True, "com"),
            wait_for_twin_ack=lambda *a, **k: True,
            prompt=lambda message: None,
        )

        code = execute_run(config, deps)

        assert code == 0
        summary_path = os.path.join(config.resolved_log_dir(), "summary.json")
        with open(summary_path) as handle:
            summary = json.load(handle)
        steps = {s["step"]: s["ok"] for s in summary["steps"]}
        assert steps == {"bridge_connected": True, "twin_acking": True,
                         "scheduler_exit": True}
        assert bridge_proc.terminated

    def test_logs_land_next_to_the_output_by_default(self, tmp_path):
        config = _config(tmp_path)
        deps = Deps(
            popen=lambda *a, **k: FakeProcess(),
            wait_for_bridge_connected=lambda *a, **k: ConnectResult(True, "com"),
            wait_for_twin_ack=lambda *a, **k: True,
            prompt=lambda message: None,
        )
        execute_run(config, deps)
        log_dir = config.resolved_log_dir()
        assert os.path.isfile(os.path.join(log_dir, "run.log"))
        assert os.path.isfile(os.path.join(log_dir, "bridge.log"))
        assert os.path.isfile(os.path.join(log_dir, "scheduler.log"))

    def test_an_explicit_log_dir_is_honoured(self, tmp_path):
        explicit = str(tmp_path / "somewhere_else")
        config = _config(tmp_path, log_dir=explicit)
        deps = Deps(
            popen=lambda *a, **k: FakeProcess(),
            wait_for_bridge_connected=lambda *a, **k: ConnectResult(True, "com"),
            wait_for_twin_ack=lambda *a, **k: True,
            prompt=lambda message: None,
        )
        execute_run(config, deps)
        assert os.path.isfile(os.path.join(explicit, "run.log"))


class TestExecuteRunFailureModes:
    def test_bridge_never_connecting_fails_before_touching_the_twin(self, tmp_path):
        config = _config(tmp_path)
        twin_wait_calls = []
        deps = Deps(
            popen=lambda *a, **k: FakeProcess(),
            wait_for_bridge_connected=lambda *a, **k: ConnectResult(
                False, "", "timed out"),
            wait_for_twin_ack=lambda *a, **k: twin_wait_calls.append(1) or True,
            prompt=lambda message: None,
        )
        code = execute_run(config, deps)
        assert code == 1
        assert twin_wait_calls == []

    def test_twin_never_acking_fails_before_starting_the_scheduler(self, tmp_path):
        config = _config(tmp_path)
        scheduler_started = []

        def popen(argv, **kwargs):
            if "acav_scheduler" in argv:
                scheduler_started.append(argv)
            return FakeProcess()

        deps = Deps(
            popen=popen,
            wait_for_bridge_connected=lambda *a, **k: ConnectResult(True, "com"),
            wait_for_twin_ack=lambda *a, **k: False,
            prompt=lambda message: None,
        )
        code = execute_run(config, deps)
        assert code == 1
        assert scheduler_started == []

    def test_a_nonzero_scheduler_exit_is_reported_not_swallowed(self, tmp_path):
        config = _config(tmp_path)
        procs = iter([FakeProcess(), FakeProcess(returncode=7)])
        deps = Deps(
            popen=lambda *a, **k: next(procs),
            wait_for_bridge_connected=lambda *a, **k: ConnectResult(True, "com"),
            wait_for_twin_ack=lambda *a, **k: True,
            prompt=lambda message: None,
        )
        code = execute_run(config, deps)
        assert code == 1

    def test_a_com_run_that_silently_becomes_mock_is_logged_not_hidden(self, tmp_path):
        config = _config(tmp_path, backend="com")
        deps = Deps(
            popen=lambda *a, **k: FakeProcess(),
            wait_for_bridge_connected=lambda *a, **k: ConnectResult(True, "mock"),
            wait_for_twin_ack=lambda *a, **k: True,
            prompt=lambda message: None,
        )
        code = execute_run(config, deps)
        assert code == 0  # not fatal -- reportability is derived downstream
        with open(os.path.join(config.resolved_log_dir(), "run.log")) as handle:
            text = handle.read()
        assert "mock" in text.lower()

    def test_the_bridge_process_is_always_terminated_even_on_failure(self, tmp_path):
        config = _config(tmp_path)
        bridge_proc = FakeProcess()
        deps = Deps(
            popen=lambda *a, **k: bridge_proc,
            wait_for_bridge_connected=lambda *a, **k: ConnectResult(
                False, "", "no status ever received"),
            wait_for_twin_ack=lambda *a, **k: True,
            prompt=lambda message: None,
        )
        execute_run(config, deps)
        assert bridge_proc.terminated


class TestRealWaitFunctionsUseTheBridgesOwnQosProfiles:
    """Regression guard for a real bug found on the Windows deploy box.

    ``{prefix}/status`` is published RELIABLE + TRANSIENT_LOCAL
    (`stk_bridge/interfaces/qos.py::status_qos`), so a late-joining
    watcher must request at least TRANSIENT_LOCAL or DDS never matches
    it to the publisher at all -- not "sees it late", *never sees it*,
    with no error either side. An earlier version of
    ``_wait_for_bridge_connected_ros`` hand-rolled a VOLATILE profile,
    which reproduced exactly that against a live bridge that was
    actually CONNECTED the whole time: "no BridgeStatus received at
    all" for the full 90s timeout. These functions cannot be exercised
    for real without ROS sourced (see the module's own lazy-import
    comment), so this is a static guard on the source instead of a
    behavioural test: it fails loudly if either function goes back to
    building its own QoSProfile instead of reusing the bridge's.
    """

    def test_bridge_wait_reuses_the_bridges_status_qos(self):
        source = inspect.getsource(
            deploy_run_cli._wait_for_bridge_connected_ros)
        assert "from stk_bridge.interfaces.qos import status_qos" in source
        assert "status_qos(" in source
        # The bug was a hand-rolled QoSProfile importing this type
        # directly; status_qos() encapsulates it, so this function
        # should no longer need to import it at all.
        assert "QoSDurabilityPolicy" not in source

    def test_twin_ack_wait_reuses_the_bridges_command_qos(self):
        source = inspect.getsource(deploy_run_cli._wait_for_twin_ack_ros)
        assert "from stk_bridge.interfaces.qos import command_qos" in source
        assert "command_qos(" in source
