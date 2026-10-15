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


@pytest.fixture(autouse=True)
def offline_graph(monkeypatch):
    monkeypatch.setattr(deploy_run_cli, "_check_graph_idle", lambda prefix: None)
    from types import SimpleNamespace
    import acav.topic_recorder
    monkeypatch.setattr(acav.topic_recorder, "TopicRecorder",
                        lambda *args: SimpleNamespace(close=lambda: None, error=None))


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


def test_twin_wait_defaults_to_one_minute_and_allows_override():
    parser = deploy_run_cli._parser()
    args = ["--run-id", "probe", "--output", "probe.json",
            "--bridge-config", "bridge.yaml"]
    assert parser.parse_args(args).twin_ack_timeout_s == 60.0
    assert parser.parse_args(args + ["--twin-ack-timeout-s", "15"]).twin_ack_timeout_s == 15.0
    assert RunConfig.__dataclass_fields__["twin_ack_timeout_s"].default == 60.0


def _config(tmp_path, **overrides):
    base = tmp_path / "bridge.yaml"
    if not base.exists():
        base.write_text("sensing:\n  satellites: ['*/Satellite/SSS-1']\n",
                        encoding="utf-8")
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
        assert argv[1:3] == ["-m", "stk_bridge.nodes.bridge_node"]
        assert f"config_file:={json.dumps(config.bridge_config)}" in argv

    def test_scheduler_argv_carries_every_run_identifying_field(self, tmp_path):
        config = _config(tmp_path, seed="42")
        argv = scheduler_argv(config)
        import yaml
        params = dict(item.split(":=", 1) for item in argv if ":=" in item)
        values = {name: yaml.safe_load(value) for name, value in params.items()}
        assert values["scheduler_type"] == "dynamics_aware"
        assert values["output_json"] == config.output
        assert values["run_id"] == "E6-seed1"
        assert values["experiment"] == "E6"
        assert values["seed"] == 42
        assert values["auto_shutdown"] is True

    @pytest.mark.parametrize("value", ["", "{}", "null", "123", "true",
                                       '{"gain": 1.2}', r"C:\My Runs\test.json"])
    def test_string_parameters_keep_their_value_and_type(self, tmp_path, value):
        import yaml
        argv = scheduler_argv(_config(tmp_path, scenario=value, gains_json=value))
        params = dict(item.split(":=", 1) for item in argv if ":=" in item)
        for name in ("scenario_config", "gains_json"):
            decoded = yaml.safe_load(params[name])
            assert isinstance(decoded, str)
            assert decoded == value

    def test_scheduler_argv_omits_seed_param_when_unset(self, tmp_path):
        config = _config(tmp_path, seed="")
        argv = scheduler_argv(config)
        assert "seed:=" not in " ".join(argv)


class TestPerRunBridgeConfig:
    """``--attitude-provider`` must reach the bridge, not only the record.

    Until 2026-10 it reached only the scheduler: the record said
    "treatment" whatever the bridge read through, and E1's treatment ran
    only because a hand-made YAML was passed alongside it.
    """

    def _launched_config(self, tmp_path, **overrides):
        import yaml
        config = _config(tmp_path, **overrides)
        launched = []
        def popen(argv, **kwargs):
            launched.append(argv)
            return FakeProcess()
        deps = Deps(popen=popen,
                    wait_for_bridge_connected=lambda *a: ConnectResult(True, "com"),
                    wait_for_twin_ack=lambda *a: True, prompt=lambda m: None)
        assert execute_run(config, deps) == 0
        params = dict(item.split(":=", 1) for item in launched[0] if ":=" in item)
        path = json.loads(params["config_file"])
        assert path != config.bridge_config
        with open(path, encoding="utf-8") as handle:
            return yaml.safe_load(handle)

    def test_the_treatment_provider_is_forced_on_the_bridge(self, tmp_path):
        settings = self._launched_config(
            tmp_path, attitude_provider="ECF Attitude Quaternions")
        assert settings["sensing"]["force_attitude_provider"] == \
            "ECF Attitude Quaternions"
        assert settings["sensing"]["satellites"] == ["*/Satellite/SSS-1"]

    def test_a_provider_left_in_the_base_file_is_cleared(self, tmp_path):
        (tmp_path / "bridge.yaml").write_text(
            "sensing:\n  force_attitude_provider: ECF Attitude Quaternions\n",
            encoding="utf-8")
        settings = self._launched_config(tmp_path, attitude_provider="")
        assert settings["sensing"]["force_attitude_provider"] == ""
        # The base file itself is not edited.
        assert "ECF" in (tmp_path / "bridge.yaml").read_text(encoding="utf-8")

    def test_an_unreadable_base_config_fails_before_any_launch(self, tmp_path):
        config = _config(tmp_path, bridge_config=str(tmp_path / "missing.yaml"))
        launched = []
        deps = Deps(popen=lambda *a, **k: launched.append(a),
                    wait_for_bridge_connected=lambda *a: ConnectResult(True, "com"),
                    wait_for_twin_ack=lambda *a: True, prompt=lambda m: None)
        assert execute_run(config, deps) == deploy_run_cli.EXIT_STEP_FAILED
        assert launched == []
        summary = json.loads(
            (tmp_path / "run.logs" / "summary.json").read_text(encoding="utf-8"))
        assert summary["steps"][0]["step"] == "bridge_config"
        assert not summary["steps"][0]["ok"]


class TestExecuteRunHappyPath:
    def test_bridge_is_stopped_before_waiting_for_manual_twin_stop(self, tmp_path):
        bridge, scheduler = FakeProcess(), FakeProcess()
        procs = iter([bridge, scheduler])
        prompts = []
        def prompt(message):
            prompts.append(message)
            if 'Scheduler finished' in message:
                assert bridge.terminated
                assert bridge.waited
        deps = Deps(popen=lambda *a, **k: next(procs),
                    wait_for_bridge_connected=lambda *a: ConnectResult(True, 'com'),
                    wait_for_twin_ack=lambda *a: True, prompt=prompt)
        assert execute_run(_config(tmp_path), deps) == 0
        assert len(prompts) == 2

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
        assert steps == {"idle_check": True,
                         "bridge_connected": True, "twin_acking": True,
                         "twin_identity": True,
                         "scheduler_exit": True, "topic_capture": True}
        assert bridge_proc.terminated

    def test_the_code_version_is_resolved_recorded_and_exported(
            self, tmp_path, monkeypatch):
        from acav import code_version
        # A stale value in the operator's shell must not be trusted.
        monkeypatch.setenv(code_version.ENV_VAR, "stale")
        config = _config(tmp_path)
        deps = Deps(
            popen=lambda *a, **k: FakeProcess(),
            wait_for_bridge_connected=lambda *a, **k: ConnectResult(True, "com"),
            wait_for_twin_ack=lambda *a, **k: True,
            prompt=lambda message: None,
        )
        execute_run(config, deps)
        with open(os.path.join(config.resolved_log_dir(), "summary.json")) as f:
            recorded = json.load(f)["code_version"]
        assert recorded != "stale" and not recorded.startswith("unknown")
        # Exported for the bridge and scheduler started after it.
        assert os.environ[code_version.ENV_VAR] == recorded

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

def test_busy_graph_prevents_launch(tmp_path):
    def busy(prefix):
        raise RuntimeError('Another run is active')
    from unittest.mock import Mock
    popen = Mock()
    assert execute_run(_config(tmp_path),
                       Deps(check_idle=busy, popen=popen)) == 1
    popen.assert_not_called()


def test_interrupt_stops_scheduler_and_bridge(tmp_path):
    class InterruptedProcess(FakeProcess):
        def wait(self, timeout=None):
            if not self.terminated:
                raise KeyboardInterrupt()
            return 0
    bridge = FakeProcess(); scheduler = InterruptedProcess()
    processes = iter([bridge, scheduler])
    deps = Deps(popen=lambda *a, **k: next(processes),
                wait_for_bridge_connected=lambda *a: ConnectResult(True,'com'),
                wait_for_twin_ack=lambda *a: True, prompt=lambda text: None)
    with pytest.raises(KeyboardInterrupt):
        execute_run(_config(tmp_path), deps)
    assert bridge.terminated and scheduler.terminated


class TestStandaloneRunConfig:
    """A standalone run must have its MATLAB run configuration on disk
    before any bridge starts. Campaign runs already wrote one
    (campaign_runner.write_run_config); a hand-launched smoke run printed
    a RunConfigPath that did not exist, so build_acav_twin could not
    start after a clean setup.
    """

    def _run(self, tmp_path, **overrides):
        config = _config(tmp_path, **overrides)
        events = {"launched": [], "prompts": [], "existed_at_launch": []}

        def popen(argv, **kwargs):
            events["launched"].append(argv)
            events["existed_at_launch"].append(
                os.path.isfile(self._expected_path(config)))
            return FakeProcess()

        deps = Deps(popen=popen,
                    wait_for_bridge_connected=lambda *a: ConnectResult(True, "com"),
                    wait_for_twin_ack=lambda *a: True,
                    prompt=events["prompts"].append)
        code = execute_run(config, deps)
        summary = json.loads(open(os.path.join(
            config.resolved_log_dir(), "summary.json"), encoding="utf-8").read())
        return code, events, summary, config

    @staticmethod
    def _expected_path(config):
        if config.run_config:
            return os.path.abspath(config.run_config)
        return os.path.abspath(
            os.path.splitext(config.output)[0] + ".runconfig.json")

    def test_a_missing_config_is_created_before_the_bridge_launches(self, tmp_path):
        code, events, summary, config = self._run(
            tmp_path, gains_json='{"natural_frequency_rad_s": 1.2}',
            fault_json='{"axis": 1, "at_s": 12.5, "kind": "saturation"}')
        assert code == 0
        assert events["existed_at_launch"] == [True, True]
        payload = json.loads(open(self._expected_path(config),
                                  encoding="utf-8").read())
        assert payload["run_id"] == "E6-seed1"
        assert payload["experiment"] == "E6"
        assert payload["gains"] == {"natural_frequency_rad_s": 1.2}
        assert payload["fault"] == {"axis": 1, "at_s": 12.5, "kind": "saturation"}
        assert payload["attitude_provider"] == ""
        assert payload["attitude_mismatch_policy"] == "strict"
        assert payload["code_version"] == summary["code_version"]

    def test_default_path_is_beside_the_output_and_printed_absolute(self, tmp_path):
        code, events, summary, config = self._run(tmp_path, run_config="")
        expected = str(tmp_path / "run.runconfig.json")
        assert os.path.isfile(expected)
        assert summary["run_config"] == expected
        assert expected in events["prompts"][0]

    def test_a_relative_path_is_resolved_and_printed_absolute(
            self, tmp_path, monkeypatch):
        monkeypatch.chdir(tmp_path)
        code, events, summary, config = self._run(
            tmp_path, run_config=os.path.join("cfg", "smoke.runconfig.json"))
        expected = str(tmp_path / "cfg" / "smoke.runconfig.json")
        assert code == 0 and os.path.isfile(expected)
        assert summary["run_config"] == expected
        assert expected in events["prompts"][0]

    def test_the_created_file_has_no_byte_order_mark(self, tmp_path):
        self._run(tmp_path)
        raw = (tmp_path / "run.runconfig.json").read_bytes()
        assert raw[:3] != b"\xef\xbb\xbf"
        json.loads(raw.decode("utf-8"))      # strict UTF-8, no BOM

    @pytest.mark.parametrize("experiment, provider, policy", [
        ("E1", "ECF Attitude Quaternions", "e1_readback_ablation"),
        ("E1", "", "strict"),
        ("E2", "ECF Attitude Quaternions", "strict"),
        ("E6", "", "strict"),
    ])
    def test_the_mismatch_policy_follows_the_campaign_rule(
            self, tmp_path, experiment, provider, policy):
        from acav.campaign import RunSpec
        from acav.campaign_runner import write_run_config
        self._run(tmp_path, experiment=experiment, attitude_provider=provider)
        standalone = json.loads((tmp_path / "run.runconfig.json")
                                .read_text(encoding="utf-8"))
        assert standalone["attitude_mismatch_policy"] == policy
        # Same rule and same fields as the campaign writer.
        (tmp_path / "campaign").mkdir()
        written = write_run_config(
            RunSpec(experiment=experiment, attitude_provider=provider),
            str(tmp_path / "campaign" / "c.json"))
        campaign = json.loads(open(written, encoding="utf-8").read())
        assert campaign["attitude_mismatch_policy"] == policy
        assert set(standalone) == set(campaign)

    def test_an_existing_config_is_preserved_byte_for_byte(self, tmp_path):
        path = tmp_path / "run.runconfig.json"
        original = (b'{"run_id": "E6-seed1", "gains": '
                    b'{"profile_authority": 0.6}, "fault": null}\n')
        path.write_bytes(original)
        code, events, summary, config = self._run(
            tmp_path, gains_json='{"profile_authority": 0.9}')
        assert code == 0
        assert path.read_bytes() == original

    def test_a_campaign_written_config_is_used_unchanged(self, tmp_path):
        from acav.campaign import RunSpec
        from acav.campaign_runner import write_run_config
        run = RunSpec(experiment="E1",
                      attitude_provider="ECF Attitude Quaternions")
        output = str(tmp_path / f"{run.run_id}.json")
        path = write_run_config(run, output)
        before = open(path, "rb").read()
        code, events, summary, config = self._run(
            tmp_path, run_id=run.run_id, experiment="E1",
            attitude_provider="ECF Attitude Quaternions",
            output=output, run_config=path)
        assert code == 0
        assert open(path, "rb").read() == before

    @pytest.mark.parametrize("content, reason", [
        (b"{not json", "not valid"),
        (b"[1, 2, 3]", "JSON object"),
        (b'"text"', "JSON object"),
        (b'{"run_id": "someone-else"}', "someone-else"),
        (b'\xef\xbb\xbf{"run_id": "E6-seed1"}', "byte-order mark"),
    ])
    def test_a_bad_existing_config_fails_before_any_launch(
            self, tmp_path, content, reason):
        path = tmp_path / "run.runconfig.json"
        path.write_bytes(content)
        code, events, summary, config = self._run(tmp_path)
        assert code == deploy_run_cli.EXIT_STEP_FAILED
        assert events["launched"] == []
        assert summary["steps"][0]["step"] == "run_config"
        assert not summary["steps"][0]["ok"]
        assert reason in summary["steps"][0]["detail"]
        assert path.read_bytes() == content

    def test_the_bom_rejection_says_how_to_fix_it(self, tmp_path):
        (tmp_path / "run.runconfig.json").write_bytes(
            b'\xef\xbb\xbf{"run_id": "E6-seed1"}')
        code, events, summary, config = self._run(tmp_path)
        detail = summary["steps"][0]["detail"]
        assert "re-save it as UTF-8 without a byte-order mark" in detail
        assert "delete it so it is regenerated" in detail

    def test_an_existing_config_without_run_id_is_used_with_a_warning(
            self, tmp_path):
        path = tmp_path / "run.runconfig.json"
        original = b'{"gains": {}, "fault": null}\n'
        path.write_bytes(original)
        code, events, summary, config = self._run(tmp_path)
        assert code == 0
        assert path.read_bytes() == original
        assert summary["run_config_run_id_verified"] is False
        log = open(os.path.join(config.resolved_log_dir(), "run.log"),
                   encoding="utf-8").read()
        assert ("WARNING run config has no run_id; cannot verify it "
                "belongs to this run") in log

    @pytest.mark.parametrize("content", [
        None, b'{"run_id": "E6-seed1", "gains": {}}'])
    def test_a_created_or_matching_config_is_recorded_as_verified(
            self, tmp_path, content):
        if content is not None:
            (tmp_path / "run.runconfig.json").write_bytes(content)
        code, events, summary, config = self._run(tmp_path)
        assert code == 0
        assert summary["run_config_run_id_verified"] is True
        log = open(os.path.join(config.resolved_log_dir(), "run.log"),
                   encoding="utf-8").read()
        assert "has no run_id" not in log

    @pytest.mark.parametrize("field, value", [
        ("gains_json", "{oops"), ("gains_json", "[1]"),
        ("fault_json", "{oops"), ("fault_json", "3"),
    ])
    def test_unusable_requested_overrides_fail_before_any_launch(
            self, tmp_path, field, value):
        code, events, summary, config = self._run(tmp_path, **{field: value})
        assert code == deploy_run_cli.EXIT_STEP_FAILED
        assert events["launched"] == []
        assert not (tmp_path / "run.runconfig.json").exists()


def test_topic_recorder_starts_before_bridge_and_closes_on_connection_failure(tmp_path):
    from types import SimpleNamespace
    events = []
    recorder = SimpleNamespace(error=None, close=lambda: events.append('closed'))
    def start(config, directory):
        events.append('recording')
        assert directory == config.resolved_log_dir()
        return recorder
    def launch(*args, **kwargs):
        events.append('bridge')
        return FakeProcess()
    code = execute_run(_config(tmp_path), Deps(
        start_recorder=start, popen=launch,
        wait_for_bridge_connected=lambda *args: ConnectResult(False),
        prompt=lambda message: None))
    assert code != 0
    assert events == ['recording', 'bridge', 'closed']


class TestEquivalenceDiagnosticFlag:
    """The diagnostic is enabled only by a per-run bridge configuration,
    writes next to the run's logs, and marks the run as diagnostic."""

    def _run(self, tmp_path, base_yaml):
        import yaml
        (tmp_path / "bridge.yaml").write_text(base_yaml, encoding="utf-8")
        config = _config(tmp_path)
        launched = []
        deps = Deps(popen=lambda argv, **k: launched.append(argv) or FakeProcess(),
                    wait_for_bridge_connected=lambda *a: ConnectResult(True, "com"),
                    wait_for_twin_ack=lambda *a: True, prompt=lambda m: None)
        assert execute_run(config, deps) == 0
        params = dict(item.split(":=", 1) for item in launched[0] if ":=" in item)
        with open(json.loads(params["config_file"]), encoding="utf-8") as handle:
            settings = yaml.safe_load(handle)
        summary = json.loads((tmp_path / "run.logs" / "summary.json")
                             .read_text(encoding="utf-8"))
        return settings, summary, config

    def test_off_by_default_and_recorded_as_false(self, tmp_path):
        settings, summary, _ = self._run(
            tmp_path, "sensing:\n  satellites: ['*/Satellite/SSS-1']\n")
        assert summary["equivalence_diagnostic"] is False
        assert "equivalence_diagnostic_dir" not in settings["sensing"]

    def test_enabled_writes_next_to_the_logs_and_is_recorded(self, tmp_path):
        settings, summary, config = self._run(
            tmp_path, "sensing:\n  equivalence_diagnostic: true\n")
        assert summary["equivalence_diagnostic"] is True
        assert settings["sensing"]["equivalence_diagnostic_dir"] == \
            os.path.abspath(config.resolved_log_dir())

    def test_an_explicit_directory_is_kept(self, tmp_path):
        settings, summary, _ = self._run(
            tmp_path, "sensing:\n  equivalence_diagnostic: true\n"
                      "  equivalence_diagnostic_dir: C:/elsewhere\n")
        assert settings["sensing"]["equivalence_diagnostic_dir"] == \
            "C:/elsewhere"


class TestSensingModeLabelsAndE1Exclusion:
    """Decisions Q2/Q3: the per-run bridge config and summary.json carry
    the sensing mode, and E1 (both arms) and any forced provider stay on
    the original path whatever the base configuration says."""

    ACAV_LIKE = ("sensing:\n  combine_attitude_rate: true\n"
                 "  orbit_batch_samples: 100\n")

    def _run(self, tmp_path, base_yaml, **overrides):
        import yaml
        (tmp_path / "bridge.yaml").write_text(base_yaml, encoding="utf-8")
        config = _config(tmp_path, **overrides)
        launched = []
        deps = Deps(popen=lambda argv, **k: launched.append(argv)
                    or FakeProcess(),
                    wait_for_bridge_connected=lambda *a: ConnectResult(
                        True, "com"),
                    wait_for_twin_ack=lambda *a: True, prompt=lambda m: None)
        assert execute_run(config, deps) == 0
        params = dict(item.split(":=", 1) for item in launched[0]
                      if ":=" in item)
        with open(json.loads(params["config_file"]),
                  encoding="utf-8") as handle:
            sensing = yaml.safe_load(handle)["sensing"]
        summary = json.loads((tmp_path / "run.logs" / "summary.json")
                             .read_text(encoding="utf-8"))
        return sensing, summary

    def test_an_ordinary_run_keeps_the_base_settings_and_labels_them(
            self, tmp_path):
        sensing, summary = self._run(tmp_path, self.ACAV_LIKE)
        assert sensing["combine_attitude_rate"] is True
        assert sensing["orbit_batch_samples"] == 100
        assert sensing["sensing_mode"] == "combined_attitude_rate"
        assert summary["sensing_mode"] == "combined_attitude_rate"
        assert summary["orbit_batch_samples"] == 100

    def test_defaults_are_separate_and_one_sample(self, tmp_path):
        sensing, summary = self._run(tmp_path, "sensing: {}\n")
        assert sensing["combine_attitude_rate"] is False
        assert sensing["orbit_batch_samples"] == 1
        assert summary["sensing_mode"] == "separate"
        assert summary["orbit_batch_samples"] == 1

    @pytest.mark.parametrize("provider", ["", "ECF Attitude Quaternions"])
    def test_e1_both_arms_stay_on_the_original_path(self, tmp_path,
                                                    provider):
        sensing, summary = self._run(tmp_path, self.ACAV_LIKE,
                                     experiment="E1",
                                     attitude_provider=provider)
        assert sensing["combine_attitude_rate"] is False
        assert sensing["orbit_batch_samples"] == 1
        assert summary["sensing_mode"] == "separate"
        assert summary["orbit_batch_samples"] == 1

    def test_a_forced_provider_outside_e1_is_not_combined(self, tmp_path):
        sensing, summary = self._run(
            tmp_path, self.ACAV_LIKE, experiment="E6",
            attitude_provider="ECF Attitude Quaternions")
        assert sensing["combine_attitude_rate"] is False
        assert summary["sensing_mode"] == "separate"


class TestOrbitBatchWindowCrossCheck:
    """Decision Q4: the bridge cross-checks STK's stop time against the
    scenario window, which run_acav_run.py passes in from the spec."""

    def _sensing(self, tmp_path, base_yaml, **overrides):
        import yaml
        (tmp_path / "bridge.yaml").write_text(base_yaml, encoding="utf-8")
        config = _config(tmp_path, **overrides)
        launched = []
        deps = Deps(popen=lambda argv, **k: launched.append(argv)
                    or FakeProcess(),
                    wait_for_bridge_connected=lambda *a: ConnectResult(
                        True, "com"),
                    wait_for_twin_ack=lambda *a: True, prompt=lambda m: None)
        assert execute_run(config, deps) == 0
        params = dict(item.split(":=", 1) for item in launched[0]
                      if ":=" in item)
        with open(json.loads(params["config_file"]),
                  encoding="utf-8") as handle:
            return yaml.safe_load(handle)["sensing"]

    def test_the_window_is_passed_when_batching(self, tmp_path):
        from acav.spec import AcavSpec
        sensing = self._sensing(tmp_path,
                                "sensing:\n  orbit_batch_samples: 100\n")
        assert sensing["orbit_batch_expected_window_s"] == \
            AcavSpec.load().window_duration_s

    def test_not_passed_without_batching(self, tmp_path):
        sensing = self._sensing(tmp_path, "sensing: {}\n")
        assert "orbit_batch_expected_window_s" not in sensing

    def test_not_passed_for_e1(self, tmp_path):
        sensing = self._sensing(tmp_path,
                                "sensing:\n  orbit_batch_samples: 100\n",
                                experiment="E1")
        assert "orbit_batch_expected_window_s" not in sensing
