"""The requested execution mode reaches the bridge and the twin.

Before this, --execution-mode was parsed and stored but never applied:
write_bridge_config copied time.mode from the base YAML (lockstep), so
an E3 "free_run" run ran in lockstep and was recorded as lockstep. The
per-run bridge config now sets time.mode from the run, and the run
config gives the twin "execution_mode" (+adcs/config.m), which selects
its free-run integration. For a lockstep run the bridge config is the
same as before.
"""

import json
import logging
import os

import pytest
import yaml

import acav.deploy_run_cli as deploy_run_cli
from acav.campaign import FREE_RUN, LOCKSTEP, RunSpec
from acav.campaign_runner import write_run_config
from acav.deploy_run_cli import prepare_run_config, write_bridge_config

from test_deploy_run_cli import _config  # noqa: I100 - shared helper

ACAV_YAML = os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "config", "bridge_params_acav.yaml")


def _bridge_settings(tmp_path, **overrides):
    config = _config(tmp_path, bridge_config=ACAV_YAML, **overrides)
    log_dir = str(tmp_path / "run.logs")
    os.makedirs(log_dir, exist_ok=True)
    path = write_bridge_config(config, log_dir, logging.getLogger("t"))
    with open(path, encoding="utf-8") as handle:
        return yaml.safe_load(handle)


class TestBridgeConfig:
    def test_free_run_reaches_the_bridge(self, tmp_path):
        settings = _bridge_settings(tmp_path, execution_mode="free_run")
        assert settings["time"]["mode"] == "free_run"

    def test_lockstep_time_section_is_unchanged(self, tmp_path):
        with open(ACAV_YAML, encoding="utf-8-sig") as handle:
            base = yaml.safe_load(handle)
        settings = _bridge_settings(tmp_path, execution_mode="lockstep")
        assert settings["time"] == base["time"]

    def test_an_unknown_mode_is_refused(self, tmp_path):
        with pytest.raises(ValueError, match="execution mode"):
            _bridge_settings(tmp_path, execution_mode="paced")

    def test_the_cli_accepts_only_the_two_e3_modes(self):
        base = ["--run-id", "r", "--output", "r.json",
                "--bridge-config", "b.yaml"]
        parser = deploy_run_cli._parser()
        assert parser.parse_args(base).execution_mode == "lockstep"
        assert parser.parse_args(
            base + ["--execution-mode", "free_run"]).execution_mode == \
            "free_run"
        with pytest.raises(SystemExit):
            parser.parse_args(base + ["--execution-mode", "paced"])


class TestRunConfig:
    @pytest.mark.parametrize("mode", [LOCKSTEP, FREE_RUN])
    def test_the_campaign_writes_it(self, tmp_path, mode):
        run = RunSpec(experiment="E3", execution_mode=mode, seed=0)
        path = write_run_config(run, str(tmp_path / f"{run.run_id}.json"))
        with open(path, encoding="utf-8") as handle:
            assert json.load(handle)["execution_mode"] == mode

    @pytest.mark.parametrize("mode", [LOCKSTEP, FREE_RUN])
    def test_a_standalone_run_writes_it(self, tmp_path, mode):
        config = _config(tmp_path, execution_mode=mode)
        path, created, _ = prepare_run_config(config, "v")
        assert created
        with open(path, encoding="utf-8") as handle:
            assert json.load(handle)["execution_mode"] == mode

    def test_an_existing_run_config_for_another_mode_is_refused(
            self, tmp_path):
        """A run config written for lockstep must not drive a free-run
        twin, or the twin would integrate the wrong way silently."""
        config = _config(tmp_path, execution_mode="free_run")
        path = tmp_path / "run.runconfig.json"
        path.write_text(json.dumps({"run_id": config.run_id,
                                    "execution_mode": "lockstep"}),
                        encoding="utf-8")
        with pytest.raises(deploy_run_cli.RunConfigError,
                           match="execution_mode"):
            prepare_run_config(_config(tmp_path, execution_mode="free_run",
                                       run_config=str(path)), "v")


class TestStartupGateFile:
    """The free-run start gate's record goes beside the run's logs; a
    lockstep run's bridge config does not name one."""

    def test_free_run_names_the_gate_file(self, tmp_path):
        settings = _bridge_settings(tmp_path, execution_mode="free_run")
        assert settings["startup_gate_file"] == os.path.join(
            os.path.abspath(str(tmp_path / "run.logs")), "startup_gate.json")

    def test_lockstep_does_not(self, tmp_path):
        settings = _bridge_settings(tmp_path, execution_mode="lockstep")
        assert "startup_gate_file" not in settings
