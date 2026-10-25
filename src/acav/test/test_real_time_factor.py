"""The real-time factor and free-run sensing are recorded truthfully.

* ``--real-time-factor`` sets the per-run bridge config's
  time.real_time_factor (default: the base YAML's value, 1.0; values
  <= 0 are refused). The effective value goes to summary.json and to
  provenance.real_time_factor; ``report`` refuses records with different
  effective values. A record without the field counts as 1.0 only when
  it is lockstep (simulation time there advances by steps, not by the
  wall clock) or when its bridge.log shows ``rtf=1.000``; otherwise it is
  refused as unknown.
* Free-run turns the combined attitude/rate read off and orbit
  batching to 1 in the per-run bridge config, so summary.json says
  "separate" and 1 -- the bridge never ran either in free-run.
* A free-run bridge writes its pacing summary to the file the per-run
  config names; the runner copies it into summary.json as
  "free_run_pacing". Lockstep runs carry none of these free-run keys.
"""

import json
import logging
import os

import pytest
import yaml

import acav.deploy_run_cli as deploy_run_cli
from acav.campaign_cli import effective_real_time_factors, main
from acav.deploy_run_cli import (
    ConnectResult,
    Deps,
    execute_run,
    write_bridge_config,
)
from acav.metrics import RunResult
from acav.run_record import Provenance, RunRecord, read_record, write_record

from test_campaign import _result, covering  # noqa: I100 - shared helper
from test_deploy_run_cli import FakeProcess, _config  # noqa: I100

ACAV_YAML = os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "config", "bridge_params_acav.yaml")
PACING = {"cycles": 5321, "overruns": 4210, "period_s": 0.1}


@pytest.fixture(autouse=True)
def offline(monkeypatch):
    from types import SimpleNamespace
    import acav.topic_recorder
    monkeypatch.setattr(deploy_run_cli, "_check_graph_idle", lambda p: None)
    monkeypatch.setattr(acav.topic_recorder, "TopicRecorder",
                        lambda *a: SimpleNamespace(close=lambda: None,
                                                   error=None))


def _settings(tmp_path, **overrides):
    config = _config(tmp_path, bridge_config=ACAV_YAML, **overrides)
    log_dir = str(tmp_path / "run.logs")
    os.makedirs(log_dir, exist_ok=True)
    with open(write_bridge_config(config, log_dir, logging.getLogger("t")),
              encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def _base():
    with open(ACAV_YAML, encoding="utf-8-sig") as handle:
        return yaml.safe_load(handle)


class TestOption:
    def _parse(self, *extra):
        return deploy_run_cli._parser().parse_args([
            "--run-id", "r", "--output", "r.json",
            "--bridge-config", "b.yaml", *extra])

    def test_default_keeps_the_base_config(self):
        assert self._parse().real_time_factor is None

    def test_a_value(self):
        assert self._parse("--real-time-factor", "0.5").real_time_factor \
            == 0.5

    @pytest.mark.parametrize("value", ["0", "-1", "abc", "nan", "inf"])
    def test_invalid_values_are_refused(self, value):
        with pytest.raises(SystemExit):
            self._parse("--real-time-factor", value)


class TestBridgeConfig:
    def test_default_keeps_the_yaml_value(self, tmp_path):
        settings = _settings(tmp_path, execution_mode="free_run")
        assert settings["time"]["real_time_factor"] == \
            _base()["time"]["real_time_factor"] == 1.0

    def test_the_option_is_written(self, tmp_path):
        settings = _settings(tmp_path, execution_mode="free_run",
                             real_time_factor=0.5)
        assert settings["time"]["real_time_factor"] == 0.5

    def test_free_run_turns_b_and_c_off(self, tmp_path):
        sensing = _settings(tmp_path, execution_mode="free_run")["sensing"]
        assert sensing["combine_attitude_rate"] is False
        assert sensing["orbit_batch_samples"] == 1
        assert sensing["sensing_mode"] == "separate"

    def test_free_run_names_the_pacing_file(self, tmp_path):
        settings = _settings(tmp_path, execution_mode="free_run")
        assert settings["free_run_pacing_file"] == os.path.join(
            os.path.abspath(str(tmp_path / "run.logs")),
            "free_run_pacing.json")

    def test_lockstep_sensing_and_time_are_unchanged(self, tmp_path):
        base = _base()
        settings = _settings(tmp_path, execution_mode="lockstep")
        assert settings["time"] == base["time"]
        assert settings["sensing"]["combine_attitude_rate"] is True
        assert settings["sensing"]["orbit_batch_samples"] == \
            base["sensing"]["orbit_batch_samples"]
        assert "free_run_pacing_file" not in settings


def _run(tmp_path, mode, rtf=None, pacing=PACING):
    config = _config(tmp_path, bridge_config=ACAV_YAML, execution_mode=mode,
                     real_time_factor=rtf)
    logs = tmp_path / "run.logs"

    def popen(argv, **kwargs):
        if "acav.scheduler_node" in argv:
            write_record(config.output, RunRecord(
                result=RunResult(run_id=config.run_id,
                                 scheduler="dynamics_aware"),
                provenance=Provenance(backend_type="com",
                                      execution_mode=mode)))
            (logs / "startup_gate.json").write_text(json.dumps(
                {"opened": True, "wait_s": 1.0}), encoding="utf-8")
            if pacing is not None:
                (logs / "free_run_pacing.json").write_text(
                    json.dumps(pacing), encoding="utf-8")
        return FakeProcess()

    deps = Deps(popen=popen,
                wait_for_bridge_connected=lambda *a: ConnectResult(True,
                                                                   "com"),
                wait_for_twin_ack=lambda *a: True, prompt=lambda m: None)
    code = execute_run(config, deps)
    summary = json.loads((logs / "summary.json").read_text(encoding="utf-8"))
    return code, summary, read_record(config.output)


class TestRecorded:
    def test_free_run_at_half_speed(self, tmp_path):
        code, summary, record = _run(tmp_path, "free_run", rtf=0.5)
        assert code == 0
        assert summary["real_time_factor"] == 0.5
        assert record.provenance.real_time_factor == 0.5
        assert summary["sensing_mode"] == "separate"
        assert summary["orbit_batch_samples"] == 1
        assert summary["free_run_pacing"] == PACING

    def test_free_run_without_a_pacing_file(self, tmp_path):
        _, summary, _ = _run(tmp_path, "free_run", pacing=None)
        assert summary["free_run_pacing"] is None

    def test_lockstep(self, tmp_path):
        code, summary, record = _run(tmp_path, "lockstep")
        assert code == 0
        assert summary["real_time_factor"] == 1.0
        assert record.provenance.real_time_factor == 1.0
        assert summary["sensing_mode"] == "combined_attitude_rate"
        assert "free_run_pacing" not in summary

    def test_provenance_round_trip_and_old_records(self):
        assert Provenance.from_dict(Provenance(
            real_time_factor=0.25).to_dict()).real_time_factor == 0.25
        assert Provenance.from_dict({}).real_time_factor is None

    def test_not_part_of_the_signature(self):
        result = RunResult(run_id="r", scheduler="dynamics_aware")
        assert RunRecord(result, Provenance(real_time_factor=0.5)).signature \
            == RunRecord(result, Provenance()).signature


def _directory(tmp_path, runs):
    """runs: (mode, recorded rtf or None, bridge.log text or None)."""
    for seed, (mode, rtf, log) in enumerate(runs):
        name = f"E6-dynamics_aware-lockstep-seed{seed}"
        write_record(str(tmp_path / f"{name}.json"), covering(RunRecord(
            result=_result(name, seed=seed),
            provenance=Provenance(backend_type="com", execution_mode=mode,
                                  real_time_factor=rtf))))
        logs = tmp_path / f"{name}.logs"
        logs.mkdir()
        (logs / "summary.json").write_text(json.dumps({"run_id": name}),
                                           encoding="utf-8")
        if log is not None:
            (logs / "bridge.log").write_text(log, encoding="utf-8")


RTF1_LOG = ("2026-10-15 15:24:36,830 INFO    stk_bridge.core.time_sync: time "
            "sync started: mode=FREE_RUN epoch=1717237800.000 step=0.1000s "
            "rtf=1.000 participants=['acav_scheduler', 'matlab_twin']\n")


def _report(tmp_path, runs):
    return main(["report", "--experiment", "E6", "--repeats", str(runs),
                 "--no-figures", "--output-dir", str(tmp_path)])


class TestReport:
    def test_one_value(self, tmp_path):
        _directory(tmp_path, [("lockstep", 1.0, None)] * 2)
        assert _report(tmp_path, 2) == 0

    def test_different_values_are_refused(self, tmp_path, caplog):
        _directory(tmp_path, [("free_run", 1.0, None),
                              ("free_run", 0.5, None)])
        assert _report(tmp_path, 2) == 1
        assert "real-time factor" in caplog.text

    def test_old_lockstep_records_count_as_one(self, tmp_path):
        _directory(tmp_path, [("lockstep", None, None),
                              ("lockstep", 1.0, None)])
        assert effective_real_time_factors(str(tmp_path)) == {
            1.0: ["E6-dynamics_aware-lockstep-seed0",
                  "E6-dynamics_aware-lockstep-seed1"]}
        assert _report(tmp_path, 2) == 0

    def test_old_free_run_record_proven_by_its_log(self, tmp_path):
        _directory(tmp_path, [("free_run", None, RTF1_LOG),
                              ("free_run", 1.0, None)])
        assert _report(tmp_path, 2) == 0

    def test_old_free_run_record_without_proof_is_refused(self, tmp_path,
                                                          caplog):
        _directory(tmp_path, [("free_run", None, None),
                              ("free_run", 1.0, None)])
        assert _report(tmp_path, 2) == 1
        assert "unknown" in caplog.text
