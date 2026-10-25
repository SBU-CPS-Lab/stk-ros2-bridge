"""Every run requires a graphics mode, checked before the twin prompt.

run_acav_run.py --graphics-mode {off,on}, default off. "off" is no 2D
window, no 3D window and STK minimised -- the condition measured; anything else is "on" (stk_bridge/core/graphics_state.py).
Right after bridge_connected the runner reads the bridge's startup
sample; a state that does not match fails the stk_graphics step before
any handoff time is spent. After the scheduler exits it reads the whole
run's samples: the run's mode ("mixed" if it changed, "unverified" with
no terminal sample) goes to summary.json and is written into the
record's provenance.graphics_mode; a mode other than the required one
fails the stk_graphics_end step but keeps the labelled record.
"""

import json
import os

import pytest

import acav.deploy_run_cli as deploy_run_cli
from acav.deploy_run_cli import ConnectResult, Deps, execute_run
from acav.metrics import RunResult
from acav.run_record import Provenance, RunRecord, read_record, write_record

from test_deploy_run_cli import FakeProcess, _config  # noqa: I100

OFF = {"windows_2d": 0, "windows_3d": 0, "main_window": "minimized",
       "visible": True}
ON = {"windows_2d": 1, "windows_3d": 1, "main_window": "normal",
      "visible": True}


def _entry(state, step):
    from stk_bridge.core.graphics_state import classify
    return {**state, "mode": classify(state), "step": step}


def _data(startup, samples=(), end=None):
    return {"startup": _entry(startup, 0) if startup is not None else None,
            "samples": [_entry(s, 1000 * (i + 1))
                        for i, s in enumerate(samples)],
            "end": _entry(end, 7200) if end else None}


@pytest.fixture(autouse=True)
def offline(monkeypatch):
    from types import SimpleNamespace
    import acav.topic_recorder
    monkeypatch.setattr(deploy_run_cli, "_check_graph_idle", lambda p: None)
    monkeypatch.setattr(acav.topic_recorder, "TopicRecorder",
                        lambda *a: SimpleNamespace(close=lambda: None,
                                                   error=None))


def _run(tmp_path, data, **overrides):
    """One run; the fake scheduler writes a record, like the real one."""
    config = _config(tmp_path, **overrides)
    launched, prompts, reads = [], [], []

    def popen(argv, **kwargs):
        launched.append(argv)
        if "acav.scheduler_node" in argv:
            write_record(config.output, RunRecord(
                result=RunResult(run_id=config.run_id,
                                 scheduler="dynamics_aware"),
                provenance=Provenance(backend_type="com")))
        return FakeProcess()

    def read_graphics_state(path, phase, timeout_s):
        reads.append((path, phase))
        return data

    deps = Deps(popen=popen,
                wait_for_bridge_connected=lambda *a: ConnectResult(True,
                                                                   "com"),
                wait_for_twin_ack=lambda *a: True,
                prompt=prompts.append,
                read_graphics_state=read_graphics_state)
    code = execute_run(config, deps)
    logs = tmp_path / "run.logs"
    summary = json.loads((logs / "summary.json").read_text(encoding="utf-8"))
    steps = {s["step"]: s for s in summary["steps"]}
    return code, launched, prompts, reads, summary, steps, config


class TestOption:
    def _args(self, *extra):
        return deploy_run_cli._parser().parse_args([
            "--run-id", "r", "--output", "r.json",
            "--bridge-config", "b.yaml", *extra])

    def test_default_is_off(self):
        assert self._args().graphics_mode == "off"

    def test_on_must_be_explicit(self):
        assert self._args("--graphics-mode", "on").graphics_mode == "on"

    def test_nothing_else(self):
        with pytest.raises(SystemExit):
            self._args("--graphics-mode", "auto")

    def test_the_bridge_is_told_where_to_write(self, tmp_path):
        import logging
        import yaml
        config = _config(tmp_path)
        log_dir = str(tmp_path / "run.logs")
        os.makedirs(log_dir)
        path = deploy_run_cli.write_bridge_config(config, log_dir,
                                                  logging.getLogger("t"))
        with open(path, encoding="utf-8") as handle:
            settings = yaml.safe_load(handle)
        assert settings["graphics_state_file"] == os.path.join(
            os.path.abspath(log_dir), "stk_graphics.json")


class TestStartup:
    def test_a_mismatch_fails_before_the_twin_prompt(self, tmp_path):
        code, launched, prompts, _, summary, steps, _ = _run(
            tmp_path, _data(ON))
        assert code == 1
        assert prompts == []                       # no handoff asked for
        assert len(launched) == 1                  # the bridge only
        assert steps["stk_graphics"]["ok"] is False
        detail = steps["stk_graphics"]["detail"]
        assert "requires 'off'" in detail
        assert "close every 2D and 3D Graphics window" in detail
        assert "2D windows 1, 3D windows 1, main window normal" in detail
        assert summary["graphics_mode_required"] == "off"
        assert summary["graphics_startup"]["mode"] == "on"

    def test_on_is_accepted_when_required(self, tmp_path):
        code, _, _, _, summary, steps, _ = _run(
            tmp_path, _data(ON, [ON], ON), graphics_mode="on")
        assert code == 0
        assert steps["stk_graphics"]["ok"] is True
        assert summary["graphics_mode"] == "on"

    def test_no_state_reported_fails(self, tmp_path):
        code, _, prompts, _, _, steps, _ = _run(tmp_path, None)
        assert code == 1 and prompts == []
        assert "no STK graphics state" in steps["stk_graphics"]["detail"]

    def test_an_unavailable_state_fails(self, tmp_path):
        code, _, _, _, _, steps, _ = _run(tmp_path, _data({}))
        assert code == 1
        assert "unavailable" in steps["stk_graphics"]["detail"]

    def test_the_bridge_file_is_the_one_read(self, tmp_path):
        _, _, _, reads, _, _, _ = _run(tmp_path, _data(OFF, [OFF], OFF))
        expected = os.path.join(os.path.abspath(str(tmp_path / "run.logs")),
                                "stk_graphics.json")
        assert reads == [(expected, "startup"), (expected, "end")]


class TestWholeRun:
    def test_off_throughout(self, tmp_path):
        code, _, _, _, summary, steps, config = _run(
            tmp_path, _data(OFF, [OFF, OFF], OFF))
        assert code == 0
        assert steps["stk_graphics_end"]["ok"] is True
        assert summary["graphics_mode"] == "off"
        assert len(summary["graphics_samples"]) == 2
        assert read_record(config.output).provenance.graphics_mode == "off"

    def test_a_change_mid_run_is_mixed_failed_and_labelled(self, tmp_path):
        code, _, _, _, summary, steps, config = _run(
            tmp_path, _data(OFF, [OFF, ON], OFF))
        assert code == 1
        assert steps["stk_graphics_end"]["ok"] is False
        assert "mixed" in steps["stk_graphics_end"]["detail"]
        assert summary["graphics_mode"] == "mixed"
        record = read_record(config.output)          # kept
        assert record.provenance.graphics_mode == "mixed"

    def test_no_terminal_sample_is_unverified(self, tmp_path):
        code, _, _, _, summary, _, config = _run(tmp_path,
                                                  _data(OFF, [OFF]))
        assert code == 1
        assert summary["graphics_mode"] == "unverified"
        assert read_record(config.output).provenance.graphics_mode == \
            "unverified"

    def test_labelling_keeps_the_signature_and_everything_else(self, tmp_path):
        _, _, _, _, _, _, config = _run(tmp_path, _data(OFF, [], OFF))
        record = read_record(config.output)
        assert record.provenance.backend_type == "com"
        assert record.result.run_id == config.run_id


class TestProvenance:
    def test_round_trip_and_old_records(self):
        assert Provenance.from_dict(
            Provenance(graphics_mode="off").to_dict()).graphics_mode == "off"
        assert Provenance.from_dict({}).graphics_mode == ""

    def test_not_part_of_the_signature(self):
        result = RunResult(run_id="r", scheduler="dynamics_aware")
        assert RunRecord(result, Provenance(graphics_mode="on")).signature \
            == RunRecord(result, Provenance()).signature
