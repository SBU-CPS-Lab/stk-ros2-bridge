"""A run made with --diagnostic-label is never pooled by `report`.

For one-off measurements (e.g. STK graphics off) that must not be
mixed with ordinary E6 runs. The label goes into the run's summary.json;
`report` refuses a directory that contains any labelled run.
"""

import json

import pytest

import acav.deploy_run_cli as deploy_run_cli
from acav.campaign_cli import labelled_runs, main
from acav.deploy_run_cli import ConnectResult, Deps, execute_run
from acav.run_record import write_record

from test_campaign import _record             # noqa: I100 - shared helper
from test_deploy_run_cli import FakeProcess, _config  # noqa: I100


@pytest.fixture(autouse=True)
def offline(monkeypatch):
    from types import SimpleNamespace
    import acav.topic_recorder
    monkeypatch.setattr(deploy_run_cli, "_check_graph_idle", lambda p: None)
    monkeypatch.setattr(acav.topic_recorder, "TopicRecorder",
                        lambda *a: SimpleNamespace(close=lambda: None,
                                                   error=None))


def _summary(tmp_path, **overrides):
    deps = Deps(popen=lambda *a, **k: FakeProcess(),
                wait_for_bridge_connected=lambda *a: ConnectResult(True, "com"),
                wait_for_twin_ack=lambda *a: True, prompt=lambda m: None)
    assert execute_run(_config(tmp_path, **overrides), deps) == 0
    return json.loads((tmp_path / "run.logs" / "summary.json")
                      .read_text(encoding="utf-8"))


def test_the_label_is_recorded(tmp_path):
    assert _summary(tmp_path, diagnostic_label="graphics-off")[
        "diagnostic_label"] == "graphics-off"


def test_no_label_is_recorded_as_empty(tmp_path):
    assert _summary(tmp_path)["diagnostic_label"] == ""


def test_the_cli_option(tmp_path):
    args = deploy_run_cli._parser().parse_args([
        "--run-id", "r", "--output", "r.json", "--bridge-config", "b.yaml",
        "--diagnostic-label", "graphics-off"])
    assert args.diagnostic_label == "graphics-off"


@pytest.mark.parametrize("label", ["has space", "x" * 65, "a/b"])
def test_an_unsafe_label_is_an_invocation_error(label):
    with pytest.raises(SystemExit):
        deploy_run_cli.main(["--run-id", "r", "--output", "r.json",
                             "--bridge-config", "b.yaml",
                             "--diagnostic-label", label])


def _directory(tmp_path, label):
    name = "E6-dynamics_aware-lockstep-seed0"
    write_record(str(tmp_path / f"{name}.json"), _record(name, seed=0))
    logs = tmp_path / f"{name}.logs"
    logs.mkdir()
    (logs / "summary.json").write_text(json.dumps(
        {"run_id": name, "diagnostic_label": label}), encoding="utf-8")


def _report(tmp_path):
    return main(["report", "--experiment", "E6", "--repeats", "1",
                 "--no-figures", "--output-dir", str(tmp_path)])


def test_report_refuses_a_labelled_run(tmp_path, caplog):
    _directory(tmp_path, "graphics-off")
    assert labelled_runs(str(tmp_path)) == [
        ("E6-dynamics_aware-lockstep-seed0", "graphics-off")]
    assert _report(tmp_path) == 1
    assert "graphics-off" in caplog.text


def test_report_accepts_an_unlabelled_run(tmp_path):
    _directory(tmp_path, "")
    assert labelled_runs(str(tmp_path)) == []
    assert _report(tmp_path) == 0
