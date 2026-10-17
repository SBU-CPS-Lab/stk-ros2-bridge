"""Which STK a run used, who started it, and what the runner does about it.

P10 live session (3186642): build_acav_scenario started STK through COM
(AgUiApplication.exe -Embedding, parent svchost) because none was
running. E6 on it stepped at 95.7 ms against 60.0-61.1 ms on the P9 runs,
and a free-run bridge's validation on it ran past 90 s. Final runs use an
STK the operator started, by protocol, so:

  * build/verify attach only; --start-stk starts one, with a warning;
  * every COM run records stk_process (pid, start time, command line,
    parent, uptime, started_by "operator" | "com" | "unknown") in
    summary.json and in the record's provenance;
  * the runner refuses a COM run on an STK started through COM, or one
    it cannot identify, unless --allow-com-started-stk;
  * report refuses to pool records whose STK was started differently
    (including records that do not say).
"""

import json
from types import SimpleNamespace

import pytest

from acav import stk_process
from acav.deploy_run_cli import ConnectResult, Deps, _parser, execute_run
from acav.metrics import RunResult
from acav.run_record import Provenance, RunRecord, read_record, write_record

from test_deploy_run_cli import FakeProcess, _config  # noqa: I100

OPERATOR = {"pid": 7100, "parent_pid": 5000, "parent_name": "explorer.exe",
            "command_line": '"C:\\Program Files\\AGI\\STK 12\\bin\\'
                            'AgUiApplication.exe"',
            "started_at": "2026-10-17T09:40:00", "uptime_s": 900.0}
COM = {"pid": 4820, "parent_pid": 708, "parent_name": "svchost.exe",
       "command_line": '"C:\\Program Files\\AGI\\STK 12\\bin\\'
                       'AgUiApplication.exe"  /pers "STK" /ProgID '
                       '"STK12.Application" -Embedding',
       "started_at": "2026-10-17T09:27:06", "uptime_s": 4409.1}


# --- the probe ---------------------------------------------------------------

def _ran(stdout, returncode=0):
    def run(argv, **kwargs):
        assert argv[0] == "powershell"
        return SimpleNamespace(stdout=stdout, stderr="",
                               returncode=returncode)
    return run


def test_the_probe_reads_a_list_and_classifies_each_process():
    found = stk_process.stk_processes(run=_ran(json.dumps([OPERATOR, COM])))
    assert [p["pid"] for p in found] == [7100, 4820]
    assert [p["started_by"] for p in found] == ["operator", "com"]


def test_the_probe_reads_a_single_object_too():
    found = stk_process.stk_processes(run=_ran(json.dumps(COM)))
    assert found[0]["started_by"] == "com"


@pytest.mark.parametrize("stdout", ["", "[]", "not json"])
def test_nothing_or_garbage_is_no_process(stdout):
    assert stk_process.stk_processes(run=_ran(stdout)) == []


def test_the_probe_uses_no_double_quotes_powershell_51_would_strip():
    seen = {}

    def run(argv, **kwargs):
        seen["script"] = argv[-1]
        return SimpleNamespace(stdout="[]", stderr="", returncode=0)

    stk_process.stk_processes(run=run)
    assert '"' not in seen["script"]
    assert "AgUiApplication.exe" in seen["script"]


@pytest.mark.parametrize("command_line, expected", [
    (COM["command_line"], True),
    ("AgUiApplication.exe /embedding", True),
    (OPERATOR["command_line"], False),
    ("", False),
])
def test_started_through_com(command_line, expected):
    assert stk_process.started_through_com(command_line) is expected


@pytest.mark.parametrize("found, allowed, ok, started_by, words", [
    ([OPERATOR], False, True, "operator", "operator-started"),
    ([COM], False, False, "com", "started through COM"),
    ([COM], True, True, "com", "allowed by --allow-com-started-stk"),
    ([], False, False, "unknown", "no STK Desktop process"),
    ([OPERATOR, COM], False, False, "unknown", "2 STK Desktop processes"),
    ([OPERATOR, COM], True, True, "unknown", "2 STK Desktop processes"),
])
def test_assess(found, allowed, ok, started_by, words):
    classified = [dict(p, started_by="com" if p is COM else "operator")
                  for p in found]
    record, passed, detail = stk_process.assess(classified, allowed)
    assert passed is ok
    assert record["started_by"] == started_by
    assert words in detail
    if len(found) == 1:
        assert record["pid"] == found[0]["pid"]
    else:
        assert record["candidates"] == [p["pid"] for p in found]


# --- build and verify ----------------------------------------------------------

def test_build_and_verify_attach_only_unless_start_stk(monkeypatch, capsys):
    from acav import build_cli

    seen = []

    class Backend:
        def connect(self):
            pass

    def create(config):
        seen.append(config)
        return Backend()

    monkeypatch.setattr(build_cli, "create_backend", create)
    parser = build_cli.build_parser("t")
    build_cli.open_backend(parser.parse_args([]))
    assert seen[-1]["backend"]["com"]["start_stk"] is False
    build_cli.open_backend(parser.parse_args(["--start-stk"]))
    assert seen[-1]["backend"]["com"]["start_stk"] is True
    assert "--start-stk" in capsys.readouterr().err


def test_verify_shares_the_flag():
    from acav import verify_cli
    import inspect
    assert "build_parser" in inspect.getsource(verify_cli)


# --- the runner ------------------------------------------------------------------

def _deps(found, scheduler=None):
    recorder = SimpleNamespace(close=lambda: None, error=None)
    return Deps(check_idle=lambda prefix: None,
                start_recorder=lambda *a: recorder,
                popen=scheduler or (lambda *a, **k: FakeProcess()),
                wait_for_bridge_connected=lambda *a: ConnectResult(True,
                                                                   "com"),
                wait_for_twin_ack=lambda *a: True,
                prompt=lambda message: None,
                find_stk_processes=lambda: [
                    dict(p, started_by=stk_process.started_by(p))
                    for p in found])


def _summary(tmp_path):
    return json.loads((tmp_path / "run.logs" / "summary.json")
                      .read_text(encoding="utf-8"))


def _step(summary, name):
    return [s for s in summary["steps"] if s["step"] == name]


def test_an_operator_started_stk_passes_and_is_recorded(tmp_path):
    config = _config(tmp_path)
    write_record(config.output, RunRecord(
        result=RunResult(run_id=config.run_id, scheduler="x"),
        provenance=Provenance(backend_type="com")))
    assert execute_run(config, _deps([OPERATOR])) == 0
    summary = _summary(tmp_path)
    step, = _step(summary, "stk_process")
    assert step["ok"] is True
    assert summary["stk_process"]["pid"] == 7100
    assert summary["stk_process"]["started_by"] == "operator"
    assert read_record(config.output).provenance.stk_process[
        "started_by"] == "operator"


def test_a_com_started_stk_is_refused_before_the_twin(tmp_path):
    def no_twin(*args):
        raise AssertionError("the twin must not be waited for")

    deps = _deps([COM])
    deps.wait_for_twin_ack = no_twin
    assert execute_run(_config(tmp_path), deps) == 1
    summary = _summary(tmp_path)
    step, = _step(summary, "stk_process")
    assert step["ok"] is False
    assert "started through COM" in step["detail"]
    assert "--allow-com-started-stk" in step["detail"]
    assert summary["stk_process"]["started_by"] == "com"


def test_a_com_started_stk_runs_when_allowed(tmp_path):
    config = _config(tmp_path, allow_com_started_stk=True)
    assert execute_run(config, _deps([COM])) == 0
    summary = _summary(tmp_path)
    assert _step(summary, "stk_process")[0]["ok"] is True
    assert summary["allow_com_started_stk"] is True


def test_two_stks_are_refused(tmp_path):
    assert execute_run(_config(tmp_path), _deps([OPERATOR, COM])) == 1
    step, = _step(_summary(tmp_path), "stk_process")
    assert "cannot tell which" in step["detail"]


def test_a_mock_run_has_no_stk_to_check(tmp_path):
    config = _config(tmp_path, backend="mock")

    def never():
        raise AssertionError("mock runs have no STK process")

    deps = _deps([])
    deps.find_stk_processes = never
    assert execute_run(config, deps) == 0
    summary = _summary(tmp_path)
    assert _step(summary, "stk_process") == []
    assert summary["stk_process"] is None


def test_the_cli_flag():
    base = ["--run-id", "r", "--output", "o.json", "--bridge-config",
            "b.yaml"]
    assert _parser().parse_args(base).allow_com_started_stk is False
    assert _parser().parse_args(
        base + ["--allow-com-started-stk"]).allow_com_started_stk is True


def test_the_default_finder_is_the_probe(monkeypatch):
    """Deps' default is the module hook (conftest replaces it offline),
    and the hook's real value runs the probe."""
    import acav.deploy_run_cli as deploy_run_cli

    assert Deps().find_stk_processes is deploy_run_cli._find_stk_processes
    monkeypatch.setattr(stk_process, "stk_processes", lambda: ["probed"])
    assert deploy_run_cli._probe_stk_processes() == ["probed"]


# --- provenance and report ----------------------------------------------------

def test_stk_process_round_trips_and_old_records_have_none(tmp_path):
    record = RunRecord(result=RunResult(run_id="r", scheduler="x"),
                       provenance=Provenance(stk_process=dict(
                           COM, started_by="com")))
    write_record(str(tmp_path / "r.json"), record)
    assert read_record(str(tmp_path / "r.json")).provenance.stk_process[
        "pid"] == 4820
    old = record.to_dict()
    old["provenance"].pop("stk_process")
    assert Provenance.from_dict(old["provenance"]).stk_process is None


def _directory(tmp_path, *modes):
    from acav.run_record import write_record as write
    from test_campaign import _result, covering  # noqa: I100

    for seed, mode in enumerate(modes):
        name = f"E6-dynamics_aware-lockstep-seed{seed}"
        process = None if mode is None else {"pid": seed,
                                             "started_by": mode}
        write(str(tmp_path / f"{name}.json"), covering(RunRecord(
            result=_result(name, seed=seed),
            provenance=Provenance(backend_type="com",
                                  execution_mode="lockstep",
                                  graphics_mode="off",
                                  stk_process=process))))
        logs = tmp_path / f"{name}.logs"
        logs.mkdir()
        (logs / "summary.json").write_text(json.dumps({"run_id": name}),
                                           encoding="utf-8")


def _report(tmp_path, runs):
    from acav.campaign_cli import main
    return main(["report", "--experiment", "E6", "--repeats", str(runs),
                 "--no-figures", "--output-dir", str(tmp_path)])


def test_one_start_mode_is_reported(tmp_path):
    _directory(tmp_path, "operator", "operator")
    assert _report(tmp_path, 2) == 0


def test_older_records_alone_still_report(tmp_path):
    _directory(tmp_path, None, None)
    assert _report(tmp_path, 2) == 0


@pytest.mark.parametrize("modes", [("operator", "com"),
                                   ("operator", None),
                                   ("com", None)])
def test_different_start_modes_are_refused(tmp_path, caplog, modes):
    _directory(tmp_path, *modes)
    assert _report(tmp_path, 2) == 1
    assert "STK" in caplog.text and "started" in caplog.text
    assert "E6-dynamics_aware-lockstep-seed1" in caplog.text


def test_stk_start_modes_lists_runs(tmp_path):
    from acav.campaign_cli import load_results, stk_start_modes
    _directory(tmp_path, "operator", "com", None)
    assert stk_start_modes(load_results(str(tmp_path))) == {
        "operator": ["E6-dynamics_aware-lockstep-seed0"],
        "com": ["E6-dynamics_aware-lockstep-seed1"],
        "": ["E6-dynamics_aware-lockstep-seed2"]}
