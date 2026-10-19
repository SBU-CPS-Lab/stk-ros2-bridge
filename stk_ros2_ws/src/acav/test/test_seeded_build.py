"""Building seeded scenarios one after another, and running E2 over them.

E2 final runs, 2026-10-19:
1. build_acav_scenario into an STK that already holds ACAV_SSS1 failed
   ("NewScenario ... Already exists"): every scenario, reference and
   seeded, is named ACAV_SSS1. The builder now closes an open scenario
   (logged, and verified closed) before creating the new one; it never
   starts STK.
2. A failed build went unnoticed by the shell loop, and seed 1's campaign
   ran against the still-open reference scenario. run-seeds stops a seed
   on any non-zero build result.
3. A successful build left 2D/3D windows open and STK maximised, which
   the run's graphics guard refuses. After build and verify, the build
   closes them and minimises STK, reads the state back and fails unless
   it is "off" (--leave-graphics skips this).
Also the resume code-version guard checked seeded records against the
reference scenario, so it skipped them (a spurious "does not match"
warning, and resuming over other code was not refused).

Mock backend and fakes only.
"""

import pytest

from acav.scenario_builder import AcavScenarioBuilder
from acav.spec import AcavSpec
from stk_bridge.backends.mock_backend import MockBackend

class Spy(MockBackend):
    def __init__(self, keep_open=False):
        super().__init__()
        self.closed = 0
        self.keep_open = keep_open

    def close_scenario(self):
        self.closed += 1
        if not self.keep_open:
            super().close_scenario()


def _spy(open_first, keep_open=False):
    backend = Spy(keep_open)
    backend.connect()
    if open_first:
        backend.new_scenario("ACAV_SSS1")
    return backend


def test_an_open_scenario_is_closed_then_built():
    backend = _spy(open_first=True)
    report = AcavScenarioBuilder(backend, AcavSpec.load()).build()
    assert backend.closed == 1
    assert report.ok, report.errors
    assert any("closed the open scenario 'ACAV_SSS1'" in w
               for w in report.warnings)


def test_with_no_scenario_open_nothing_is_closed():
    backend = _spy(open_first=False)
    report = AcavScenarioBuilder(backend, AcavSpec.load()).build()
    assert backend.closed == 0
    assert report.ok, report.errors


def test_a_scenario_that_will_not_close_fails_the_build():
    backend = _spy(open_first=True, keep_open=True)
    report = AcavScenarioBuilder(backend, AcavSpec.load()).build()
    assert not report.ok
    assert any("still open after CloseScenario" in e for e in report.errors)


# --- graphics left 'off' ------------------------------------------------------

class Desktop(MockBackend):
    """A mock with STK Desktop's graphics state."""

    def __init__(self, obeys=True):
        super().__init__()
        self.state = {"windows_2d": 1, "windows_3d": 1,
                      "main_window": "maximized", "visible": True}
        self.obeys = obeys
        self.set_calls = 0

    def graphics_state(self):
        return dict(self.state)

    def set_graphics_off(self):
        self.set_calls += 1
        if self.obeys:
            self.state.update(windows_2d=0, windows_3d=0,
                              main_window="minimized")


def _build(monkeypatch, backend, extra=()):
    from acav import build_cli
    monkeypatch.setattr(build_cli, "create_backend", lambda config: backend)
    return build_cli.main(["--backend", "com", "--verify"] + list(extra))


def test_the_build_leaves_graphics_off(monkeypatch):
    backend = Desktop()
    assert _build(monkeypatch, backend) == 0
    assert backend.set_calls == 1
    assert backend.state["main_window"] == "minimized"


def test_graphics_that_stay_on_fail_the_build(monkeypatch, capsys):
    backend = Desktop(obeys=False)
    assert _build(monkeypatch, backend) == 1
    assert "STK graphics is not off after the build" in \
        capsys.readouterr().err


def test_leave_graphics_skips_it(monkeypatch):
    backend = Desktop(obeys=False)
    assert _build(monkeypatch, backend, ["--leave-graphics"]) == 0
    assert backend.set_calls == 0


# --- run-seeds --------------------------------------------------------------------

class Calls:
    def __init__(self, failing_builds=(), complete=(), failing_runs=()):
        self.log = []
        self.failing_builds = set(failing_builds)
        self.complete = set(complete)
        self.failing_runs = set(failing_runs)

    def build(self, scenario):
        name = "reference" if not scenario else scenario.rsplit("seed", 1)[
            1].split(".")[0]
        self.log.append(("build", name))
        return 1 if name in self.failing_builds else 0

    def campaign(self, argv):
        verb = argv[0]
        out = argv[argv.index("--output-dir") + 1]
        self.log.append((verb, out.rsplit("\\", 1)[-1].rsplit("/", 1)[-1]))
        if verb == "run" and any(f"seed{s}_" in out
                                 for s in self.failing_runs):
            return 1
        return 0

    def is_complete(self, out, scenario):
        return any(f"seed{s}_" in out for s in self.complete)


def _run_seeds(tmp_path, calls, seeds="1-3"):
    from acav.e2_seeds import run_seeds
    return run_seeds(seeds="1-3" if seeds is None else seeds, tag="t1",
                     results_dir=str(tmp_path), backend="com",
                     twin_launch="matlab-batch", launch=["python", "x.py"],
                     build=calls.build, campaign=calls.campaign,
                     is_complete=calls.is_complete)


def test_a_failed_build_stops_that_seed_only(tmp_path):
    calls = Calls(failing_builds={"2"})
    code = _run_seeds(tmp_path, calls)
    assert code != 0
    assert ("run", "E2_final_seed2_t1") not in calls.log
    assert ("run", "E2_final_seed1_t1") in calls.log
    assert ("run", "E2_final_seed3_t1") in calls.log


def test_a_complete_seed_is_neither_built_nor_run(tmp_path):
    calls = Calls(complete={"1"})
    assert _run_seeds(tmp_path, calls) == 0
    assert ("build", "1") not in calls.log
    assert ("run", "E2_final_seed1_t1") not in calls.log


def test_the_reference_is_rebuilt_before_the_reports(tmp_path):
    calls = Calls()
    assert _run_seeds(tmp_path, calls) == 0
    rebuilt = calls.log.index(("build", "reference"))
    reports = [i for i, c in enumerate(calls.log) if c[0] == "report"]
    assert len(reports) == 2 and min(reports) > rebuilt
    assert ("run", "E2_final_ref_t1") in calls.log
    assert calls.log.index(("run", "E2_final_ref_t1")) > rebuilt


def test_a_failed_run_makes_the_exit_non_zero(tmp_path):
    assert _run_seeds(tmp_path, Calls(failing_runs={"3"})) != 0


def test_the_verb_parses():
    from acav.campaign_cli import _parser
    args = _parser().parse_args(["run-seeds", "--experiment", "E2",
                                 "--output-dir", "results", "--seeds",
                                 "1-10", "--commit-tag", "abc1234"])
    assert (args.verb, args.seeds_range, args.commit_tag) == (
        "run-seeds", "1-10", "abc1234")


# --- resume across code versions on a seeded scenario -----------------------------

def test_resume_refuses_seeded_records_from_other_code(tmp_path):
    import os

    from acav.campaign import build_e2_scheduler_comparison
    from acav.campaign_runner import (CampaignRefused, RunnerOptions,
                                      run_campaign)
    from acav.run_record import write_record
    from test_paired_report import GENERATED, _record

    scenario = os.path.join(GENERATED, "acav_seed1.yaml")
    campaign = build_e2_scheduler_comparison(scenario, "com")
    old, new = "a" * 40, "b" * 40          # two clean commits
    for run in campaign.runs:
        record = _record(1, run.scheduler, 10, code_version=old)
        write_record(str(tmp_path / run.output_name), record)
    with pytest.raises(CampaignRefused, match="completed run"):
        run_campaign(campaign, lambda *a: None, RunnerOptions(
            output_dir=str(tmp_path), code_version=new,
            tree_of=lambda version: version))


# --- seeded scenario files from the workspace, not the installed package -----------
# Live check at eb69251: through `ros2 run` the package lives under
# build\acav, where generated\acav_seed*.yaml does not exist, and
# run-seeds failed before building anything.

FIXED = pytest.mark.xfail(strict=True, reason="not fixed yet")


def _workspace(tmp_path, seeds):
    """A workspace whose src tree holds the seeded scenarios; the
    package itself is elsewhere (the installed layout)."""
    import os
    import shutil

    from test_paired_report import GENERATED
    generated = tmp_path / "ws" / "src" / "acav" / "config" / "scenarios" \
        / "generated"
    generated.mkdir(parents=True)
    for seed in seeds:
        shutil.copy(os.path.join(GENERATED, f"acav_seed{seed}.yaml"),
                    generated)
    return tmp_path / "ws", generated


class Recorder(Calls):
    def __init__(self):
        super().__init__()
        self.built = []
        self.runs = []

    def build(self, scenario):
        self.built.append(scenario)
        return super().build(scenario)

    def campaign(self, argv):
        if argv[0] == "run":
            self.runs.append(argv)
        return super().campaign(argv)


@FIXED
def test_seeds_resolve_under_the_workspace_src_tree(tmp_path, monkeypatch):
    import os

    from acav.e2_seeds import run_seeds
    workspace, generated = _workspace(tmp_path, (1, 2))
    monkeypatch.chdir(workspace)
    calls = Recorder()
    assert run_seeds(seeds="1-2", tag="t1", results_dir="results",
                     backend="com", twin_launch="matlab-batch",
                     launch=["python", "x.py"], build=calls.build,
                     campaign=calls.campaign) == 0
    expected = [str(generated / f"acav_seed{k}.yaml") for k in (1, 2)]
    assert calls.built[:2] == expected
    for argv, path in zip(calls.runs, expected):
        assert os.path.isabs(argv[argv.index("--scenario") + 1])
        assert argv[argv.index("--scenario") + 1] == path


@FIXED
def test_missing_seed_files_fail_before_any_build(tmp_path, capsys):
    from acav.e2_seeds import run_seeds
    _, generated = _workspace(tmp_path, (1,))
    calls = Recorder()
    code = run_seeds(seeds="1-3", tag="t1", results_dir=str(tmp_path),
                     backend="com", twin_launch="matlab-batch",
                     launch=["python", "x.py"], build=calls.build,
                     campaign=calls.campaign, scenario_dir=str(generated))
    assert code != 0 and calls.log == []
    err = capsys.readouterr().err
    assert "acav_seed2.yaml" in err and "acav_seed3.yaml" in err


@FIXED
def test_the_verb_takes_a_scenario_dir():
    from acav.campaign_cli import _parser
    args = _parser().parse_args(["run-seeds", "--experiment", "E2",
                                 "--output-dir", "r", "--scenario-dir", "d"])
    assert args.scenario_dir == "d"
