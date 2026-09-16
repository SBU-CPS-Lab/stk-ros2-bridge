"""Tests for the campaign matrices, aggregation and the runner loop.

A campaign is 40 to 100+ runs, each needing STK and MATLAB, so it is an
overnight thing on one machine and cannot be iterated on. Everything
that can be established before it starts is established here.
"""

import json
import math
import os

import pytest

from acav.campaign import (
    FREE_RUN,
    LOCKSTEP,
    Campaign,
    RunSpec,
    MISPAIRED_ATTITUDE_PROVIDER,
    build_e1_convention,
    build_e2_scheduler_comparison,
    build_e3_determinism,
    build_e4_gain_sweep,
    build_e5_fault_campaign,
    build_e5_replay,
    build_e6_latency,
    compare_schedulers,
    determinism_comparison,
    gain_surface,
    latency_breakdown,
    replay_matches,
)
from acav.campaign_runner import (
    CommandLauncher,
    RunnerOptions,
    check_record,
    run_campaign,
)
from acav.fault_injection import WheelFault
from acav.metrics import CaptureOutcome, RunResult, TargetResult
from acav.run_record import Provenance, RunRecord, write_record

WINDOW = 720.0


def _result(run_id, seed=None, scheduler="dynamics_aware", error=0.2,
            captured=True):
    outcome = (CaptureOutcome.CAPTURED if captured
               else CaptureOutcome.NOT_SETTLED)
    return RunResult(
        run_id=run_id, scheduler=scheduler, seed=seed,
        targets=[TargetResult(target_id="T1", attempted=True,
                              outcome=outcome, image_time_s=120.0,
                              error_at_image_deg=error,
                              max_error_in_hold_deg=error)])


def _record(run_id, *, backend="com", mode=LOCKSTEP, seed=None,
            scheduler="dynamics_aware", error=0.2, captured=True,
            gains=None, fault=None):
    return RunRecord(
        result=_result(run_id, seed, scheduler, error, captured),
        provenance=Provenance(backend_type=backend, execution_mode=mode,
                              gains=gains or {}, fault=fault))


class TestRunIdentity:
    def test_the_id_is_derived_from_the_configuration(self):
        run = RunSpec(experiment="E4", gains={
            "natural_frequency_rad_s": 1.0, "profile_authority": 0.8})
        assert run.run_id == "E4-dynamics_aware-lockstep-wn1-auth0p8"

    def test_gain_tags_stay_filename_safe_and_distinct(self):
        ids = {RunSpec(experiment="E4",
                       gains={"natural_frequency_rad_s": value}).run_id
               for value in (0.4, 0.75, 1.0, 1.25, 1.6)}
        assert len(ids) == 5
        assert all(not set(i) & set(r'/\ .:*?"<>|') for i in ids)

    def test_the_id_is_stable_when_other_runs_are_inserted(self):
        """Positional ids renumber on insertion and silently repoint
        every result file an earlier campaign wrote. They are also what
        makes resume possible at all."""
        one = build_e3_determinism(repeats=2)
        two = build_e3_determinism(repeats=3)
        shared = {r.run_id for r in one.runs} & {r.run_id for r in two.runs}
        assert len(shared) == len(one.runs)

    def test_duplicate_run_ids_are_refused(self):
        run = RunSpec(experiment="E2")
        with pytest.raises(ValueError, match="duplicate run id"):
            Campaign("c", "E2", "", runs=(run, run))

    def test_an_unknown_gain_is_refused(self):
        """A gain the twin does not have would be swept over with no
        effect, producing a flat surface that looks like a finding."""
        with pytest.raises(ValueError, match="unknown gain"):
            RunSpec(experiment="E4", gains={"stiffness": 1.0})


class TestMatrices:
    def test_e2_pairs_the_two_schedulers(self):
        campaign = build_e2_scheduler_comparison()
        assert {r.scheduler for r in campaign.runs} == {
            "kinematic", "dynamics_aware"}

    def test_e3_varies_only_the_execution_mode(self):
        campaign = build_e3_determinism(repeats=20)
        assert len(campaign) == 40
        assert sum(1 for r in campaign.runs if r.execution_mode == LOCKSTEP) \
            == 20
        assert sum(1 for r in campaign.runs if r.execution_mode == FREE_RUN) \
            == 20
        # Same seeds on both sides: if a free-run batch does come out
        # identical it must be because the loop was deterministic, not
        # because nothing in it varied.
        lockstep = sorted(r.seed for r in campaign.runs
                          if r.execution_mode == LOCKSTEP)
        free = sorted(r.seed for r in campaign.runs
                      if r.execution_mode == FREE_RUN)
        assert lockstep == free

    def test_e3_refuses_a_single_repeat(self):
        with pytest.raises(ValueError, match="spread"):
            build_e3_determinism(repeats=1)

    def test_e4_defaults_to_the_forty_nine_run_grid(self):
        campaign = build_e4_gain_sweep()
        assert len(campaign) == 49
        assert len({r.run_id for r in campaign.runs}) == 49

    def test_e5_resolves_every_fault_before_anything_runs(self):
        campaign = build_e5_fault_campaign([1, 2, 3], WINDOW)
        assert all(r.fault is not None for r in campaign.runs)
        assert {r.fault.seed for r in campaign.runs} == {1, 2, 3}

    def test_e5_needs_seeds(self):
        with pytest.raises(ValueError, match="seed"):
            build_e5_fault_campaign([], WINDOW)


class TestE1Convention:
    """E1 is expressible now that the bridge can be told to mis-pair."""

    def test_it_is_a_control_and_a_treatment(self):
        campaign = build_e1_convention()
        providers = [r.attitude_provider for r in campaign.runs]
        assert providers == ["", MISPAIRED_ATTITUDE_PROVIDER]

    def test_only_the_read_back_provider_differs(self):
        """Anything else varying between the two would confound the
        measurement with whatever else changed."""
        control, treatment = build_e1_convention().runs
        assert control.scheduler == treatment.scheduler
        assert control.execution_mode == treatment.execution_mode
        assert control.seed == treatment.seed
        assert control.gains == treatment.gains

    def test_the_two_runs_write_to_different_files(self):
        control, treatment = build_e1_convention().runs
        assert control.output_name != treatment.output_name

    def test_the_treatment_names_itself_in_its_run_id(self):
        _, treatment = build_e1_convention().runs
        assert "readback" in treatment.run_id


class TestReplayMatrix:
    def test_replay_carries_the_recorded_fault_not_the_rule(self):
        """If the replay re-derived the fault from the seed, a change to
        the sampling rule would turn a genuine reproduction into a
        silent mismatch for every historical result."""
        recorded = {"axis": 2, "at_s": 61.5, "health": 0.0, "seed": 4}
        original = _record("E5-seed4", seed=4, fault=recorded)

        replay = build_e5_replay([original])
        assert len(replay) == 1
        assert replay.runs[0].fault == WheelFault(axis=2, at_s=61.5, seed=4)
        assert replay.runs[0].replay_of == "E5-seed4"

    def test_replaying_nothing_is_an_error(self):
        with pytest.raises(ValueError, match="nothing to replay"):
            build_e5_replay([])

    def test_matching_signatures_count_as_reproduction(self):
        fault = {"axis": 0, "at_s": 10.0, "health": 0.0, "seed": 1}
        original = _record("a", fault=fault)
        replay = _record("a-replay", fault=fault)
        assert replay_matches(original, replay).reproduced

    def test_a_differing_signature_is_not_reproduction(self):
        fault = {"axis": 0, "at_s": 10.0, "health": 0.0, "seed": 1}
        original = _record("a", fault=fault, error=0.2)
        replay = _record("a-replay", fault=fault, error=0.9)
        check = replay_matches(original, replay)
        assert not check.reproduced
        assert "signatures differ" in check.detail

    def test_a_replay_of_a_different_fault_is_not_reproduction(self):
        """Even when the signatures agree. A match there says the fault
        made no difference, not that the run was reproduced."""
        original = _record("a", fault={"axis": 0, "at_s": 10.0,
                                       "health": 0.0, "seed": 1})
        replay = _record("a-replay", fault={"axis": 1, "at_s": 500.0,
                                            "health": 0.0, "seed": 1})
        check = replay_matches(original, replay)
        assert not check.reproduced
        assert "different fault" in check.detail


class TestAggregation:
    def test_determinism_is_grouped_by_the_observed_mode(self):
        """A run that asked for lockstep and fell back to free-run
        belongs in the free-run group. Grouping by what was requested
        would report determinism for a batch that was not all
        lockstep."""
        records = [
            _record("a", mode=LOCKSTEP, error=0.2),
            _record("b", mode=LOCKSTEP, error=0.2),
            _record("c", mode=FREE_RUN, error=0.3),
            _record("d", mode=FREE_RUN, error=0.4),
        ]
        report = determinism_comparison(records)

        assert report[LOCKSTEP]["deterministic"] is True
        assert report[LOCKSTEP]["distinct_signatures"] == 1
        assert report[FREE_RUN]["deterministic"] is False
        assert report[FREE_RUN]["distinct_signatures"] == 2

    def test_the_gain_surface_is_sorted_for_plotting(self):
        records = [
            _record("hi", gains={"natural_frequency_rad_s": 1.4}),
            _record("lo", gains={"natural_frequency_rad_s": 0.4}),
        ]
        surface = gain_surface(records, total_targets=7)
        assert [p["run_id"] for p in surface] == ["lo", "hi"]
        assert surface[0]["capture_rate"] == pytest.approx(1.0)

    def test_scheduler_comparison_reports_both_rates(self):
        """Capture rate alone flatters a scheduler that attempts one
        easy target, so coverage is always reported beside it."""
        records = [
            _record("k", scheduler="kinematic", captured=False),
            _record("d", scheduler="dynamics_aware", captured=True),
        ]
        summary = compare_schedulers(records, total_targets=7)

        assert summary["kinematic"]["capture_rate"] == pytest.approx(0.0)
        assert summary["dynamics_aware"]["capture_rate"] == pytest.approx(1.0)
        assert summary["kinematic"]["coverage_rate"] == pytest.approx(0.0)
        assert summary["kinematic"]["over_promise_gap"] == pytest.approx(1.0)
        assert "coverage_difference" in summary


# ---------------------------------------------------------------------------
# The runner loop
# ---------------------------------------------------------------------------

def _writing_launcher(**overrides):
    """A launcher that writes a plausible record for the run it is given."""
    def launch(run, campaign, output_path):
        result = RunResult(run_id=overrides.get("run_id", run.run_id),
                           scheduler=overrides.get("scheduler",
                                                   run.scheduler),
                           seed=overrides.get("seed", run.seed),
                           targets=[TargetResult(
                               target_id="T1", attempted=True,
                               outcome=CaptureOutcome.CAPTURED,
                               image_time_s=1.0, error_at_image_deg=0.1,
                               max_error_in_hold_deg=0.1)])
        provenance = Provenance(
            backend_type="com", execution_mode=run.execution_mode,
            gains=dict(run.gains),
            fault=None if run.fault is None else run.fault.to_dict())
        write_record(output_path,
                     RunRecord(result=result, provenance=provenance,
                               experiment=run.experiment))
    return launch


class TestRunnerLoop:
    def _campaign(self):
        return Campaign("c", "E2", "", runs=(
            RunSpec(experiment="E2", scheduler="kinematic"),
            RunSpec(experiment="E2", scheduler="dynamics_aware")))

    def test_a_completed_campaign_collects_every_record(self, tmp_path):
        outcome = run_campaign(self._campaign(), _writing_launcher(),
                               RunnerOptions(output_dir=str(tmp_path)))
        assert outcome.complete
        assert len(outcome.records) == 2
        assert not outcome.failed

    def test_a_manifest_is_written_beside_the_results(self, tmp_path):
        run_campaign(self._campaign(), _writing_launcher(),
                     RunnerOptions(output_dir=str(tmp_path)))
        assert os.path.exists(tmp_path / "campaign.json")

    def test_resume_skips_runs_already_on_disk(self, tmp_path):
        launched = []

        def counting(run, campaign, path):
            launched.append(run.run_id)
            _writing_launcher()(run, campaign, path)

        options = RunnerOptions(output_dir=str(tmp_path))
        run_campaign(self._campaign(), counting, options)
        assert len(launched) == 2

        second = run_campaign(self._campaign(), counting, options)
        assert len(launched) == 2            # nothing re-launched
        assert len(second.records) == 2      # but everything collected

    def test_a_launcher_that_writes_nothing_is_a_failure(self, tmp_path):
        """Not an empty result. Scoring a launch problem as a run that
        captured nothing would put infrastructure faults into the
        study's headline metric."""
        outcome = run_campaign(self._campaign(), lambda *_: None,
                               RunnerOptions(output_dir=str(tmp_path)))
        assert not outcome.complete
        assert outcome.records == []
        assert all(o.status == "failed" for o in outcome.outcomes)
        assert "wrote no record" in outcome.outcomes[0].detail

    def test_a_raising_launcher_costs_one_run_not_the_campaign(self, tmp_path):
        def flaky(run, campaign, path):
            if run.scheduler == "kinematic":
                raise RuntimeError("STK did not start")
            _writing_launcher()(run, campaign, path)

        outcome = run_campaign(self._campaign(), flaky,
                               RunnerOptions(output_dir=str(tmp_path)))
        assert len(outcome.records) == 1
        assert len(outcome.failed) == 1

    def test_stop_on_error_halts_the_campaign(self, tmp_path):
        outcome = run_campaign(
            self._campaign(), lambda *_: (_ for _ in ()).throw(
                RuntimeError("boom")),
            RunnerOptions(output_dir=str(tmp_path), stop_on_error=True))
        assert len(outcome.outcomes) == 1

    def test_a_dry_run_launches_nothing_but_writes_the_manifest(self,
                                                                tmp_path):
        launched = []
        outcome = run_campaign(
            self._campaign(), lambda *a: launched.append(a),
            RunnerOptions(output_dir=str(tmp_path), dry_run=True))
        assert launched == []
        assert os.path.exists(tmp_path / "campaign.json")
        assert all(o.status == "skipped" for o in outcome.outcomes)


class TestRecordValidation:
    def test_a_record_with_the_wrong_seed_is_not_merged(self, tmp_path):
        """The specific harness fault that would make E5 report
        reproduction of something other than what it replayed."""
        campaign = Campaign("c", "E5", "", runs=(
            RunSpec(experiment="E5", seed=4,
                    fault=WheelFault(axis=0, at_s=10.0, seed=4)),))

        outcome = run_campaign(campaign, _writing_launcher(seed=99),
                               RunnerOptions(output_dir=str(tmp_path)))
        assert outcome.records == []
        assert "seed" in outcome.failed[0].mismatches[0]

    def test_a_mismatched_record_on_disk_is_not_resumed_over(self, tmp_path):
        campaign = Campaign("c", "E5", "", runs=(
            RunSpec(experiment="E5", seed=4),))
        options = RunnerOptions(output_dir=str(tmp_path))

        run_campaign(campaign, _writing_launcher(seed=99), options)
        relaunched = []

        def counting(run, c, path):
            relaunched.append(run.run_id)
            _writing_launcher()(run, c, path)

        run_campaign(campaign, counting, options)
        assert relaunched            # re-run rather than trusted

    def test_a_matching_record_reports_no_mismatches(self):
        run = RunSpec(experiment="E4", seed=1,
                      gains={"natural_frequency_rad_s": 1.0})
        record = _record(run.run_id, seed=1,
                         gains={"natural_frequency_rad_s": 1.0})
        assert check_record(run, record) == []

    def test_a_run_that_forgot_to_mis_pair_is_caught(self):
        """The E1 treatment's whole content is the forced provider. A
        record without it is the control, and merging it would make E1
        compare the nominal run against itself and report no effect."""
        run = RunSpec(experiment="E1",
                      attitude_provider="Attitude Quaternions")
        record = _record(run.run_id)          # provenance has no override
        assert any("attitude_provider" in m
                   for m in check_record(run, record))

    def test_wrong_gains_are_caught(self):
        run = RunSpec(experiment="E4", gains={"profile_authority": 0.8})
        record = _record(run.run_id, gains={"profile_authority": 0.6})
        assert any("profile_authority" in m for m in check_record(run, record))

    def test_the_execution_mode_is_deliberately_not_checked(self):
        """A bridge that fell back to free-run produced a real run in
        the mode it reports, and E3 groups by the observed mode so that
        it is counted where it belongs rather than discarded."""
        run = RunSpec(experiment="E3", execution_mode=LOCKSTEP)
        record = _record(run.run_id, mode=FREE_RUN)
        assert check_record(run, record) == []


class TestCommandLauncher:
    def test_the_run_configuration_is_substituted_into_the_command(self,
                                                                   tmp_path):
        marker = tmp_path / "out.json"
        launcher = CommandLauncher([
            "python", "-c",
            "import sys, pathlib; pathlib.Path(sys.argv[1]).write_text("
            "sys.argv[2])",
            "{output}", "{scheduler}"])
        run = RunSpec(experiment="E2", scheduler="kinematic")
        launcher(run, Campaign("c", "E2", "", runs=(run,)), str(marker))
        assert marker.read_text() == "kinematic"

    def test_a_failing_command_raises(self, tmp_path):
        launcher = CommandLauncher(["python", "-c", "raise SystemExit(3)"])
        run = RunSpec(experiment="E2")
        with pytest.raises(RuntimeError, match="exited 3"):
            launcher(run, Campaign("c", "E2", "", runs=(run,)),
                     str(tmp_path / "x.json"))

    def test_an_empty_command_is_refused(self):
        with pytest.raises(ValueError, match="launch command"):
            CommandLauncher([])


class TestRunConfigDelivery:
    """E4's gains and E5's fault have to cross into MATLAB.

    They are not scenario properties -- the scenario is identical in
    every run of both -- so they travel as a per-run file the twin reads
    through ``adcs.config``. Without this the fault was planned,
    recorded and replayed while never reaching the vehicle.
    """

    def test_a_run_config_is_written_beside_each_result(self, tmp_path):
        campaign = Campaign("c", "E5", "", runs=(
            RunSpec(experiment="E5", seed=4,
                    fault=WheelFault(axis=1, at_s=61.5, seed=4)),))
        run_campaign(campaign, _writing_launcher(),
                     RunnerOptions(output_dir=str(tmp_path)))

        written = sorted(p.name for p in tmp_path.glob("*.runconfig.json"))
        assert len(written) == 1
        payload = json.loads((tmp_path / written[0]).read_text())
        assert payload["fault"]["axis"] == 1
        assert payload["fault"]["at_s"] == 61.5
        assert payload["fault"]["kind"] == "saturation"

    def test_gains_travel_under_the_records_own_field_names(self, tmp_path):
        """One spelling from the campaign through the twin and back into
        the record. Two spellings for one quantity is how a sweep ends
        up flat."""
        campaign = Campaign("c", "E4", "", runs=(
            RunSpec(experiment="E4",
                    gains={"natural_frequency_rad_s": 1.2}),))
        run_campaign(campaign, _writing_launcher(),
                     RunnerOptions(output_dir=str(tmp_path)))

        payload = json.loads(
            next(tmp_path.glob("*.runconfig.json")).read_text())
        assert payload["gains"] == {"natural_frequency_rad_s": 1.2}

    def test_a_healthy_run_writes_a_null_fault(self, tmp_path):
        campaign = Campaign("c", "E2", "", runs=(
            RunSpec(experiment="E2"),))
        run_campaign(campaign, _writing_launcher(),
                     RunnerOptions(output_dir=str(tmp_path)))
        payload = json.loads(
            next(tmp_path.glob("*.runconfig.json")).read_text())
        assert payload["fault"] is None

    def test_the_launcher_can_reach_it_by_placeholder(self, tmp_path):
        """``{run_config}`` resolves beside whatever output path the run
        was given, so it stays paired with the record it produced even
        if the caller names the output something of its own."""
        launcher = CommandLauncher([
            "python", "-c",
            "import sys, pathlib; pathlib.Path(sys.argv[1]).write_text("
            "sys.argv[2])",
            "{output}", "{run_config}"])
        run = RunSpec(experiment="E5", seed=1,
                      fault=WheelFault(axis=0, at_s=3.0, seed=1))
        target = tmp_path / "out.json"
        launcher(run, Campaign("c", "E5", "", runs=(run,)), str(target))

        assert target.read_text() == str(tmp_path / "out.runconfig.json")


class TestE6Latency:
    def test_repeats_are_identical_but_for_seed(self):
        campaign = build_e6_latency(repeats=5)
        assert len(campaign) == 5
        assert len({r.execution_mode for r in campaign.runs}) == 1
        assert {r.execution_mode for r in campaign.runs} == {LOCKSTEP}
        assert {r.seed for r in campaign.runs} == {0, 1, 2, 3, 4}

    def test_at_least_one_run_is_required(self):
        with pytest.raises(ValueError, match="at least one"):
            build_e6_latency(repeats=0)

    def test_latency_breakdown_excludes_unreportable_runs(self):
        """A latency number from the mock says nothing about the deploy
        machine's real STK round trip, which is the entire point."""
        records = [
            _record("real", backend="com"),
            _record("mock", backend="mock"),
        ]
        # give the reportable one a real number to average
        records[0].result.mean_sense_latency_ms = 4.0
        records[1].result.mean_sense_latency_ms = 999.0

        breakdown = latency_breakdown(records)
        assert breakdown["runs"] == 1
        assert breakdown["excluded_unreportable"] == 1
        assert breakdown["mean_sense_latency_ms"] == pytest.approx(4.0)

    def test_an_empty_campaign_reports_nan_not_zero(self):
        breakdown = latency_breakdown([])
        assert breakdown["runs"] == 0
        assert math.isnan(breakdown["mean_sense_latency_ms"])


class TestReportCliFigures:
    """The `report` verb's figure generation -- see campaign_cli.py."""

    def _campaign_dir(self, tmp_path):
        campaign = Campaign("c", "E2", "", runs=(
            RunSpec(experiment="E2", scheduler="kinematic"),
            RunSpec(experiment="E2", scheduler="dynamics_aware")))
        run_campaign(campaign, _writing_launcher(), RunnerOptions(
            output_dir=str(tmp_path)))
        return campaign

    def test_runconfig_files_are_not_mistaken_for_records(self, tmp_path):
        """Both *.json and *.runconfig.json live in the output directory
        (see campaign_runner.write_run_config); a naive *.json glob
        would try to read the run-config as a record and fail every
        run in the campaign."""
        from acav.campaign_cli import load_results

        self._campaign_dir(tmp_path)
        assert any(tmp_path.glob("*.runconfig.json"))   # the fixture exists
        records = load_results(str(tmp_path))
        assert len(records) == 2

    def test_make_figure_e2_returns_two_named_figures(self, tmp_path):
        from acav.campaign_cli import analyse, load_results, make_figure

        self._campaign_dir(tmp_path)
        records = load_results(str(tmp_path))
        report = analyse("E2", records, total_targets=1)
        figures = make_figure("E2", report, records, total_targets=1)
        assert set(figures) == {"capture_coverage", "outcomes"}

    def test_write_figures_survives_a_missing_matplotlib(self, tmp_path,
                                                          monkeypatch):
        """A colleague running `report` for the JSON alone must not lose
        that because a plotting library is not installed."""
        import builtins

        from acav.campaign_cli import _write_figures

        real_import = builtins.__import__

        def blocking_import(name, *args, **kwargs):
            if name == "acav.report_figures" or name.startswith("matplotlib"):
                raise ImportError("simulated: matplotlib not installed")
            return real_import(name, *args, **kwargs)

        monkeypatch.setattr(builtins, "__import__", blocking_import)
        _write_figures("E2", {}, [], 1, str(tmp_path))   # must not raise
        assert not (tmp_path / "figures").exists()

    def test_write_figures_does_not_fail_the_report_on_a_bad_input(
            self, tmp_path):
        """A malformed report dict should not take the whole `report`
        verb down with it -- the JSON already printed is the valuable
        part."""
        from acav.campaign_cli import _write_figures

        _write_figures("E4", {"surface": "not a list"}, [], 1, str(tmp_path))
