"""Every scenario target stays in the run record, scored or not.

In a free-run run at rtf 1.0 the twin published nothing between t = 247.734 s and 252.219 s, so T4's hold
window [248, 250] s held no telemetry. The scheduler then dropped T4
from result.targets -- and so from the determinism signature and from
every count -- while the record stayed reportable, so a report would
have scored the run 3/4 instead of 3/5.

An attempted target that cannot be scored is now recorded with why:
``no_telemetry`` (no twin sample in the hold window) or
``no_access_answer`` (STK's access answer never arrived). In lockstep
``no_telemetry`` breaks an invariant -- the barrier makes the twin
publish every step -- so such a record is not reportable; in free-run
it is what asynchronous coupling can do, and stays reportable. A
missing access answer is a harness failure in either mode.
"""

import json
import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from acav.campaign import compare_schedulers
from acav.capture_tracking import no_telemetry_result
from acav.metrics import (
    CaptureOutcome,
    RunResult,
    TargetResult,
    capture_rate,
    determinism_signature,
    outcome_breakdown,
)
from acav.run_record import Provenance, RunRecord, read_record, write_record
from acav.scheduler_node import SchedulerNode
from stk_bridge.backends.base import StkTime

EPOCH = 1717237800.0


def _target(target_id, outcome, attempted=True, error=0.002, reason=""):
    return TargetResult(target_id=target_id, attempted=attempted,
                        outcome=outcome, image_time_s=60.0,
                        error_at_image_deg=error,
                        max_error_in_hold_deg=error, reason=reason)


def _result(unscored=CaptureOutcome.NO_TELEMETRY):
    return RunResult(run_id="run", scheduler="dynamics_aware", targets=[
        _target("T1", CaptureOutcome.CAPTURED),
        _target("T4", unscored, error=float("nan"),
                reason="no telemetry samples in the hold window"),
        _target("T5", CaptureOutcome.NOT_ATTEMPTED, attempted=False,
                error=float("nan")),
    ])


def _record(mode, unscored=CaptureOutcome.NO_TELEMETRY):
    return RunRecord(result=_result(unscored), experiment="E3",
                     provenance=Provenance(backend_type="com",
                                           execution_mode=mode))


# -- the outcome itself -------------------------------------------------

class TestOutcome:
    def test_attempted_and_not_captured(self):
        result = _result()
        assert result.attempted_count == 2
        assert result.captured_count == 1
        assert capture_rate(result) == pytest.approx(0.5)

    def test_counted_as_its_own_cause(self):
        counts = outcome_breakdown(_result())
        assert counts["no_telemetry"] == 1
        assert counts["no_access"] == 0
        assert counts["not_settled"] == 0
        assert counts["no_access_and_not_settled"] == 0
        counts = outcome_breakdown(_result(CaptureOutcome.NO_ACCESS_ANSWER))
        assert counts["no_access_answer"] == 1

    def test_the_signature_names_every_target(self):
        signature = determinism_signature(_result())
        assert "T1:captured:" in signature
        assert "T4:no_telemetry:nan:nan" in signature
        assert "T5:not_attempted:nan:nan" in signature

    def test_the_helper_builds_it_from_a_schedule_entry(self):
        entry = SimpleNamespace(target_id="T4", t_image_s=250.0)
        result = no_telemetry_result(entry, "why")
        assert result.target_id == "T4" and result.attempted
        assert result.outcome is CaptureOutcome.NO_TELEMETRY
        assert result.image_time_s == 250.0 and result.reason == "why"
        assert math.isnan(result.error_at_image_deg)


# -- the record ---------------------------------------------------------

class TestRecord:
    def test_lockstep_no_telemetry_is_not_reportable(self):
        record = _record("lockstep")
        assert not record.reportable
        reasons = record.unreportable_reasons
        assert len(reasons) == 1
        assert "T4" in reasons[0] and "lockstep" in reasons[0]

    def test_free_run_no_telemetry_stays_reportable(self):
        record = _record("free_run")
        assert record.reportable
        assert record.unreportable_reasons == []

    @pytest.mark.parametrize("mode", ["lockstep", "free_run"])
    def test_a_missing_access_answer_is_never_reportable(self, mode):
        record = _record(mode, CaptureOutcome.NO_ACCESS_ANSWER)
        assert not record.reportable
        assert any("T4" in r for r in record.unreportable_reasons)

    def test_the_mock_reason_is_still_given(self):
        record = RunRecord(result=_result(CaptureOutcome.CAPTURED),
                           provenance=Provenance(backend_type="mock",
                                                 execution_mode="lockstep"))
        assert not record.reportable
        assert any("mock" in r for r in record.unreportable_reasons)

    def test_round_trip_keeps_the_outcome_and_reason(self, tmp_path):
        path = tmp_path / "r.json"
        write_record(str(path), _record("free_run"))
        raw = json.loads(path.read_text(encoding="utf-8"))
        assert raw["unreportable_reasons"] == []
        back = read_record(str(path))
        t4 = [t for t in back.result.targets if t.target_id == "T4"][0]
        assert t4.outcome is CaptureOutcome.NO_TELEMETRY
        assert t4.reason == "no telemetry samples in the hold window"
        assert back.signature == _record("free_run").signature

    def test_an_old_record_without_reasons_still_reads(self, tmp_path):
        """A target with no reason is written exactly as before, and a
        record written before the field existed reads with an empty one."""
        record = RunRecord(result=_result(CaptureOutcome.CAPTURED),
                           provenance=Provenance(backend_type="com",
                                                 execution_mode="lockstep"))
        payload = record.to_dict()
        for target in payload["result"]["targets"]:
            target.pop("reason", None)
        payload.pop("unreportable_reasons")
        path = tmp_path / "old.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        back = read_record(str(path))
        assert all(t.reason == "" for t in back.result.targets)
        assert back.reportable

    def test_a_target_without_a_reason_is_written_as_before(self):
        payload = RunRecord(result=_result(CaptureOutcome.CAPTURED)).to_dict()
        assert all("reason" not in t for t in payload["result"]["targets"]
                   if t["target_id"] != "T4")


# -- the scheduler ------------------------------------------------------

def _stamp(seconds):
    whole = int(seconds)
    return SimpleNamespace(sec=whole,
                           nanosec=int(round((seconds - whole) * 1e9)))


def _entry(target_id, t_image_s):
    return SimpleNamespace(target_id=target_id, t_image_s=t_image_s,
                           target=SimpleNamespace(id=target_id,
                                                  t_image_s=t_image_s))


def _node(target_ids):
    node = SchedulerNode.__new__(SchedulerNode)
    node._epoch = StkTime(EPOCH)
    node._results = {}
    node._finalized = set()
    node._access_requested = set()
    node._init_access_state()
    node._spec = SimpleNamespace(
        window_duration_s=720.0,
        targets=[SimpleNamespace(id=t, t_image_s=60.0 * (k + 1))
                 for k, t in enumerate(target_ids)])
    node._plan = SimpleNamespace(
        attempted=[_entry(t, 60.0 * (k + 1))
                   for k, t in enumerate(target_ids)],
        scheduler_type=SimpleNamespace(value="dynamics_aware"))
    node._options = SimpleNamespace(run_id="run", seed=0, output_json="",
                                    auto_shutdown=False)
    node._aggregate = Mock()
    node._latency = Mock()
    node._tick_intervals_ms = []
    node._shutdown_requested = False
    node._window_elapsed_wall = None
    return node


def _access_future(intervals=()):
    response = SimpleNamespace(
        success=True, message="",
        accesses=SimpleNamespace(intervals=[
            SimpleNamespace(start_time=_stamp(a), stop_time=_stamp(b))
            for a, b in intervals]))
    return SimpleNamespace(result=lambda: response)


class TestScheduler:
    def test_an_unscorable_target_is_recorded_not_dropped(self):
        node = _node(["T1", "T4"])
        node._tracker = Mock()
        node._tracker.finalize.side_effect = ValueError(
            "no telemetry samples in the hold window [248.000, 250.000] s")
        entry = node._plan.attempted[1]
        node._on_access_response(entry, _access_future())
        result = node._results["T4"]
        assert result.outcome is CaptureOutcome.NO_TELEMETRY
        assert result.attempted
        assert "no telemetry samples" in result.reason
        assert "T4" in node._finalized

    def test_a_no_telemetry_target_does_not_force_the_finish(self):
        node = _node(["T1"])
        node._tracker = Mock()
        node._tracker.finalize.side_effect = ValueError("no telemetry")
        node._on_access_response(node._plan.attempted[0], _access_future())
        node._finish_run = Mock()
        node._check_run_complete(720.0)
        node._finish_run.assert_called_once_with(forced=False)

    def test_a_missing_access_answer_is_recorded_at_the_forced_finish(self):
        node = _node(["T1", "T2"])
        node._results["T1"] = _target("T1", CaptureOutcome.CAPTURED)
        node._finalized.add("T1")
        node._finish_run(forced=True)
        result = node._results["T2"]
        assert result.outcome is CaptureOutcome.NO_ACCESS_ANSWER
        assert result.attempted and result.image_time_s == 120.0
        assert "no access answer" in result.reason

    def test_every_scenario_target_reaches_the_result(self):
        node = _node(["T1", "T2", "T4"])
        node._results["T1"] = _target("T1", CaptureOutcome.CAPTURED)
        node._finalized.add("T1")
        node._tracker = Mock()
        node._tracker.finalize.side_effect = ValueError("no telemetry")
        node._on_access_response(node._plan.attempted[2], _access_future())
        node._finish_run(forced=True)                   # T2 never answered
        ids = [t.target_id for t in node._build_run_result().targets]
        assert ids == ["T1", "T2", "T4"]


# -- the report ---------------------------------------------------------

class TestReport:
    def test_aggregation_counts_no_telemetry(self):
        summary = compare_schedulers([_record("free_run")], total_targets=3)
        outcomes = summary["dynamics_aware"]["outcomes"]
        assert outcomes["no_telemetry"] == 1
        assert outcomes["captured"] == 1

    def test_analyse_names_why_a_run_is_not_reportable(self):
        from acav.campaign_cli import analyse

        report = analyse("E3", [_record("lockstep")], total_targets=3)
        assert report["unreportable"] == ["run"]
        assert "T4" in report["unreportable_reasons"]["run"][0]

    def test_check_record_accepts_the_new_outcome(self):
        from acav.campaign import RunSpec
        from acav.campaign_runner import check_record

        from test_campaign import covering

        run = RunSpec(experiment="E3", execution_mode="free_run")
        record = covering(_record("free_run"))
        record.result.run_id = run.run_id
        assert check_record(run, record) == []

    def test_the_outcome_figure_shows_no_telemetry(self):
        pytest.importorskip("matplotlib")
        from acav.report_figures import e2_outcome_breakdown_figure
        from acav.report_style import STATUS_COLOR, status_label

        assert status_label("no_telemetry") == "No telemetry"
        assert status_label("no_access_answer") == "No access answer"
        assert {"no_telemetry", "no_access_answer"} <= set(STATUS_COLOR)
        summary = compare_schedulers([_record("free_run")], total_targets=3)
        fig = e2_outcome_breakdown_figure(summary)
        labels = [t.get_text() for t in fig.axes[0].get_legend().get_texts()]
        assert "No telemetry" in labels
