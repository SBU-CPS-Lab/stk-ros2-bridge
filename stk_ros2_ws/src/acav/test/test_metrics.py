"""Tests for the capture criterion and metrics (the C4/C5/C6 contract)."""

import math

import pytest

from acav.metrics import (
    CaptureOutcome,
    PointingSample,
    RunResult,
    TargetResult,
    capture_rate,
    coverage_rate,
    determinism_report,
    determinism_signature,
    evaluate_capture,
    outcome_breakdown,
    over_promise_gap,
    pointing_error_stats,
)

CAPTURE_ERR = 0.5
HOLD = 2.0


def samples(spec, image_time=100.0, hz=10.0):
    """Build a hold-window telemetry series from (error_deg, has_access)."""
    out = []
    count = int(HOLD * hz) + 1
    for i in range(count):
        t = image_time - HOLD + i / hz
        error, access, mode = spec(t)
        out.append(PointingSample(time_s=t, error_deg=error,
                                  has_access=access, mode=mode))
    return out


class TestCaptureCriterion:
    def test_settled_and_access_is_captured(self):
        s = samples(lambda t: (0.2, True, "IMAGE"))
        outcome, at_image, worst = evaluate_capture(s, 100.0, CAPTURE_ERR, HOLD)
        assert outcome is CaptureOutcome.CAPTURED
        assert at_image == pytest.approx(0.2)
        assert worst == pytest.approx(0.2)

    def test_access_but_not_settled_is_not_settled(self):
        """The failure mode the whole study is about: geometry permitted
        the image, the controller had not converged."""
        s = samples(lambda t: (1.5 if t < 99.0 else 0.2, True, "SETTLE"))
        outcome, _, worst = evaluate_capture(s, 100.0, CAPTURE_ERR, HOLD)
        assert outcome is CaptureOutcome.NOT_SETTLED
        assert worst > CAPTURE_ERR

    def test_settled_but_no_access_is_no_access(self):
        s = samples(lambda t: (0.1, False, "IMAGE"))
        outcome, _, _ = evaluate_capture(s, 100.0, CAPTURE_ERR, HOLD)
        assert outcome is CaptureOutcome.NO_ACCESS

    def test_neither_is_combined_outcome(self):
        s = samples(lambda t: (3.0, False, "SLEW"))
        outcome, _, _ = evaluate_capture(s, 100.0, CAPTURE_ERR, HOLD)
        assert outcome is CaptureOutcome.NO_ACCESS_AND_NOT_SETTLED

    def test_settling_late_within_the_window_still_fails(self):
        """Settling at t-0.1s is not enough: the criterion requires the
        error to hold for the WHOLE window, which is what distinguishes
        a usable image from a lucky instant."""
        s = samples(lambda t: (0.2 if t > 99.9 else 0.9, True, "SETTLE"))
        outcome, _, _ = evaluate_capture(s, 100.0, CAPTURE_ERR, HOLD)
        assert outcome is CaptureOutcome.NOT_SETTLED

    def test_error_exactly_at_tolerance_is_accepted(self):
        s = samples(lambda t: (CAPTURE_ERR, True, "IMAGE"))
        outcome, _, _ = evaluate_capture(s, 100.0, CAPTURE_ERR, HOLD)
        assert outcome is CaptureOutcome.CAPTURED

    def test_safe_mode_dominates_all_other_outcomes(self):
        s = samples(lambda t: (0.1, True, "SAFE"))
        outcome, _, _ = evaluate_capture(s, 100.0, CAPTURE_ERR, HOLD)
        assert outcome is CaptureOutcome.SAFE_MODE

    def test_access_evaluated_at_image_epoch_not_across_window(self):
        """The spacecraft may legitimately be slewing INTO the footprint
        during settling, so access is required at the image epoch, not
        throughout the hold."""
        s = samples(lambda t: (0.2, t > 99.5, "IMAGE"))
        outcome, _, _ = evaluate_capture(s, 100.0, CAPTURE_ERR, HOLD)
        assert outcome is CaptureOutcome.CAPTURED

    def test_missing_telemetry_raises_rather_than_scoring_a_failure(self):
        """A harness bug must not be silently recorded as a capture
        failure -- that would corrupt the headline metric with
        data-collection problems."""
        far_away = [PointingSample(time_s=0.0, error_deg=0.1, has_access=True)]
        with pytest.raises(ValueError, match="no telemetry samples"):
            evaluate_capture(far_away, 100.0, CAPTURE_ERR, HOLD)


def make_run(outcomes, scheduler="kinematic"):
    targets = [
        TargetResult(
            target_id=f"T{i+1}",
            attempted=(o is not CaptureOutcome.NOT_ATTEMPTED),
            outcome=o,
            image_time_s=60.0 + 50 * i,
            error_at_image_deg=0.2 if o is CaptureOutcome.CAPTURED else 1.4,
        )
        for i, o in enumerate(outcomes)
    ]
    return RunResult(run_id="r", scheduler=scheduler, targets=targets)


class TestMetrics:
    def test_capture_rate_denominator_is_attempted_not_total(self):
        """A scheduler that correctly declines an infeasible target must
        not be penalised for it."""
        run = make_run([CaptureOutcome.CAPTURED,
                        CaptureOutcome.NOT_ATTEMPTED])
        assert capture_rate(run) == pytest.approx(1.0)

    def test_coverage_rate_uses_the_full_target_list(self):
        """The complement to capture_rate: catches a scheduler gaming
        capture rate by attempting only one easy target."""
        run = make_run([CaptureOutcome.CAPTURED,
                        CaptureOutcome.NOT_ATTEMPTED])
        assert coverage_rate(run, total_targets=2) == pytest.approx(0.5)

    def test_capture_and_coverage_diverge_for_a_conservative_scheduler(self):
        conservative = make_run(
            [CaptureOutcome.CAPTURED] + [CaptureOutcome.NOT_ATTEMPTED] * 6)
        assert capture_rate(conservative) == pytest.approx(1.0)
        assert coverage_rate(conservative, 7) == pytest.approx(1 / 7)

    def test_over_promise_gap_is_the_headline_quantity(self):
        run = make_run([CaptureOutcome.CAPTURED,
                        CaptureOutcome.NOT_SETTLED,
                        CaptureOutcome.NOT_SETTLED,
                        CaptureOutcome.CAPTURED])
        assert over_promise_gap(run) == pytest.approx(0.5)

    def test_nothing_attempted_yields_nan_not_zero(self):
        run = make_run([CaptureOutcome.NOT_ATTEMPTED])
        assert math.isnan(capture_rate(run))
        assert math.isnan(over_promise_gap(run))

    def test_outcome_breakdown_attributes_failures(self):
        run = make_run([CaptureOutcome.CAPTURED,
                        CaptureOutcome.NOT_SETTLED,
                        CaptureOutcome.NO_ACCESS])
        counts = outcome_breakdown(run)
        assert counts["captured"] == 1
        assert counts["not_settled"] == 1
        assert counts["no_access"] == 1

    def test_pointing_error_stats_ignores_unattempted(self):
        run = make_run([CaptureOutcome.CAPTURED,
                        CaptureOutcome.NOT_ATTEMPTED])
        stats = pointing_error_stats(run)
        assert stats["count"] == 1


class TestDeterminism:
    def test_identical_runs_share_a_signature(self):
        a = make_run([CaptureOutcome.CAPTURED, CaptureOutcome.NOT_SETTLED])
        b = make_run([CaptureOutcome.CAPTURED, CaptureOutcome.NOT_SETTLED])
        assert determinism_signature(a) == determinism_signature(b)

    def test_signature_catches_differences_a_capture_count_would_miss(self):
        """Two runs can agree on capture count while differing in every
        pointing error. Comparing summary statistics would call that
        deterministic; the signature does not."""
        a = make_run([CaptureOutcome.CAPTURED])
        b = make_run([CaptureOutcome.CAPTURED])
        b.targets[0].error_at_image_deg = 0.3      # was 0.2
        assert a.captured_count == b.captured_count
        assert determinism_signature(a) != determinism_signature(b)

    def test_signature_is_order_independent(self):
        a = make_run([CaptureOutcome.CAPTURED, CaptureOutcome.NOT_SETTLED])
        b = make_run([CaptureOutcome.CAPTURED, CaptureOutcome.NOT_SETTLED])
        b.targets.reverse()
        assert determinism_signature(a) == determinism_signature(b)

    def test_differences_far_below_tolerance_do_not_flag(self):
        """Rounding at 1e-9 deg keeps float noise from turning a real
        result into spurious non-determinism."""
        a = make_run([CaptureOutcome.CAPTURED])
        b = make_run([CaptureOutcome.CAPTURED])
        b.targets[0].error_at_image_deg += 1e-15
        assert determinism_signature(a) == determinism_signature(b)

    def test_report_flags_lockstep_style_determinism(self):
        runs = [make_run([CaptureOutcome.CAPTURED]) for _ in range(20)]
        report = determinism_report(runs)
        assert report["deterministic"] is True
        assert report["distinct_signatures"] == 1
        assert report["pointing_error_stdev_deg"] == pytest.approx(0.0)

    def test_report_flags_free_run_style_variability(self):
        runs = []
        for i in range(20):
            run = make_run([CaptureOutcome.CAPTURED])
            run.targets[0].error_at_image_deg = 0.2 + i * 0.001
            runs.append(run)
        report = determinism_report(runs)
        assert report["deterministic"] is False
        assert report["distinct_signatures"] == 20
        assert report["pointing_error_stdev_deg"] > 0.0
