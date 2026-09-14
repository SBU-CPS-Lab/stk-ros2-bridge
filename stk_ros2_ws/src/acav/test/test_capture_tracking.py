"""Tests for the telemetry buffering / capture-finalisation glue.

Deliberately synthetic telemetry throughout: this module's job is
bookkeeping (which samples belong to which target's hold window, when
is it safe to discard old ones), not physics, and the criterion itself
is already pinned by ``test_metrics.py``. What matters here is that the
bookkeeping cannot silently misattribute a sample or drop one that was
still needed.
"""

import pytest

from acav.capture_tracking import (
    CaptureTracker,
    TelemetrySample,
    declined_result,
    RunAggregate,
)
from acav.metrics import RunResult, CaptureOutcome
from acav.scheduling import DeclinedEntry, ScheduleEntry
from acav.spec import TargetSpec


def target(id="T1", t_image_s=100.0, cross_track_km=0.0):
    return TargetSpec(id=id, t_image_s=t_image_s,
                      cross_track_km=cross_track_km, priority=1.0)


def entry(**kwargs):
    kwargs.setdefault("target", target())
    kwargs.setdefault("activation_start_s", 0.0)
    kwargs.setdefault("required_s", 10.0)
    kwargs.setdefault("available_s", 100.0)
    return ScheduleEntry(**kwargs)


class TestObserveOrdering:
    def test_accepts_non_decreasing_times(self):
        tracker = CaptureTracker(0.5, 2.0, retention_s=10.0)
        tracker.observe(TelemetrySample(1.0, 5.0))
        tracker.observe(TelemetrySample(1.0, 4.0))
        tracker.observe(TelemetrySample(2.0, 3.0))

    def test_rejects_a_sample_that_arrives_out_of_order(self):
        """STK itself imposes strictly-increasing time on injected
        real-time samples; the tracker holds telemetry to the same
        standard rather than silently reordering or dropping it."""
        tracker = CaptureTracker(0.5, 2.0, retention_s=10.0)
        tracker.observe(TelemetrySample(5.0, 1.0))
        with pytest.raises(ValueError, match="non-decreasing"):
            tracker.observe(TelemetrySample(4.0, 1.0))


class TestRetentionWindow:
    def test_short_retention_is_rejected_at_construction(self):
        with pytest.raises(ValueError, match="retention_s"):
            CaptureTracker(0.5, 2.0, retention_s=1.0)

    def test_samples_older_than_retention_are_discarded(self):
        tracker = CaptureTracker(0.5, 2.0, retention_s=5.0)
        for t in range(0, 21):
            tracker.observe(TelemetrySample(float(t), 0.1))
        # Newest is t=20; retention 5.0 keeps t >= 15.
        assert min(s.time_s for s in tracker.window(20.0)) >= 15.0 - 1e-9

    def test_window_never_returns_samples_outside_the_hold_period(self):
        tracker = CaptureTracker(0.5, 2.0, retention_s=10.0)
        for t in (95.0, 97.9, 98.0, 99.0, 100.0, 100.1):
            tracker.observe(TelemetrySample(t, 0.1))
        window = tracker.window(100.0)
        times = sorted(s.time_s for s in window)
        assert times == [98.0, 99.0, 100.0]


class TestFinalize:
    def test_captured_when_settled_and_access_true(self):
        tracker = CaptureTracker(0.5, 2.0, retention_s=10.0)
        for t in (98.0, 99.0, 100.0):
            tracker.observe(TelemetrySample(t, 0.1))
        result = tracker.finalize(entry(target=target(t_image_s=100.0)),
                                  has_access=True)
        assert result.outcome is CaptureOutcome.CAPTURED
        assert result.attempted
        assert result.target_id == "T1"
        assert result.error_at_image_deg == pytest.approx(0.1)

    def test_not_settled_when_access_true_but_error_too_large(self):
        tracker = CaptureTracker(0.5, 2.0, retention_s=10.0)
        for t, err in ((98.0, 5.0), (99.0, 1.0), (100.0, 0.3)):
            tracker.observe(TelemetrySample(t, err))
        result = tracker.finalize(entry(target=target(t_image_s=100.0)),
                                  has_access=True)
        assert result.outcome is CaptureOutcome.NOT_SETTLED

    def test_no_access_when_settled_but_access_false(self):
        tracker = CaptureTracker(0.5, 2.0, retention_s=10.0)
        for t in (98.0, 99.0, 100.0):
            tracker.observe(TelemetrySample(t, 0.1))
        result = tracker.finalize(entry(target=target(t_image_s=100.0)),
                                  has_access=False)
        assert result.outcome is CaptureOutcome.NO_ACCESS

    def test_safe_mode_is_detected_from_the_mode_field(self):
        tracker = CaptureTracker(0.5, 2.0, retention_s=10.0)
        tracker.observe(TelemetrySample(98.0, 0.1, mode="IMAGE"))
        tracker.observe(TelemetrySample(99.0, 20.0, mode="SAFE"))
        tracker.observe(TelemetrySample(100.0, 20.0, mode="SAFE"))
        result = tracker.finalize(entry(target=target(t_image_s=100.0)),
                                  has_access=True)
        assert result.outcome is CaptureOutcome.SAFE_MODE

    def test_raises_when_the_hold_window_has_no_telemetry(self):
        """A harness bug (nothing recorded, or too short a retention)
        must not be scored as a physics result -- it must be loud."""
        tracker = CaptureTracker(0.5, 2.0, retention_s=10.0)
        tracker.observe(TelemetrySample(0.0, 0.1))
        with pytest.raises(ValueError, match="no telemetry"):
            tracker.finalize(entry(target=target(t_image_s=100.0)),
                            has_access=True)

    def test_two_consecutive_targets_do_not_cross_contaminate(self):
        """The scenario's real activation windows are contiguous, so a
        sample recorded one instant before a handoff must not leak into
        the NEXT target's hold-window evaluation, and vice versa."""
        tracker = CaptureTracker(0.5, 2.0, retention_s=20.0)
        # Target A settles by its image epoch (t=50); target B is
        # slewing hard and is NOT settled by its own epoch (t=60).
        for t in (48.0, 49.0, 50.0):
            tracker.observe(TelemetrySample(t, 0.1, mode="IMAGE"))
        for t in (58.0, 59.0, 60.0):
            tracker.observe(TelemetrySample(t, 5.0, mode="SLEW"))

        result_a = tracker.finalize(
            entry(target=target(id="A", t_image_s=50.0)), has_access=True)
        result_b = tracker.finalize(
            entry(target=target(id="B", t_image_s=60.0)), has_access=True)

        assert result_a.outcome is CaptureOutcome.CAPTURED
        assert result_b.outcome is CaptureOutcome.NOT_SETTLED


class TestDeclinedResult:
    def test_declined_target_is_not_attempted(self):
        declined = DeclinedEntry(target=target(id="T5", t_image_s=265.0),
                                 required_s=16.9, available_s=15.0)
        result = declined_result(declined)
        assert not result.attempted
        assert result.outcome is CaptureOutcome.NOT_ATTEMPTED
        assert result.target_id == "T5"
        assert result.image_time_s == pytest.approx(265.0)


class TestRunAggregate:
    """Run-scoped quantities, which belong to the run and not to a target.

    Without these the record's peak momentum, saturation count, SAFE flag
    and wheel energy all stayed at their defaults, so E5's SAFE count
    read zero and E3's determinism signature hashed three constants.
    """

    def _sample(self, momentum=(0.0, 0.0, 0.0), saturated=False,
                energy=0.0, mode="SLEW", time_s=0.0):
        return TelemetrySample(time_s=time_s, error_deg=1.0, mode=mode,
                               wheel_momentum_Nms=momentum,
                               wheels_saturated=saturated,
                               wheel_energy_J=energy)

    def test_peak_momentum_is_per_axis_and_kept(self):
        aggregate = RunAggregate(momentum_capacity_Nms=0.02)
        aggregate.observe(self._sample(momentum=(0.01, 0.002, 0.0)))
        aggregate.observe(self._sample(momentum=(0.004, 0.02, 0.0)))
        assert aggregate.peak_momentum_fraction == pytest.approx(
            [0.5, 1.0, 0.0])

    def test_momentum_magnitude_counts_regardless_of_sign(self):
        """A wheel at -h_max is exactly as saturated as one at +h_max."""
        aggregate = RunAggregate(momentum_capacity_Nms=0.02)
        aggregate.observe(self._sample(momentum=(-0.02, 0.0, 0.0)))
        assert aggregate.peak_momentum_fraction[0] == pytest.approx(1.0)

    def test_saturation_counts_entries_not_steps(self):
        """Counting steps would make the number a function of the
        telemetry rate: the same run sampled twice as often would report
        twice the faults, and two runs at different rates could not be
        compared -- which is exactly what E5 needs of them."""
        aggregate = RunAggregate(momentum_capacity_Nms=0.02)
        for saturated in (False, True, True, True, False, True, True):
            aggregate.observe(self._sample(saturated=saturated))
        assert aggregate.wheel_saturation_events == 2

    def test_safe_mode_is_sticky(self):
        aggregate = RunAggregate(momentum_capacity_Nms=0.02)
        aggregate.observe(self._sample(mode="SLEW"))
        aggregate.observe(self._sample(mode="SAFE"))
        aggregate.observe(self._sample(mode="SAFE"))
        assert aggregate.entered_safe_mode

    def test_energy_takes_the_maximum_of_a_cumulative_series(self):
        """The twin publishes a running integral. Taking the last value
        instead would let one late-arriving stale message under-report
        the whole run."""
        aggregate = RunAggregate(momentum_capacity_Nms=0.02)
        for energy in (0.1, 0.4, 0.9, 0.4):      # last one arrives late
            aggregate.observe(self._sample(energy=energy))
        assert aggregate.wheel_energy_J == pytest.approx(0.9)

    def test_the_safe_mode_name_is_parameterised(self):
        aggregate = RunAggregate(momentum_capacity_Nms=0.02,
                                 safe_mode_name="HALT")
        aggregate.observe(self._sample(mode="SAFE"))
        assert not aggregate.entered_safe_mode
        aggregate.observe(self._sample(mode="HALT"))
        assert aggregate.entered_safe_mode

    def test_it_writes_through_to_a_run_result(self):
        aggregate = RunAggregate(momentum_capacity_Nms=0.02)
        aggregate.observe(self._sample(momentum=(0.02, 0.0, 0.0),
                                       saturated=True, energy=1.5,
                                       mode="SAFE"))
        result = RunResult(run_id="r", scheduler="dynamics_aware")
        aggregate.apply_to(result)

        assert result.peak_momentum_fraction == pytest.approx((1.0, 0.0, 0.0))
        assert result.wheel_saturation_events == 1
        assert result.entered_safe_mode
        assert result.wheel_energy_J == pytest.approx(1.5)

    def test_an_untouched_run_reports_zeroes_not_defaults_by_accident(self):
        result = RunResult(run_id="r", scheduler="dynamics_aware")
        RunAggregate(momentum_capacity_Nms=0.02).apply_to(result)
        assert result.peak_momentum_fraction == (0.0, 0.0, 0.0)
        assert not result.entered_safe_mode
