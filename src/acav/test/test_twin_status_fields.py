"""TwinStatus carries what happened between two publishes.

In free-run the twin integrates several steps per published status, so
a brief event between two publishes would be invisible: a free-run
fault run recorded 0 saturation events where lockstep recorded
the 0.1 s saturation at 127.1 s, and the capture hold window saw only
the published samples. TwinStatus therefore has

    uint32  saturation_events                   cumulative, every step
    float64 max_boresight_error_since_last_rad  every step since the
                                                previous status

The record's saturation count is the larger of the twin's counter and
the observed rising edges, and the hold window is judged on the largest
error since the previous status. In lockstep every step is published,
so the counter equals the rising edges and the maximum equals the
published error: lockstep results do not change.
"""

import math
from types import SimpleNamespace
from unittest.mock import Mock

from acav.capture_tracking import RunAggregate, TelemetrySample
from acav.metrics import CaptureOutcome, PointingSample, evaluate_capture
from acav.scheduler_node import SchedulerNode
from stk_bridge.backends.base import StkTime

EPOCH = 1717237800.0
TOL, HOLD = 0.5, 2.0


def _window(errors, maxima=None):
    maxima = maxima or [None] * len(errors)
    return [PointingSample(time_s=98.0 + 0.5 * k, error_deg=e,
                           has_access=True, mode="IMAGE",
                           max_error_since_last_deg=m)
            for k, (e, m) in enumerate(zip(errors, maxima))]


class TestCapture:
    def test_an_excursion_between_publishes_fails_the_hold(self):
        samples = _window([0.1] * 5, [0.1, 0.1, 0.8, 0.1, 0.1])
        outcome, at_image, worst = evaluate_capture(samples, 100.0, TOL,
                                                    HOLD)
        assert outcome is CaptureOutcome.NOT_SETTLED
        assert worst == 0.8
        assert at_image == 0.1               # the published error at t_i

    def test_without_the_field_nothing_changes(self):
        samples = _window([0.1, 0.2, 0.3, 0.2, 0.1])
        assert evaluate_capture(samples, 100.0, TOL, HOLD) == (
            CaptureOutcome.CAPTURED, 0.1, 0.3)

    def test_lockstep_equality_gives_the_same_result(self):
        errors = [0.1, 0.45, 0.3, 0.2, 0.1]
        plain = evaluate_capture(_window(errors), 100.0, TOL, HOLD)
        with_max = evaluate_capture(_window(errors, errors), 100.0, TOL,
                                    HOLD)
        assert plain == with_max


def _sample(t, saturated=False, events=None):
    return TelemetrySample(time_s=t, error_deg=0.1, wheels_saturated=saturated,
                           saturation_events=events)


class TestSaturationEvents:
    def test_the_twin_counter_counts_what_was_not_published(self):
        aggregate = RunAggregate(momentum_capacity_Nms=0.02)
        for t, events in ((1.0, 0), (1.4, 1), (1.8, 1)):
            aggregate.observe(_sample(t, events=events))
        result = SimpleNamespace()
        aggregate.apply_to(result)
        assert result.wheel_saturation_events == 1

    def test_lockstep_counter_and_edges_agree(self):
        aggregate = RunAggregate(momentum_capacity_Nms=0.02)
        flags = [False, True, True, False, True, False]
        events = 0
        for k, flag in enumerate(flags):
            if flag and (k == 0 or not flags[k - 1]):
                events += 1
            aggregate.observe(_sample(0.1 * k, flag, events))
        result = SimpleNamespace()
        aggregate.apply_to(result)
        assert result.wheel_saturation_events == 2

    def test_an_older_twin_without_the_field(self):
        aggregate = RunAggregate(momentum_capacity_Nms=0.02)
        for k, flag in enumerate([False, True, False, True]):
            aggregate.observe(_sample(0.1 * k, flag))
        result = SimpleNamespace()
        aggregate.apply_to(result)
        assert result.wheel_saturation_events == 2


def _status(t, **fields):
    whole = int(t)
    base = dict(
        header=SimpleNamespace(stamp=SimpleNamespace(
            sec=int(EPOCH) + whole, nanosec=int(round((t - whole) * 1e9)))),
        boresight_error_rad=math.radians(0.1), mode="IMAGE",
        wheel_momentum_nms=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        wheels_saturated=False, wheel_energy_j=0.0)
    base.update(fields)
    return SimpleNamespace(**base)


class TestScheduler:
    def _node(self):
        node = SchedulerNode.__new__(SchedulerNode)
        node._epoch = StkTime(EPOCH)
        node._last_sim_time_s = 10.0
        node._misstamped_twin_status = 0
        node._tracker = Mock()
        node._aggregate = Mock()
        return node

    def test_the_new_fields_reach_the_sample(self):
        node = self._node()
        node._on_twin_status(_status(
            10.0, saturation_events=3,
            max_boresight_error_since_last_rad=math.radians(0.7)))
        sample = node._tracker.observe.call_args.args[0]
        assert sample.saturation_events == 3
        assert abs(sample.max_error_since_last_deg - 0.7) < 1e-12

    def test_a_status_without_them_still_reads(self):
        node = self._node()
        node._on_twin_status(_status(10.0))
        sample = node._tracker.observe.call_args.args[0]
        assert sample.saturation_events is None
        assert sample.max_error_since_last_deg is None


def test_the_tracker_passes_the_maximum_on():
    from acav.capture_tracking import CaptureTracker

    tracker = CaptureTracker(capture_error_deg=TOL, settle_hold_s=HOLD,
                             retention_s=7.0)
    for k in range(5):
        tracker.observe(TelemetrySample(
            time_s=98.0 + 0.5 * k, error_deg=0.1, mode="IMAGE",
            max_error_since_last_deg=0.8 if k == 2 else 0.1))
    entry = SimpleNamespace(target_id="T1", t_image_s=100.0)
    assert tracker.finalize(entry, True).outcome is \
        CaptureOutcome.NOT_SETTLED
