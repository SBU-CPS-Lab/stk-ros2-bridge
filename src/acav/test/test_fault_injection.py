"""Tests for E5's seeded wheel faults.

E5's claim is deterministic fault *replay*, so the thing being replayed
has to be deterministic first. These assert the properties that make it
so, and each corresponds to a way the experiment could look correct
while being unreproducible.
"""

import json
import random

import pytest

from acav.fault_injection import (
    DEAD,
    FAILURE,
    SATURATION,
    WheelFault,
    fault_from_dict,
    plan_campaign_faults,
    plan_fault,
)

WINDOW = 720.0


class TestSeedDeterminism:
    def test_the_same_seed_gives_the_same_fault(self):
        assert plan_fault(42, WINDOW) == plan_fault(42, WINDOW)

    def test_different_seeds_generally_differ(self):
        faults = {plan_fault(seed, WINDOW) for seed in range(30)}
        assert len(faults) > 20

    def test_planning_does_not_disturb_the_global_random_state(self):
        """A fault drawn from the module-level RNG would depend on how
        many other draws happened first, which is reproducible only by
        accident and stops being so the moment anything else in the
        process uses randomness."""
        random.seed(1234)
        expected = [random.random() for _ in range(3)]

        random.seed(1234)
        plan_fault(7, WINDOW)
        plan_fault(8, WINDOW)
        assert [random.random() for _ in range(3)] == expected

    def test_a_seeds_fault_does_not_depend_on_the_other_seeds(self):
        """Adding or reordering seeds must not renumber anyone else's
        fault. If it did, E5's "re-run the seed that failed" would
        reproduce a different fault than the one that failed."""
        first = plan_campaign_faults([1, 2, 3], WINDOW)
        second = plan_campaign_faults([3, 99, 1, 2], WINDOW)
        for seed in (1, 2, 3):
            assert first[seed] == second[seed]


class TestReplayFidelity:
    def test_the_epoch_survives_a_json_round_trip_exactly(self):
        """A replay whose injection epoch differs from the original by
        float formatting is a replay that differs by one control step --
        the smallest way for this experiment to be wrong, and the
        hardest to spot."""
        fault = plan_fault(11, WINDOW)
        restored = fault_from_dict(json.loads(json.dumps(fault.to_dict())))
        assert restored == fault
        assert restored.at_s == fault.at_s

    def test_replay_uses_the_recorded_fault_not_the_rule(self):
        """``fault_from_dict`` must reconstruct exactly what is in the
        record. That is what keeps an old result replayable after the
        sampling rule in ``plan_fault`` changes."""
        recorded = {"axis": 2, "at_s": 123.456, "health": 0.0, "seed": 5}
        fault = fault_from_dict(recorded)
        assert (fault.axis, fault.at_s, fault.seed) == (2, 123.456, 5)

    def test_no_fault_round_trips_as_none(self):
        assert fault_from_dict(None) is None
        assert fault_from_dict({}) is None


class TestHealthSchedule:
    def test_the_wheel_is_healthy_before_the_epoch(self):
        fault = WheelFault(axis=1, at_s=100.0)
        assert fault.health_at(99.9) == (1.0, 1.0, 1.0)

    def test_the_wheel_is_dead_from_the_epoch_inclusive(self):
        """Inclusive on purpose: a fault landing exactly on a step
        boundary must be applied on that step rather than depending on a
        floating-point comparison to decide, which is how two supposedly
        identical replays drift apart by one step.

        Spelled with FAILURE because health is that kind's mechanism;
        SATURATION fills the wheel instead and leaves health alone.
        """
        fault = WheelFault(axis=1, at_s=100.0, kind=FAILURE)
        assert fault.health_at(100.0) == (1.0, DEAD, 1.0)
        assert fault.health_at(700.0) == (1.0, DEAD, 1.0)

    def test_only_the_faulted_axis_is_affected(self):
        assert WheelFault(axis=0, at_s=0.0, kind=FAILURE).health_at(1.0) == \
            (0.0, 1.0, 1.0)
        assert WheelFault(axis=2, at_s=0.0, kind=FAILURE).health_at(1.0) == \
            (1.0, 1.0, 0.0)

    def test_a_partial_derate_is_expressible(self):
        assert WheelFault(axis=0, at_s=0.0, health=0.5,
                          kind=FAILURE).health_at(1.0) == (0.5, 1.0, 1.0)


class TestValidation:
    @pytest.mark.parametrize("axis", [-1, 3, 99])
    def test_an_impossible_axis_is_rejected(self, axis):
        with pytest.raises(ValueError, match="axis"):
            WheelFault(axis=axis, at_s=1.0)

    @pytest.mark.parametrize("health", [-0.1, 1.5])
    def test_health_outside_the_derate_range_is_rejected(self, health):
        with pytest.raises(ValueError, match="derate"):
            WheelFault(axis=0, at_s=1.0, health=health)

    def test_a_negative_epoch_is_rejected(self):
        with pytest.raises(ValueError, match="negative"):
            WheelFault(axis=0, at_s=-1.0)

    def test_the_epoch_lands_inside_the_requested_span(self):
        for seed in range(50):
            fault = plan_fault(seed, WINDOW, earliest_s=60.0, latest_s=600.0)
            assert 60.0 <= fault.at_s <= 600.0

    def test_an_empty_span_is_rejected(self):
        with pytest.raises(ValueError, match="earliest_s"):
            plan_fault(1, WINDOW, earliest_s=500.0, latest_s=100.0)

    def test_restricting_the_eligible_axes_is_honoured(self):
        for seed in range(30):
            assert plan_fault(seed, WINDOW, axes=(1,)).axis == 1


class TestFaultKinds:
    """The two kinds do genuinely different things to the vehicle.

    Which is which was measured, not reasoned about: this module used to
    claim a dead wheel loads the healthy axes until the momentum guard
    trips. It does not.
    """

    def test_saturation_leaves_the_wheel_healthy(self):
        """It fills the wheel; it does not break it. A wheel at capacity
        can still deliver torque that unloads it, which is what keeps
        E5 a fault injection rather than a termination."""
        fault = plan_fault(3, WINDOW)
        assert fault.kind == "saturation"
        assert fault.health_at(WINDOW) == (1.0, 1.0, 1.0)
        assert fault.saturates_at(fault.at_s)

    def test_failure_derates_and_never_saturates(self):
        fault = WheelFault(axis=1, at_s=10.0, kind="failure")
        assert fault.health_at(10.0) == (1.0, DEAD, 1.0)
        assert not fault.saturates_at(10.0)

    def test_saturation_is_the_default_because_e5_specifies_it(self):
        """Case study section 9: "inject wheel saturation ... Stateflow -> SAFE"."""
        assert WheelFault(axis=0, at_s=1.0).kind == "saturation"

    def test_an_unknown_kind_is_refused(self):
        with pytest.raises(ValueError, match="kind must be one of"):
            WheelFault(axis=0, at_s=1.0, kind="jammed")

    def test_the_kind_round_trips_through_a_record(self):
        fault = WheelFault(axis=2, at_s=5.0, kind="failure")
        assert fault_from_dict(fault.to_dict()).kind == "failure"

    def test_a_record_predating_kinds_reads_as_saturation(self):
        """Absence has one correct reading: saturation was the only
        behaviour when those records were written."""
        assert fault_from_dict(
            {"axis": 0, "at_s": 1.0, "health": 0.0}).kind == "saturation"
