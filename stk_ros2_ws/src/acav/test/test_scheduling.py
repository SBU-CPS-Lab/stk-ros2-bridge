"""Tests for the kinematic vs. dynamics-aware scheduling policies.

These pin the design intent stated in ``config/acav_scenario.yaml``'s
target comments -- "a kinematic scheduler attempts all 7; a
dynamics-aware one should decline at least T5 and T7" -- as an
executable check, not a comment someone can silently invalidate by
editing the target set.
"""

import pytest

from acav.scheduling import (
    SchedulerType,
    plan,
    plan_dynamics_aware,
    plan_kinematic,
)
from acav.spec import AcavSpec


@pytest.fixture(scope="module")
def spec():
    return AcavSpec.load()


class TestKinematicAttemptsEverything:
    def test_every_target_is_attempted(self, spec):
        result = plan_kinematic(spec)
        assert result.target_ids_attempted == [t.id for t in
                                                sorted(spec.targets,
                                                      key=lambda t: t.t_image_s)]
        assert result.declined == []

    def test_activation_windows_are_contiguous(self, spec):
        """No nadir dwell between back-to-back targets -- matches
        scheduling.return_to_nadir_between_targets: false."""
        result = plan_kinematic(spec)
        for previous, current in zip(result.attempted, result.attempted[1:]):
            assert current.activation_start_s == pytest.approx(
                previous.t_image_s)

    def test_the_first_window_opens_at_the_scenario_start(self, spec):
        result = plan_kinematic(spec)
        assert result.attempted[0].activation_start_s == pytest.approx(0.0)

    def test_required_and_available_match_the_documented_margins(self, spec):
        """Cross-checks against the C1 spec's own table (section 5): T1
        needs ~17.2s with 60.0s available; T5 needs ~16.9s with only
        15.0s available, etc. These numbers are independently in the
        spec doc, the YAML comments, and here -- three places that must
        agree or the study's own documentation is lying to a reader."""
        result = plan_kinematic(spec)
        by_id = {e.target_id: e for e in result.attempted}

        expectations = {
            "T1": (17.2, 60.0), "T2": (21.5, 50.0), "T3": (16.8, 17.0),
            "T4": (19.4, 123.0), "T5": (16.9, 15.0), "T6": (19.4, 135.0),
            "T7": (15.6, 10.0),
        }
        for target_id, (need, have) in expectations.items():
            entry = by_id[target_id]
            assert entry.required_s == pytest.approx(need, abs=0.15), \
                target_id
            assert entry.available_s == pytest.approx(have, abs=0.05), \
                target_id


class TestDynamicsAwareDeclinesInfeasibleTargets:
    def test_declines_at_least_t5_and_t7(self, spec):
        """The scenario's own design intent, made executable."""
        result = plan_dynamics_aware(spec)
        assert {"T5", "T7"} <= set(result.target_ids_declined)

    def test_attempts_every_comfortable_and_knife_edge_target(self, spec):
        result = plan_dynamics_aware(spec)
        assert {"T1", "T2", "T3", "T4", "T6"} <= set(
            result.target_ids_attempted)

    def test_attempted_targets_all_have_non_negative_margin(self, spec):
        result = plan_dynamics_aware(spec)
        for entry in result.attempted:
            assert entry.margin_s >= 0.0, (
                f"{entry.target_id} was attempted with a negative margin "
                f"({entry.margin_s:.2f}s) -- the go/no-go check is broken")

    def test_declined_targets_all_have_negative_margin(self, spec):
        result = plan_dynamics_aware(spec)
        for entry in result.declined:
            assert entry.margin_s < 0.0, (
                f"{entry.target_id} was declined despite a non-negative "
                f"margin ({entry.margin_s:.2f}s) -- over-cautious")

    def test_declining_a_target_extends_the_next_one_s_available_time(
            self, spec):
        """Skip-awareness: after declining T5, the scheduler heads
        straight for T6 from T4's release point, not T5's -- so T6's
        available time here must exceed what the static, non-reactive
        feasibility_table() reports for it (which chains through T5
        regardless of whether T5 was ever attempted)."""
        result = plan_dynamics_aware(spec)
        by_id = {e.target_id: e for e in result.attempted}
        static_rows = {row["id"]: row for row in spec.feasibility_table()}

        assert by_id["T6"].available_s > static_rows["T6"]["available_s"]

    def test_activation_windows_are_contiguous_across_a_decline(self, spec):
        """T4 -> T6 (T5 declined) must still be a seamless handoff: T6's
        activation starts exactly when T4's image epoch was, not later."""
        result = plan_dynamics_aware(spec)
        by_id = {e.target_id: e for e in result.attempted}
        assert by_id["T6"].activation_start_s == pytest.approx(
            by_id["T4"].t_image_s)

    def test_attempted_and_declined_partition_the_target_list(self, spec):
        result = plan_dynamics_aware(spec)
        attempted_and_declined = (set(result.target_ids_attempted)
                                  | set(result.target_ids_declined))
        assert attempted_and_declined == {t.id for t in spec.targets}
        assert not (set(result.target_ids_attempted)
                   & set(result.target_ids_declined))


class TestActiveEntryAt:
    def test_the_first_target_is_active_from_window_open(self, spec):
        """This scenario's first activation starts exactly at t=0 (no
        target needs to wait), so the natural nadir-hold case has to be
        exercised with a constructed gap instead -- see the next test."""
        result = plan_kinematic(spec)
        first = result.attempted[0]
        assert first.activation_start_s == pytest.approx(0.0)
        assert result.active_entry_at(0.0).target_id == first.target_id

    def test_holds_at_nadir_before_an_activation_that_starts_late(self):
        """A constructed plan with a genuine gap before its one target
        opens: anything before that instant must return None."""
        from acav.scheduling import ScheduleEntry, SchedulePlan
        target = spec_fixture_target()
        plan_with_gap = SchedulePlan(
            scheduler_type=SchedulerType.KINEMATIC,
            attempted=[ScheduleEntry(target=target, activation_start_s=30.0,
                                     required_s=10.0, available_s=30.0)],
            declined=[])
        assert plan_with_gap.active_entry_at(0.0) is None
        assert plan_with_gap.active_entry_at(29.99) is None
        assert plan_with_gap.active_entry_at(30.0) is not None

    def test_returns_the_target_whose_window_contains_the_time(self, spec):
        result = plan_kinematic(spec)
        t1 = next(e for e in result.attempted if e.target_id == "T1")
        midpoint = (t1.activation_start_s + t1.t_image_s) / 2.0
        assert result.active_entry_at(midpoint).target_id == "T1"

    def test_boundaries_are_inclusive_and_handoff_is_seamless(self, spec):
        """At the exact instant one target's image epoch equals the
        next's activation start, either answer would be defensible;
        what must NOT happen is neither (a nadir gap) or a crash."""
        result = plan_kinematic(spec)
        for previous, current in zip(result.attempted, result.attempted[1:]):
            boundary = previous.t_image_s
            active = result.active_entry_at(boundary)
            assert active is not None
            assert active.target_id in (previous.target_id,
                                        current.target_id)

    def test_declined_targets_are_never_returned_as_active(self, spec):
        result = plan_dynamics_aware(spec)
        # Sample densely across the whole window; a declined target's
        # own image epoch must never come back as "active" (it has no
        # entry to be active from).
        declined_ids = set(result.target_ids_declined)
        for step in range(0, int(spec.window_duration_s) + 1):
            active = result.active_entry_at(float(step))
            if active is not None:
                assert active.target_id not in declined_ids


class TestSchedulerTypeAndDispatch:
    def test_parse_accepts_enum_strings_and_hyphens(self):
        assert SchedulerType.parse("kinematic") is SchedulerType.KINEMATIC
        assert SchedulerType.parse("dynamics_aware") is \
            SchedulerType.DYNAMICS_AWARE
        assert SchedulerType.parse("dynamics-aware") is \
            SchedulerType.DYNAMICS_AWARE
        assert SchedulerType.parse(SchedulerType.KINEMATIC) is \
            SchedulerType.KINEMATIC

    def test_parse_rejects_garbage(self):
        with pytest.raises(ValueError, match="unknown scheduler type"):
            SchedulerType.parse("greedy")

    def test_plan_dispatches_to_the_right_policy(self, spec):
        kinematic = plan(spec, "kinematic")
        dynamics_aware = plan(spec, "dynamics_aware")
        assert kinematic.scheduler_type is SchedulerType.KINEMATIC
        assert dynamics_aware.scheduler_type is SchedulerType.DYNAMICS_AWARE
        assert len(kinematic.attempted) > len(dynamics_aware.attempted)


class TestSummary:
    def test_summary_mentions_every_target(self, spec):
        result = plan_dynamics_aware(spec)
        text = result.summary()
        for target in spec.targets:
            assert target.id in text
        assert "DECLINED" in text


def spec_fixture_target():
    from acav.spec import TargetSpec
    return TargetSpec(id="ZZ", t_image_s=60.0, cross_track_km=0.0,
                      priority=1.0)
