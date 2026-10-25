"""Target sequencing -- the kinematic vs. dynamics-aware scheduler pair.

This module is the entirety of what makes the two schedulers in E2
different. Both walk the target list in image-time order and switch the
guidance target ("point the boresight here now") at exactly the same
calendar instants -- as soon as the vehicle is done with whatever it was
previously pointed at. Neither scheduler starts a target early "just in
case"; both give every attempted target the full gap the pass geometry
allows. The ONLY difference is the go/no-go decision:

* **Kinematic** attempts every target, unconditionally. This is the
  textbook "assumes instantaneous, perfect pointing" planning
  assumption taken literally: since a reorientation is believed to cost
  no time, there is no reason to ever decline one.
* **Dynamics-aware** declines a target when the shared
  ``AcavSpec.estimated_slew_time_s`` bound -- the same lower-bound model
  the twin's reference profile generator is built from (see
  ``acav/twin_reference.py``) -- exceeds the time actually available
  before it. Declining does not spend the gap: the vehicle keeps
  heading toward whatever comes next, so a declined target can hand its
  slot to the target after it.

Why this must not just re-run ``AcavSpec.feasibility_table()``
----------------------------------------------------------------
That table is deliberately static and non-reactive (see its own
docstring): it always chains from one target to the next in the
scenario list regardless of whether either was actually attempted, and
that is correct for its purpose -- an at-a-glance planning summary
computed once, before any run. A live scheduler is different: once it
declines a target, the "previous target" for computing the NEXT one's
margin has to be whatever was last *actually pointed at*, not
whatever was merely next on the list. For this scenario's target set the two
give the same attempt/decline verdicts (checked in
``test_scheduling.py``), but they are not the same computation, and a
scenario with a different target set could make them disagree.

Nothing here touches ROS or STK. It is pure arithmetic over
``AcavSpec``, exercised entirely offline -- consistent with C1 section 8
point 3: development and the scheduler's own feasibility reasoning need
neither STK nor the mock backend, only the shared model.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Sequence

from acav.spec import AcavSpec, TargetSpec


class SchedulerType(str, Enum):
    """The two policies E2 compares."""

    KINEMATIC = "kinematic"
    DYNAMICS_AWARE = "dynamics_aware"

    @classmethod
    def parse(cls, value) -> "SchedulerType":
        if isinstance(value, cls):
            return value
        key = str(value).strip().lower().replace("-", "_")
        try:
            return cls(key)
        except ValueError:
            raise ValueError(
                f"unknown scheduler type {value!r}; valid values are "
                f"{[member.value for member in cls]}"
            ) from None


@dataclass(frozen=True)
class ScheduleEntry:
    """One target the scheduler has decided to attempt.

    ``activation_start_s`` is when the guidance target switches to this
    entry -- immediately after the previous ATTEMPTED entry's
    ``t_image_s`` (or window start, for the first). The active window is
    therefore ``[activation_start_s, t_image_s]``, and consecutive
    entries are contiguous by construction: there is never a nadir dwell
    between two attempted targets, matching
    ``scheduling.return_to_nadir_between_targets: false``.
    """

    target: TargetSpec
    activation_start_s: float
    required_s: float
    available_s: float

    @property
    def target_id(self) -> str:
        return self.target.id

    @property
    def t_image_s(self) -> float:
        return self.target.t_image_s

    @property
    def margin_s(self) -> float:
        return self.available_s - self.required_s


@dataclass(frozen=True)
class DeclinedEntry:
    """One target the dynamics-aware scheduler chose not to attempt."""

    target: TargetSpec
    required_s: float
    available_s: float

    @property
    def target_id(self) -> str:
        return self.target.id

    @property
    def margin_s(self) -> float:
        return self.available_s - self.required_s


@dataclass(frozen=True)
class SchedulePlan:
    """The complete output of one scheduler policy over one spec."""

    scheduler_type: SchedulerType
    attempted: List[ScheduleEntry]
    declined: List[DeclinedEntry]

    @property
    def target_ids_attempted(self) -> List[str]:
        return [entry.target_id for entry in self.attempted]

    @property
    def target_ids_declined(self) -> List[str]:
        return [entry.target_id for entry in self.declined]

    def active_entry_at(self, sim_time_s: float) -> Optional[ScheduleEntry]:
        """Which attempted target the boresight should be on right now.

        ``None`` means hold at nadir: either before the first target's
        activation, or (should the schedule ever leave a gap) between
        two entries. Entries are assumed sorted by ``t_image_s``, which
        both planners below produce them in.
        """
        for entry in self.attempted:
            if entry.activation_start_s <= sim_time_s <= entry.t_image_s:
                return entry
        return None

    def summary(self) -> str:
        lines = [f"schedule ({self.scheduler_type.value}): "
                 f"{len(self.attempted)} attempted, "
                 f"{len(self.declined)} declined"]
        for entry in self.attempted:
            lines.append(
                f"    {entry.target_id:<3} "
                f"active [{entry.activation_start_s:6.1f}, "
                f"{entry.t_image_s:6.1f}] s  "
                f"need={entry.required_s:5.1f}s  "
                f"have={entry.available_s:6.1f}s  "
                f"margin={entry.margin_s:+6.1f}s")
        for entry in self.declined:
            lines.append(
                f"    {entry.target_id:<3} DECLINED  "
                f"need={entry.required_s:5.1f}s  "
                f"have={entry.available_s:6.1f}s  "
                f"margin={entry.margin_s:+6.1f}s")
        return "\n".join(lines)


def _ordered_targets(spec: AcavSpec) -> Sequence[TargetSpec]:
    return sorted(spec.targets, key=lambda t: t.t_image_s)


def plan_kinematic(spec: AcavSpec) -> SchedulePlan:
    """Attempt every target, back to back, with no feasibility check.

    "need" and "have" are still computed and attached to each entry --
    the kinematic policy does not USE them to decide anything, but
    reporting them is what lets E2's analysis show, for a target that
    failed, whether the real vehicle was ever going to make it (the
    story the paper wants) rather than just a bare pass/fail.
    """
    attempted: List[ScheduleEntry] = []
    previous_cross_track_km = 0.0     # start at nadir
    previous_release_s = 0.0          # window start

    for target in _ordered_targets(spec):
        required = spec.estimated_slew_time_s(previous_cross_track_km,
                                              target.cross_track_km)
        available = target.t_image_s - previous_release_s
        attempted.append(ScheduleEntry(
            target=target, activation_start_s=previous_release_s,
            required_s=required, available_s=available))
        previous_cross_track_km = target.cross_track_km
        previous_release_s = target.t_image_s

    return SchedulePlan(scheduler_type=SchedulerType.KINEMATIC,
                        attempted=attempted, declined=[])


def plan_dynamics_aware(spec: AcavSpec) -> SchedulePlan:
    """Attempt a target only if the shared slew-time bound fits the gap.

    Skip-aware: declining a target leaves ``previous_cross_track_km``
    and ``previous_release_s`` unchanged, so the NEXT target's "have" is
    measured from the last target actually pointed at -- the vehicle
    starts heading toward it immediately rather than idling through the
    declined target's slot.
    """
    attempted: List[ScheduleEntry] = []
    declined: List[DeclinedEntry] = []
    previous_cross_track_km = 0.0
    previous_release_s = 0.0

    for target in _ordered_targets(spec):
        required = spec.estimated_slew_time_s(previous_cross_track_km,
                                              target.cross_track_km)
        available = target.t_image_s - previous_release_s

        if available >= required:
            attempted.append(ScheduleEntry(
                target=target, activation_start_s=previous_release_s,
                required_s=required, available_s=available))
            previous_cross_track_km = target.cross_track_km
            previous_release_s = target.t_image_s
        else:
            declined.append(DeclinedEntry(
                target=target, required_s=required, available_s=available))

    return SchedulePlan(scheduler_type=SchedulerType.DYNAMICS_AWARE,
                        attempted=attempted, declined=declined)


_PLANNERS = {
    SchedulerType.KINEMATIC: plan_kinematic,
    SchedulerType.DYNAMICS_AWARE: plan_dynamics_aware,
}


def plan(spec: AcavSpec, scheduler_type) -> SchedulePlan:
    """Dispatch to the planner named by ``scheduler_type``.

    Accepts a :class:`SchedulerType`, or anything
    :meth:`SchedulerType.parse` understands (a plain string, as arrives
    from a ROS parameter or a CLI argument).
    """
    return _PLANNERS[SchedulerType.parse(scheduler_type)](spec)
