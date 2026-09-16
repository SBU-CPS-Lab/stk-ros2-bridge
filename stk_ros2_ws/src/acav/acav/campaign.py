"""Run matrices and aggregation for the E2-E5 campaigns.

The pure half of C5. This module decides *what runs to do* and *what the
results mean*; it never launches anything, opens a socket or imports
ROS. :mod:`acav.campaign_runner` is the thin layer that executes a
matrix built here, in the same division C4 already uses between
``scheduling.py`` and ``scheduler_node.py``.

Keeping it pure is not tidiness. A campaign is 40 runs for E3, 49 for
E4 and 100+ for E5, each needing STK and MATLAB, so a campaign is a
thing you run overnight on the Windows machine and cannot iterate on.
Everything about it that can be wrong *before* it starts -- which runs,
which seeds, which gains, which fault, and whether the results support
the claim afterwards -- is decided by code that runs in milliseconds on
any machine and is covered by tests here.

Why run identifiers are constructed, not counted
------------------------------------------------
Every :class:`RunSpec` gets an id derived from what makes it distinct
(experiment, scheduler, mode, seed, gains), not from its position in the
list. Positional ids renumber when a run is inserted or a seed is added,
which silently repoints every result file written by an earlier
campaign; an id derived from the configuration stays attached to the
configuration it describes. It is also what lets a campaign resume: the
runner can tell which runs are already on disk only if the name of a
run's output is a function of the run.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from acav.fault_injection import WheelFault, plan_campaign_faults
from acav.metrics import (
    capture_rate,
    coverage_rate,
    determinism_report,
    outcome_breakdown,
    over_promise_gap,
)
from acav.run_record import RunRecord

#: The execution modes E3 contrasts. Names match the bridge's own
#: ``time.mode`` setting and ``stk_msgs/ScenarioTime``'s constants, so a
#: campaign cannot ask for a mode the bridge does not have.
LOCKSTEP = "lockstep"
FREE_RUN = "free_run"

#: Gain keys, matching ``TwinConfig`` field names exactly. Spelling them
#: the same on both sides means E4's sweep can be applied to the twin
#: without a translation table that could drift.
GAIN_KEYS = ("natural_frequency_rad_s", "damping_ratio", "profile_authority")


@dataclass(frozen=True)
class RunSpec:
    """One run's configuration: everything that varies between runs.

    Anything constant across a campaign (the scenario file, the backend,
    the window) lives on :class:`Campaign` instead, so that a diff
    between two runs shows only what the experiment actually varied.
    """

    experiment: str
    scheduler: str = "dynamics_aware"
    execution_mode: str = LOCKSTEP
    seed: Optional[int] = None
    fault: Optional[WheelFault] = None
    gains: Dict[str, float] = field(default_factory=dict)
    #: E1's independent variable: force the bridge to read attitude back
    #: through this provider instead of the validated one. Empty means
    #: the validated pairing, which is what every other experiment uses.
    #: The name is passed to the bridge as
    #: ``sensing.force_attitude_provider``.
    attitude_provider: str = ""
    #: Set on a run that re-executes an earlier one to test replay (E5).
    replay_of: str = ""
    notes: str = ""

    def __post_init__(self) -> None:
        unknown = set(self.gains) - set(GAIN_KEYS)
        if unknown:
            raise ValueError(
                f"unknown gain(s) {sorted(unknown)}; expected a subset of "
                f"{list(GAIN_KEYS)}. A gain the twin does not have would "
                f"be accepted silently and swept over with no effect.")

    @property
    def run_id(self) -> str:
        """A name derived from the configuration, stable under insertion."""
        parts = [self.experiment, self.scheduler, self.execution_mode]
        for key in GAIN_KEYS:
            if key in self.gains:
                parts.append(f"{_abbreviate(key)}{_tag(self.gains[key])}")
        if self.seed is not None:
            parts.append(f"seed{self.seed}")
        if self.fault is not None:
            parts.append(f"fault{self.fault.axis_name}"
                         f"{_tag(self.fault.at_s)}")
        if self.attitude_provider:
            parts.append("readback" + _slug(self.attitude_provider))
        if self.replay_of:
            parts.append("replay")
        return "-".join(parts)

    @property
    def output_name(self) -> str:
        return f"{self.run_id}.json"


def _abbreviate(gain_key: str) -> str:
    return {"natural_frequency_rad_s": "wn",
            "damping_ratio": "zeta",
            "profile_authority": "auth"}[gain_key]


def _slug(text: str) -> str:
    """A filename-safe token from a provider name."""
    return "".join(c for c in text.title() if c.isalnum())


def _tag(value: float) -> str:
    """A filename-safe number: 1.0 -> 1p0, 0.707 -> 0p707."""
    return f"{value:g}".replace(".", "p").replace("-", "m")


@dataclass(frozen=True)
class Campaign:
    """A named set of runs and the configuration they share."""

    campaign_id: str
    experiment: str
    description: str
    runs: Tuple[RunSpec, ...]
    scenario_path: str = ""
    backend: str = "com"

    def __post_init__(self) -> None:
        seen: Dict[str, int] = {}
        for run in self.runs:
            seen[run.run_id] = seen.get(run.run_id, 0) + 1
        clashes = sorted(k for k, count in seen.items() if count > 1)
        if clashes:
            raise ValueError(
                f"duplicate run id(s): {clashes}. Two runs sharing an id "
                f"would write to one file, so the campaign would look "
                f"complete while silently holding fewer results than it "
                f"claims -- which for E3 means comparing a repeat count "
                f"that never happened.")

    def __len__(self) -> int:
        return len(self.runs)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "campaign_id": self.campaign_id,
            "experiment": self.experiment,
            "description": self.description,
            "scenario_path": self.scenario_path,
            "backend": self.backend,
            "run_count": len(self.runs),
            "runs": [
                {
                    "run_id": run.run_id,
                    "experiment": run.experiment,
                    "scheduler": run.scheduler,
                    "execution_mode": run.execution_mode,
                    "seed": run.seed,
                    "gains": dict(run.gains),
                    "fault": None if run.fault is None else run.fault.to_dict(),
                    "attitude_provider": run.attitude_provider,
                    "replay_of": run.replay_of,
                    "notes": run.notes,
                }
                for run in self.runs
            ],
        }


# ---------------------------------------------------------------------------
# The matrices
# ---------------------------------------------------------------------------

#: The inertial read-back provider. Paired against the central-body-
#: fixed command form the actuation path uses, it is wrong by the
#: Earth-rotation angle -- the bug found during the bridge's own
#: bring-up, and the treatment E1 applies deliberately.
MISPAIRED_ATTITUDE_PROVIDER = "Attitude Quaternions"


def build_e1_convention(scenario_path: str = "", backend: str = "com",
                        scheduler: str = "dynamics_aware") -> Campaign:
    """E1: does convention validation matter?

    Two runs, identical but for the attitude read-back provider. The
    treatment pairs an inertial provider against a central-body-fixed
    command, which is wrong by the Earth-rotation angle -- and which
    raises nothing, reports no error and produces a plausible attitude.
    That silence is the finding: the expected result is that capture
    collapses while every component reports itself healthy.

    Requires the bridge to honour ``sensing.force_attitude_provider``.
    """
    runs = (
        RunSpec(experiment="E1", scheduler=scheduler,
                execution_mode=LOCKSTEP,
                notes="control: the validated command/read-back pairing"),
        RunSpec(experiment="E1", scheduler=scheduler,
                execution_mode=LOCKSTEP,
                attitude_provider=MISPAIRED_ATTITUDE_PROVIDER,
                notes="treatment: inertial read-back against a CBFQuat "
                      "command -- wrong by the Earth-rotation angle, and "
                      "nothing errors"),
    )
    return Campaign(
        campaign_id="E1_convention_validation",
        experiment="E1",
        description=("Nominal run against one with the attitude read-back "
                     "deliberately mis-paired. Expect capture to collapse "
                     "with no component reporting a fault."),
        runs=runs, scenario_path=scenario_path, backend=backend)


def build_e2_scheduler_comparison(scenario_path: str = "",
                                  backend: str = "com",
                                  repeats: int = 1) -> Campaign:
    """E2: the headline. Same targets, same vehicle, two schedulers.

    Repeats default to one because E3 is what establishes that a repeat
    adds nothing in lockstep. Running E2 many times *before* that has
    been shown would be averaging over a variance nobody has yet
    demonstrated exists.
    """
    runs = []
    for index in range(max(1, repeats)):
        seed = None if repeats == 1 else index
        for scheduler in ("kinematic", "dynamics_aware"):
            runs.append(RunSpec(experiment="E2", scheduler=scheduler,
                                execution_mode=LOCKSTEP, seed=seed))
    return Campaign(
        campaign_id="E2_scheduler_comparison",
        experiment="E2",
        description=("Kinematic vs dynamics-aware scheduling over the same "
                     "target set. Capture rate, coverage rate and the "
                     "over-promise gap between them."),
        runs=tuple(runs), scenario_path=scenario_path, backend=backend)


def build_e3_determinism(repeats: int = 20, scenario_path: str = "",
                         backend: str = "com",
                         scheduler: str = "dynamics_aware") -> Campaign:
    """E3: ``repeats`` runs in lockstep against ``repeats`` in free-run.

    The configuration is identical across all of them except the
    execution mode, which is the point: any difference in the resulting
    signatures is attributable to the mode and nothing else. The seed is
    set and equal for every run rather than left unset, so that if a
    free-run batch *does* come out identical it is because the loop was
    deterministic and not because nothing in it varied.
    """
    if repeats < 2:
        raise ValueError(
            "E3 compares a spread across repeats; one run has no spread")

    runs = []
    for mode in (LOCKSTEP, FREE_RUN):
        for index in range(repeats):
            runs.append(RunSpec(experiment="E3", scheduler=scheduler,
                                execution_mode=mode, seed=index,
                                notes="identical configuration; "
                                      "execution mode is the variable"))
    return Campaign(
        campaign_id=f"E3_determinism_{repeats}x2",
        experiment="E3",
        description=(f"{repeats} lockstep runs against {repeats} free-run "
                     f"runs, identical configuration. Expect one distinct "
                     f"determinism signature in lockstep and many in "
                     f"free-run."),
        runs=tuple(runs), scenario_path=scenario_path, backend=backend)


def build_e4_gain_sweep(frequencies: Sequence[float] = (),
                        authorities: Sequence[float] = (),
                        scenario_path: str = "", backend: str = "com",
                        scheduler: str = "dynamics_aware") -> Campaign:
    """E4: capture metric over a grid of controller gains.

    Defaults to the 7x7 grid C1's compute budget sizes (49 runs), spanning
    the bandwidth and profile-authority axes C3 actually measured when it
    chose w_n = 1.0 and authority = 0.8. Damping is not swept: C3 fixed
    zeta = 0.707 for the standard reason, and adding a third axis would
    take the campaign from an overnight run to a weekend one for a
    parameter the design does not treat as free.

    E4 is only interpretable because E3 established that a single run is
    noise-free -- otherwise each grid point would need repeats and the
    surface would be measuring variance rather than gain.
    """
    frequencies = tuple(frequencies) or (0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6)
    authorities = tuple(authorities) or (0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0)

    runs = []
    for frequency in frequencies:
        for authority in authorities:
            runs.append(RunSpec(
                experiment="E4", scheduler=scheduler,
                execution_mode=LOCKSTEP,
                gains={"natural_frequency_rad_s": frequency,
                       "profile_authority": authority}))
    return Campaign(
        campaign_id="E4_gain_sweep",
        experiment="E4",
        description=(f"{len(frequencies)}x{len(authorities)} sweep of "
                     f"closed-loop bandwidth against profile authority, "
                     f"scored by E2's capture metric."),
        runs=tuple(runs), scenario_path=scenario_path, backend=backend)


def build_e5_fault_campaign(seeds: Sequence[int], window_duration_s: float,
                            scenario_path: str = "", backend: str = "com",
                            scheduler: str = "dynamics_aware",
                            **fault_kwargs) -> Campaign:
    """E5: one seeded wheel fault per seed.

    The fault for each seed is resolved here, before anything runs, and
    travels with the run into its record. Deciding it at injection time
    would leave nothing to replay against -- see
    :mod:`acav.fault_injection`.
    """
    if not seeds:
        raise ValueError("E5 needs at least one seed")

    faults = plan_campaign_faults(seeds, window_duration_s, **fault_kwargs)
    runs = [RunSpec(experiment="E5", scheduler=scheduler,
                    execution_mode=LOCKSTEP, seed=seed, fault=faults[seed])
            for seed in seeds]
    return Campaign(
        campaign_id=f"E5_fault_replay_{len(seeds)}seeds",
        experiment="E5",
        description=(f"{len(seeds)} Monte Carlo runs, each with one "
                     f"reaction wheel failing at a seeded epoch. Expect "
                     f"SAFE on the momentum guard, and bit-identical "
                     f"reproduction on replay."),
        runs=tuple(runs), scenario_path=scenario_path, backend=backend)


def build_e6_latency(repeats: int = 5, scenario_path: str = "",
                     backend: str = "com",
                     scheduler: str = "dynamics_aware") -> Campaign:
    """E6: what does the loop cost, broken down by phase.

    Identical lockstep runs, repeated -- not to compare anything, but
    because a single run's mean can catch one slow moment (a GC pause,
    a disk stall) that a few repeats average out. C1 section 8 commits
    the paper to reporting a MEASURED latency breakdown rather than an
    implied step rate, which is what makes this experiment about
    transparency rather than a performance claim.
    """
    if repeats < 1:
        raise ValueError("E6 needs at least one run to measure anything")
    runs = tuple(
        RunSpec(experiment="E6", scheduler=scheduler, execution_mode=LOCKSTEP,
               seed=index, notes="identical configuration, repeated for a "
                                  "stable latency mean")
        for index in range(repeats))
    return Campaign(
        campaign_id=f"E6_latency_{repeats}x",
        experiment="E6",
        description=(f"{repeats} identical lockstep run(s), measuring the "
                     f"bridge's own sense/actuate/barrier-wait breakdown "
                     f"and the achieved step rate."),
        runs=runs, scenario_path=scenario_path, backend=backend)


def build_e5_replay(records: Sequence[RunRecord],
                    scenario_path: str = "",
                    backend: str = "com") -> Campaign:
    """Re-run the given records with the faults they actually carried.

    E5's claim is that a *failing* run can be reproduced exactly, so the
    replay must use the fault recorded for that run rather than
    re-deriving it from the seed. Re-deriving would make every historical
    replay depend on the current sampling rule, and a change to that rule
    would turn a genuine reproduction into a silent mismatch.
    """
    from acav.fault_injection import fault_from_dict

    runs = []
    for record in records:
        provenance = record.provenance
        runs.append(RunSpec(
            experiment="E5",
            scheduler=record.result.scheduler or "dynamics_aware",
            execution_mode=provenance.execution_mode or LOCKSTEP,
            seed=record.result.seed,
            fault=fault_from_dict(provenance.fault),
            gains=dict(provenance.gains),
            replay_of=record.result.run_id,
            notes=f"replay of {record.result.run_id}"))
    if not runs:
        raise ValueError("nothing to replay")
    return Campaign(
        campaign_id="E5_replay",
        experiment="E5",
        description=("Replay of selected E5 runs using the fault each one "
                     "actually carried, to test bit-identical "
                     "reproduction."),
        runs=tuple(runs), scenario_path=scenario_path, backend=backend)


# ---------------------------------------------------------------------------
# Aggregation: what the results mean
# ---------------------------------------------------------------------------

def determinism_comparison(records: Sequence[RunRecord]) -> Dict[str, Any]:
    """E3's answer: a determinism report per execution mode.

    Grouped by the mode the bridge actually ran in, taken from the
    record rather than from the campaign that requested it. A run that
    was asked for lockstep and fell back to free-run belongs in the
    free-run group, or E3 would report determinism for a batch that was
    not all lockstep.
    """
    grouped: Dict[str, List[RunRecord]] = {}
    for record in records:
        grouped.setdefault(record.provenance.execution_mode, []).append(record)

    report: Dict[str, Any] = {}
    for mode, group in sorted(grouped.items()):
        summary = determinism_report([r.result for r in group])
        summary["signatures"] = sorted({r.signature for r in group})
        report[mode] = summary
    return report


def gain_surface(records: Sequence[RunRecord],
                 total_targets: int) -> List[Dict[str, Any]]:
    """E4's surface: one point per gain combination, sorted for plotting."""
    points = []
    for record in records:
        points.append({
            "gains": dict(record.provenance.gains),
            "run_id": record.result.run_id,
            "capture_rate": capture_rate(record.result),
            "coverage_rate": coverage_rate(record.result, total_targets),
            "attempted": record.result.attempted_count,
            "captured": record.result.captured_count,
        })
    return sorted(points, key=lambda p: tuple(
        p["gains"].get(key, 0.0) for key in GAIN_KEYS))


@dataclass(frozen=True)
class ReplayCheck:
    """Whether one replay reproduced its original."""

    original_run_id: str
    replay_run_id: str
    reproduced: bool
    detail: str = ""


def replay_matches(original: RunRecord, replay: RunRecord) -> ReplayCheck:
    """Compare a replay against its original by determinism signature.

    The signature, not the capture count: two runs can agree on how many
    targets they captured while disagreeing about every pointing error,
    and E5's claim is reproduction rather than similarity.

    A replay that ran a different fault is reported as not reproduced
    even if the signatures happen to match, because in that case the
    match says nothing about replay -- it says the fault did not matter.
    """
    if original.provenance.fault != replay.provenance.fault:
        return ReplayCheck(
            original.result.run_id, replay.result.run_id, False,
            f"different fault: {original.provenance.fault} vs "
            f"{replay.provenance.fault}")

    same = original.signature == replay.signature
    return ReplayCheck(
        original.result.run_id, replay.result.run_id, same,
        "" if same else "signatures differ")


def compare_schedulers(records: Sequence[RunRecord],
                       total_targets: int) -> Dict[str, Any]:
    """E2's answer: the two policies side by side, plus the gap."""
    by_scheduler: Dict[str, List[RunRecord]] = {}
    for record in records:
        by_scheduler.setdefault(record.result.scheduler, []).append(record)

    summary: Dict[str, Any] = {}
    for scheduler, group in sorted(by_scheduler.items()):
        summary[scheduler] = {
            "runs": len(group),
            "capture_rate": _mean(capture_rate(r.result) for r in group),
            "coverage_rate": _mean(coverage_rate(r.result, total_targets)
                                   for r in group),
            "over_promise_gap": _mean(over_promise_gap(r.result)
                                      for r in group),
            "attempted": _mean(float(r.result.attempted_count)
                               for r in group),
            "captured": _mean(float(r.result.captured_count) for r in group),
            "outcomes": _merge_counts(outcome_breakdown(r.result)
                                      for r in group),
        }

    if {"kinematic", "dynamics_aware"} <= set(summary):
        summary["coverage_difference"] = (
            summary["dynamics_aware"]["coverage_rate"]
            - summary["kinematic"]["coverage_rate"])
    return summary


def latency_breakdown(records: Sequence[RunRecord]) -> Dict[str, Any]:
    """E6's answer: the phase breakdown and the achieved step rate.

    Unreportable runs (see acav.run_record) are excluded up front,
    matching every other aggregation function here -- a latency number
    measured against the mock says nothing about the deploy machine's
    real STK round trip, which is the entire quantity E6 exists to
    report.
    """
    usable = [r for r in records if r.reportable]
    return {
        "runs": len(usable),
        "excluded_unreportable": len(records) - len(usable),
        "mean_step_latency_ms": _mean(
            r.result.mean_step_latency_ms for r in usable),
        "mean_sense_latency_ms": _mean(
            r.result.mean_sense_latency_ms for r in usable),
        "mean_actuate_latency_ms": _mean(
            r.result.mean_actuate_latency_ms for r in usable),
        "mean_barrier_wait_ms": _mean(
            r.result.mean_barrier_wait_ms for r in usable),
    }


def convention_comparison(records: Sequence[RunRecord],
                          total_targets: int) -> Dict[str, Any]:
    """E1's answer: capture rate with vs without the mis-paired provider.

    Split by whether ``Provenance.attitude_provider`` was forced, NOT by
    scheduler -- both of E1's runs use the same scheduler on purpose
    (see :func:`build_e1_convention`'s own docstring: only the read-back
    provider is meant to vary), so grouping by scheduler the way
    :func:`compare_schedulers` does would collapse E1's two runs into
    one group and hide exactly the comparison the experiment exists to
    make.
    """
    control = [r for r in records if not r.provenance.attitude_provider]
    treatment = [r for r in records if r.provenance.attitude_provider]

    def _group(group: List[RunRecord]) -> Dict[str, Any]:
        return {
            "runs": len(group),
            "capture_rate": _mean(capture_rate(r.result) for r in group),
            "coverage_rate": _mean(coverage_rate(r.result, total_targets)
                                   for r in group),
            "outcomes": _merge_counts(outcome_breakdown(r.result)
                                      for r in group),
        }

    summary: Dict[str, Any] = {}
    if control:
        summary["control"] = _group(control)
    if treatment:
        summary["treatment"] = _group(treatment)
        summary["forced_provider"] = sorted({
            r.provenance.attitude_provider for r in treatment})
    return summary


def _mean(values: Iterable[float]) -> float:
    """Mean over the values that are not NaN; NaN when none are.

    NaN here means "undefined", not "zero" -- ``capture_rate`` returns it
    when nothing was attempted. Treating it as zero would drag a campaign
    average down with runs that made no claim either way.
    """
    numbers = [v for v in values if not math.isnan(v)]
    return float("nan") if not numbers else sum(numbers) / len(numbers)


def _merge_counts(dicts: Iterable[Dict[str, int]]) -> Dict[str, int]:
    merged: Dict[str, int] = {}
    for entry in dicts:
        for key, count in entry.items():
            merged[key] = merged.get(key, 0) + count
    return merged
