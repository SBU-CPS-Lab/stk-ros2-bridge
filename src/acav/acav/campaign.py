"""Run matrices and aggregation for the E1, E2, E3, E5 and E6 campaigns.

The pure half of C5. This module decides *what runs to do* and *what the
results mean*; it never launches anything, opens a socket or imports
ROS. :mod:`acav.campaign_runner` is the thin layer that executes a
matrix built here, in the same division C4 already uses between
``scheduling.py`` and ``scheduler_node.py``.

Keeping it pure is not tidiness. A campaign is tens of runs, each needing
STK and MATLAB for 10-16 minutes, so a campaign is a thing you run
overnight on the Windows machine and cannot iterate on.
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
#: the same on both sides means per-run gains reach the twin without a
#: translation table that could drift.
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
    #: E1's mismatch type (E1_MISMATCH_TYPES), empty for the control and
    #: every other experiment. frame_pairing is attitude_provider above;
    #: the others are injected by the bridge (sensing.inject_mismatch).
    mismatch: str = ""
    #: E1: "on" (the twin's attitude guard active) or "ablated" (only
    #: frame_pairing has that arm); empty means the original E1
    #: treatment's rule (ablated whenever attitude_provider is forced).
    mismatch_guard: str = ""
    #: The bridge's time.real_time_factor for this run (E3's matrix);
    #: None leaves the bridge config's. In the run id only when not 1.0.
    real_time_factor: Optional[float] = None
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
        if self.real_time_factor is not None and self.real_time_factor != 1.0:
            parts.append(f"rtf{_tag(self.real_time_factor)}")
        if self.fault is not None:
            parts.append(f"fault{self.fault.axis_name}"
                         f"{_tag(self.fault.at_s)}")
        if self.attitude_provider:
            parts.append("readback" + _slug(self.attitude_provider))
        if self.mismatch and self.mismatch != "frame_pairing":
            parts.append("mismatch" + _slug(self.mismatch.replace("_", " ")))
        if self.mismatch == "frame_pairing" and self.mismatch_guard == "on":
            parts.append("guardon")
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
                    "mismatch": run.mismatch,
                    "mismatch_guard": run.mismatch_guard,
                    "real_time_factor": run.real_time_factor,
                    "replay_of": run.replay_of,
                    "notes": run.notes,
                }
                for run in self.runs
            ],
        }


# ---------------------------------------------------------------------------
# The matrices
# ---------------------------------------------------------------------------

#: The Earth-fixed read-back provider. The ACAV loop commands attitude
#: in the inertial form (``AddAttitude Quat``, see
#: ``config/bridge_params_acav.yaml``), so reading it back through the
#: Earth-fixed provider is wrong by the Earth-rotation angle -- the bug
#: found during the bridge's own bring-up, and the treatment E1 applies
#: deliberately. (It was "Attitude Quaternions" while ACAV still
#: commanded in the Earth-fixed form; the mis-pairing is always the
#: provider that does NOT match the command form.)
MISPAIRED_ATTITUDE_PROVIDER = "ECF Attitude Quaternions"


def build_e1_convention(scenario_path: str = "", backend: str = "com",
                        scheduler: str = "dynamics_aware") -> Campaign:
    """E1: does convention validation matter?

    Two runs, identical but for the attitude read-back provider. The
    treatment pairs an Earth-fixed provider against an inertial
    command, which is wrong by the Earth-rotation angle -- and which
    produces a plausible but incorrect attitude. The treatment explicitly
    ablates the twin's mismatch guard and feeds measured attitude back;
    the control retains the guard. This is a deliberately unsafe-convention
    counterfactual, not the behaviour of the protected nominal pipeline.

    Requires the bridge to honour ``sensing.force_attitude_provider``.
    """
    runs = (
        RunSpec(experiment="E1", scheduler=scheduler,
                execution_mode=LOCKSTEP,
                notes="control: the validated command/read-back pairing"),
        RunSpec(experiment="E1", scheduler=scheduler,
                execution_mode=LOCKSTEP,
                attitude_provider=MISPAIRED_ATTITUDE_PROVIDER,
                notes="treatment: Earth-fixed read-back against an "
                      "inertial (Quat) command -- wrong by the "
                      "Earth-rotation angle; explicit twin mismatch-guard ablation"),
    )
    return Campaign(
        campaign_id="E1_convention_validation",
        experiment="E1",
        description=("Nominal run against one with the attitude read-back "
                     "deliberately mis-paired. Expect capture to collapse "
                     "under an explicit mismatch-guard ablation; nominal runs retain fail-fast protection."),
        runs=runs, scenario_path=scenario_path, backend=backend)


#: E1's interface-mismatch types. frame_pairing is the original treatment
#: (a forced read-back provider) and the only one with a guard-ablated
#: arm; the other five are injected by the bridge on the telemetry it
#: publishes (stk_bridge/core/mismatch_injection.py) and run guard-on.
E1_MISMATCH_TYPES = ("frame_pairing", "quaternion_order",
                     "quaternion_conjugate", "angle_rate_units",
                     "distance_units", "time_offset")
#: The expected-null control: the twin takes STK's body rate only to
#: initialise, then integrates its own (simulink_step.m), so a rate in
#: the wrong unit should change nothing. Reported without a harm claim.
E1_EXPECTED_NULL = ("angle_rate_units",)


def e1_arm(mismatch: str, mismatch_guard: str,
           attitude_provider: str) -> str:
    """The E1 arm a run or record belongs to: "control",
    "<type>/on" or "frame_pairing/ablated". A record from before the
    mismatch fields existed with a forced provider is the original
    treatment, frame_pairing with the guard ablated."""
    if not mismatch:
        return "frame_pairing/ablated" if attitude_provider else "control"
    if mismatch == "frame_pairing":
        return f"frame_pairing/{mismatch_guard or 'ablated'}"
    return f"{mismatch}/{mismatch_guard or 'on'}"


def e1_run_count(types: Sequence[str], repeats: int) -> int:
    """repeats x (1 control + 2 if frame_pairing + 1 per other type)."""
    frame = 2 if "frame_pairing" in types else 0
    others = len([t for t in types if t != "frame_pairing"])
    return repeats * (1 + frame + others)


def e1_formula_line(types: Sequence[str], repeats: int) -> str:
    frame = 2 if "frame_pairing" in types else 0
    others = len([t for t in types if t != "frame_pairing"])
    per = 1 + frame + others
    return (f"E1 runs = N_REPEATS x (1 control + {frame} frame_pairing + "
            f"{others} other types) = {repeats} x {per} = {repeats * per}")


def build_e1_matrix(types: Sequence[str] = E1_MISMATCH_TYPES,
                    repeats: int = 1, scenario_path: str = "",
                    backend: str = "com",
                    scheduler: str = "dynamics_aware") -> Campaign:
    """E1 over several interface mismatches, guard on unless ablated.

    Per repeat: the control; frame_pairing guard-on and guard-ablated;
    one guard-on run per other type. Lockstep, one scheduler, the same
    scenario: only the mismatch varies. With one repeat the control and
    the frame_pairing guard-ablated run keep build_e1_convention's ids.
    """
    unknown = [t for t in types if t not in E1_MISMATCH_TYPES]
    if unknown:
        raise ValueError(f"unknown E1 mismatch type(s) {unknown}; expected "
                         f"some of {list(E1_MISMATCH_TYPES)}")
    if repeats < 1:
        raise ValueError("E1 needs at least one repeat")
    ordered = [t for t in E1_MISMATCH_TYPES if t in types]
    runs = []
    for index in range(repeats):
        seed = None if repeats == 1 else index
        common = dict(experiment="E1", scheduler=scheduler,
                      execution_mode=LOCKSTEP, seed=seed)
        runs.append(RunSpec(notes="control: the validated pairing, no "
                                  "mismatch", **common))
        for kind in ordered:
            if kind == "frame_pairing":
                for guard in ("on", "ablated"):
                    runs.append(RunSpec(
                        attitude_provider=MISPAIRED_ATTITUDE_PROVIDER,
                        mismatch=kind, mismatch_guard=guard,
                        notes=f"frame_pairing, guard {guard}", **common))
            else:
                runs.append(RunSpec(mismatch=kind, mismatch_guard="on",
                                    notes=f"{kind}, guard on", **common))
    return Campaign(
        campaign_id=f"E1_mismatch_{len(ordered)}types_{repeats}x",
        experiment="E1",
        description=(f"Control against {len(ordered)} interface "
                     f"mismatch type(s): " + ", ".join(ordered) + ". "
                     + e1_formula_line(ordered, repeats) + "."),
        runs=tuple(runs), scenario_path=scenario_path, backend=backend)


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


#: E3's free-run real-time factors, in campaign order.
E3_FREE_RUN_RATES = (1.0, 0.5)
#: Lockstep repeats per E3 cell: two are enough to show bit-identical
#: signatures, and lockstep needs no more (it is deterministic).
E3_LOCKSTEP_REPEATS = 2


def _rate_repeats(n_repeats, lockstep_repeats, rtf05_repeats):
    """[(mode, rate, repeats)] in campaign order."""
    return [(LOCKSTEP, 1.0, lockstep_repeats)] + [
        (FREE_RUN, rate,
         rtf05_repeats if rate == 0.5 and rtf05_repeats is not None
         else n_repeats) for rate in E3_FREE_RUN_RATES]


def e3_run_count(n_repeats: int, n_fault_seeds: int,
                 lockstep_repeats: int = E3_LOCKSTEP_REPEATS,
                 rtf05_repeats=None) -> int:
    """(1 + K) x (lockstep + rtf 1.0 + rtf 0.5 repeats)."""
    return (1 + n_fault_seeds) * sum(
        r for _, _, r in _rate_repeats(n_repeats, lockstep_repeats,
                                       rtf05_repeats))


def e3_formula_line(n_repeats: int, n_fault_seeds: int,
                    lockstep_repeats: int = E3_LOCKSTEP_REPEATS,
                    rtf05_repeats=None) -> str:
    if lockstep_repeats != E3_LOCKSTEP_REPEATS or (
            rtf05_repeats is not None and rtf05_repeats != n_repeats):
        r05 = n_repeats if rtf05_repeats is None else rtf05_repeats
        per = lockstep_repeats + n_repeats + r05
        return (f"E3 runs = (1 + N_FAULT_SEEDS) x ({lockstep_repeats} "
                f"lockstep + {n_repeats} rtf 1.0 + {r05} rtf 0.5) = "
                f"(1 + {n_fault_seeds}) x {per} = "
                f"{(1 + n_fault_seeds) * per}")
    return (f"E3 runs = (1 + N_FAULT_SEEDS) x ({E3_LOCKSTEP_REPEATS} lockstep "
            f"+ {len(E3_FREE_RUN_RATES)} x N_REPEATS free-run) = "
            f"(1 + {n_fault_seeds}) x ({E3_LOCKSTEP_REPEATS} + "
            f"{len(E3_FREE_RUN_RATES)} x {n_repeats}) = "
            f"{e3_run_count(n_repeats, n_fault_seeds)}")


def fault_seed_list(fault_seeds) -> Tuple[int, ...]:
    """K (an int) -> seeds 0..K-1; a sequence -> those seeds, in order."""
    if isinstance(fault_seeds, int):
        if fault_seeds < 0:
            raise ValueError("the number of fault seeds cannot be negative")
        return tuple(range(fault_seeds))
    seeds = tuple(int(s) for s in fault_seeds)
    if len(set(seeds)) != len(seeds):
        raise ValueError(f"fault seeds repeat: {list(seeds)}")
    return seeds


def build_e3_matrix(n_repeats: int, n_fault_seeds,
                    window_duration_s: float, scenario_path: str = "",
                    backend: str = "com",
                    scheduler: str = "dynamics_aware",
                    lockstep_repeats: int = E3_LOCKSTEP_REPEATS,
                    rtf05_repeats=None) -> Campaign:
    """E3: execution mode x fault x repeats, the fault replay included.

    Fault cells: none, and one seeded wheel fault per seed 0..K-1
    (plan_campaign_faults, the same fault in every mode). Per cell:
    lockstep x E3_LOCKSTEP_REPEATS, then free-run at each rate in
    E3_FREE_RUN_RATES x n_repeats. Ordered lockstep, rtf 1.0, rtf 0.5,
    so one block runs after another without any change by the operator.
    The scenario is the campaign's (the reference unless --scenario),
    as for build_e3_determinism.
    """
    if n_repeats < 1:
        raise ValueError("E3 needs at least one free-run repeat per cell")
    seeds = fault_seed_list(n_fault_seeds)
    faults = plan_campaign_faults(list(seeds), window_duration_s)
    cells = [None] + [faults[seed] for seed in seeds]
    blocks = _rate_repeats(n_repeats, lockstep_repeats, rtf05_repeats)
    runs = []
    for mode, rate, repeats in blocks:
        for fault in cells:
            for index in range(repeats):
                runs.append(RunSpec(
                    experiment="E3", scheduler=scheduler,
                    execution_mode=mode, seed=index, fault=fault,
                    real_time_factor=rate,
                    notes=f"{mode} rtf {rate:g}, "
                          + ("no fault" if fault is None
                             else f"fault seed {fault.seed}")))
    ids = [r.run_id for r in runs]
    if len(set(ids)) != len(ids):
        raise ValueError("two E3 runs share a run id (two fault seeds "
                         "planned the same fault)")
    return Campaign(
        campaign_id=f"E3_matrix_{n_repeats}x_{len(seeds)}faults",
        experiment="E3",
        description=("Lockstep against free-run at rtf "
                     + " and ".join(f"{r:g}" for r in E3_FREE_RUN_RATES)
                     + ", without a fault and with each seeded wheel fault. "
                     + e3_formula_line(n_repeats, len(seeds),
                                       lockstep_repeats, rtf05_repeats)
                     + "."),
        runs=tuple(runs), scenario_path=scenario_path, backend=backend)


#: Measured wall time per automated run, minutes: summary.json
#: started_at to finished_at, MATLAB start and build included. Excluded:
#: E6_P9 seed1 (22.0 min, an 11.5 min console stall) and E6_P10 (14.8
#: min, an STK started through COM). None: not measured.
MEASURED_WALL_MIN = {
    (LOCKSTEP, 1.0): (9.5, 11.4,
                      "E5_P9d_lock2 9.5, E5_P9d_lock1 9.9, E6_P9 seed0 10.3, "
                      "E6_P9 seed2 10.4, E2_P10_seed1 11.4"),
    (FREE_RUN, 1.0): (15.3, 16.2,
                      "E3_P9_rtf1 15.3, E5_P9d_free1 15.3, E5_P10_free2 "
                      "16.2"),
    (FREE_RUN, 0.5): (27.3, 27.3,
                      "1 run: LIVE_E3_d0bf796 "
                      "E3-dynamics_aware-free_run-seed0-rtf0p5-faultx218p375"),
}
#: Said beside the missing rtf 0.5 time: measured, but with a manual twin.
UNMEASURED_NOTE = {(FREE_RUN, 0.5): "the one run at rtf 0.5, "
                                    "E3_P6_rtf05_8345551, had a manual "
                                    "twin (37.6 min)"}


def wall_time_lines() -> List[str]:
    lines = []
    for (mode, rate), measured in MEASURED_WALL_MIN.items():
        name = "lockstep" if mode == LOCKSTEP else f"free-run rtf {rate:.1f}"
        if measured is None:
            lines.append(f"  {name}: not measured with an automated twin "
                         f"({UNMEASURED_NOTE[(mode, rate)]})")
        else:
            low, high, source = measured
            span = (f"{low:.1f}" if low == high else f"{low:.1f}-{high:.1f}")
            lines.append(f"  {name}: {span} min ({source})")
    return lines


def estimate_lines(label: str, counts: Dict[Tuple[str, float], int]
                   ) -> Tuple[List[str], float, float, int]:
    """One line per (mode, rate) with runs and minutes; totals of the
    measured part and the number of runs not measured."""
    lines, low, high, unmeasured = [], 0.0, 0.0, 0
    for (mode, rate), runs in counts.items():
        if not runs:
            continue
        name = "lockstep" if mode == LOCKSTEP else f"free-run rtf {rate:.1f}"
        measured = MEASURED_WALL_MIN[(mode, rate)]
        if measured is None:
            unmeasured += runs
            lines.append(f"  {label}: {runs} {name} runs: not measured")
        else:
            lines.append(f"  {label}: {runs} {name} runs: "
                         f"{runs * measured[0]:.1f}-{runs * measured[1]:.1f}"
                         f" min")
            low += runs * measured[0]
            high += runs * measured[1]
    return lines, low, high, unmeasured


def e3_counts(n_repeats: int, n_fault_seeds: int,
              lockstep_repeats: int = E3_LOCKSTEP_REPEATS,
              rtf05_repeats=None) -> Dict[Tuple[str, float], int]:
    cells = 1 + n_fault_seeds
    return {(mode, rate): cells * repeats for mode, rate, repeats in
            _rate_repeats(n_repeats, lockstep_repeats, rtf05_repeats)}


def combined_plan_lines(n_repeats: int, n_fault_seeds: int,
                        e2_seeds: int = 10,
                        lockstep_repeats: int = E3_LOCKSTEP_REPEATS,
                        rtf05_repeats=None) -> List[str]:
    """E1 (all six types) + E2 (seeds x 2 arms) + E3, in hours."""
    parts = [
        (f"E1 ({len(E1_MISMATCH_TYPES)} types, N_REPEATS={n_repeats})",
         {(LOCKSTEP, 1.0): e1_run_count(E1_MISMATCH_TYPES, n_repeats)}),
        (f"E2 ({e2_seeds} seeds x 2 arms)", {(LOCKSTEP, 1.0): 2 * e2_seeds}),
        (f"E3 (N_REPEATS={n_repeats}, N_FAULT_SEEDS={n_fault_seeds})",
         e3_counts(n_repeats, n_fault_seeds, lockstep_repeats,
                   rtf05_repeats)),
    ]
    lines = [f"combined campaign (N_REPEATS={n_repeats}, "
             f"N_FAULT_SEEDS={n_fault_seeds}, E2 seeds={e2_seeds}); every "
             f"lockstep run counted at full length, so fail-stop E1 arms "
             f"make this an upper bound:"]
    low = high = 0.0
    unmeasured = 0
    for label, counts in parts:
        more, a, b, c = estimate_lines(label, counts)
        lines += more
        low, high, unmeasured = low + a, high + b, unmeasured + c
    total = (f"  total measured part: {low / 60:.2f}-{high / 60:.2f} h")
    if unmeasured:
        total += f", plus {unmeasured} runs not measured"
    lines.append(total)
    return lines


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
