"""E2 across seeded scenarios, analysed as pairs.

Each seeded scenario (acav/scenario_generator.py) is its own E2 campaign:
one kinematic and one dynamics-aware run. The pairing unit is the
scenario seed (``provenance.scenario_seed``); the paired difference is
dynamics-aware minus kinematic. A seed with only one of the two runs is
dropped and listed by name, never imputed.

The metrics are ones every record already carries (acav/metrics.py);
nothing new is computed per run:

    coverage_rate            captured / targets in the scenario
    capture_rate             captured / attempted (NaN if none attempted)
    wheel_energy_J           RunResult.wheel_energy_J
    wheel_saturation_events  RunResult.wheel_saturation_events

over_promise_gap is 1 - capture_rate, so its test would repeat
capture_rate's; it is not tested separately. A pair whose difference is
NaN for one metric is left out of that metric only, and listed.

Per metric (acav/paired_stats.py): the differences, their median, the
Hodges-Lehmann estimate, the exact two-sided Wilcoxon signed-rank p
(zeros dropped, average ranks for ties, exact conditional
distribution), the matched-pairs rank-biserial, a seeded percentile
bootstrap CI of the median difference, and Holm-adjusted p over the
metrics tested. The minimum attainable p for the number of pairs is
printed, flagged above 0.05.

Records are pooled only if they agree on: code version, twin identity
(build and runtime fingerprints and polling rate; the run-config digest
differs per run by design), how STK was started, execution mode and
generator version, and only if every record is reportable, from a
seeded scenario, and one per (seed, scheduler). paired_refusals names
the records behind each disagreement.
"""

from __future__ import annotations

import math
from typing import Callable, Dict, List, Sequence, Tuple

from acav.metrics import capture_rate, coverage_rate
from acav.paired_stats import (BOOTSTRAP_RESAMPLES, BOOTSTRAP_SEED, describe,
                               holm, min_p_line)
from acav.run_record import RunRecord

KINEMATIC, DYNAMICS_AWARE = "kinematic", "dynamics_aware"
DIRECTION = f"{DYNAMICS_AWARE} - {KINEMATIC}"

METRICS: Dict[str, Callable[[RunRecord], float]] = {
    "coverage_rate": lambda r: coverage_rate(r.result, len(r.result.targets)),
    "capture_rate": lambda r: capture_rate(r.result),
    "wheel_energy_J": lambda r: float(r.result.wheel_energy_J),
    "wheel_saturation_events": lambda r: r.result.wheel_saturation_events,
}

ZERO_AND_TIE_HANDLING = (
    "zero differences dropped before ranking (Wilcoxon's method); tied "
    "|differences| get average ranks and the exact conditional null "
    "distribution over those ranks is used")


def _name(record: RunRecord) -> str:
    return record.result.run_id


def _identity_key(text: str) -> str:
    from acav.twin_identity import parse_identity
    try:
        identity = parse_identity(text, versions=("v2", "v3"))
    except ValueError:
        return f"unreadable ({text!r})"
    return (f"build {identity.build_fingerprint}, runtime "
            f"{identity.runtime_fingerprint}, polling {identity.polling_hz}")


def _start_mode(record: RunRecord) -> str:
    from acav.stk_process import start_mode
    return start_mode(record.provenance.stk_process) or "not recorded"


#: (what, value of a record) -- every record must share one value.
AGREEMENT: Tuple[Tuple[str, Callable[[RunRecord], str]], ...] = (
    ("code version", lambda r: r.provenance.code_version or "not recorded"),
    ("twin identity", lambda r: _identity_key(r.provenance.twin_identity)),
    ("STK start mode (stk_process)", _start_mode),
    ("execution mode", lambda r: r.provenance.execution_mode),
    ("generator version",
     lambda r: r.provenance.scenario_generator_version or "not recorded"),
)


def paired_refusals(records: Sequence[RunRecord]) -> List[str]:
    """Why these records must not be analysed together; empty if none."""
    problems: List[str] = []
    not_seeded = sorted(_name(r) for r in records
                        if r.provenance.scenario_seed is None)
    if not_seeded:
        problems.append("not from a seeded scenario (no scenario_seed): "
                        + ", ".join(not_seeded))
    unreportable = sorted(f"{_name(r)} ({'; '.join(r.unreportable_reasons)})"
                          for r in records if not r.reportable)
    if unreportable:
        problems.append("not reportable: " + ", ".join(unreportable))
    other = sorted(f"{_name(r)} ({r.result.scheduler})" for r in records
                   if r.result.scheduler not in (KINEMATIC, DYNAMICS_AWARE))
    if other:
        problems.append("scheduler is neither kinematic nor dynamics_aware: "
                        + ", ".join(other))
    for what, value_of in AGREEMENT:
        groups: Dict[str, List[str]] = {}
        for record in records:
            groups.setdefault(value_of(record), []).append(_name(record))
        if len(groups) > 1:
            problems.append(f"different {what}: " + "; ".join(
                f"{value!r}: {', '.join(sorted(names))}"
                for value, names in sorted(groups.items())))
    slots: Dict[Tuple, List[str]] = {}
    for record in records:
        if record.provenance.scenario_seed is not None:
            slots.setdefault((record.provenance.scenario_seed,
                              record.result.scheduler), []).append(
                _name(record))
    duplicates = [f"seed {seed} {scheduler}: {', '.join(sorted(names))}"
                  for (seed, scheduler), names in sorted(slots.items())
                  if len(names) > 1]
    if duplicates:
        problems.append("more than one record for one seed and scheduler: "
                        + "; ".join(duplicates))
    return problems


def pair_records(records: Sequence[RunRecord]
                 ) -> Tuple[Dict[int, Tuple[RunRecord, RunRecord]],
                            List[str]]:
    """{seed: (kinematic, dynamics_aware)} and the incomplete seeds."""
    by_seed: Dict[int, Dict[str, RunRecord]] = {}
    for record in records:
        by_seed.setdefault(record.provenance.scenario_seed, {})[
            record.result.scheduler] = record
    pairs, dropped = {}, []
    for seed in sorted(by_seed):
        found = by_seed[seed]
        if KINEMATIC in found and DYNAMICS_AWARE in found:
            pairs[seed] = (found[KINEMATIC], found[DYNAMICS_AWARE])
        else:
            (scheduler, record), = found.items()
            dropped.append(f"seed {seed}: {scheduler} only ({_name(record)})")
    return pairs, dropped


def analyse_pairs(records: Sequence[RunRecord], seed: int = BOOTSTRAP_SEED,
                  resamples: int = BOOTSTRAP_RESAMPLES) -> Dict:
    pairs, dropped = pair_records(records)
    metrics: Dict[str, Dict] = {}
    for name, value_of in METRICS.items():
        diffs, nan_seeds = [], []
        for pair_seed, (kinematic, dynamics_aware) in pairs.items():
            difference = value_of(dynamics_aware) - value_of(kinematic)
            if isinstance(difference, float) and math.isnan(difference):
                nan_seeds.append(pair_seed)
            else:
                diffs.append(difference)
        metrics[name] = describe(diffs, seed=seed, resamples=resamples)
        metrics[name]["seeds_without_a_value"] = nan_seeds
    adjusted = holm({name: m["p_exact"] for name, m in metrics.items()})
    for name, value in adjusted.items():
        metrics[name]["p_holm"] = value
    return {
        "unit": "scenario seed",
        "direction": DIRECTION,
        "n_pairs": len(pairs),
        "seeds": sorted(pairs),
        "dropped_incomplete": dropped,
        "metrics": metrics,
        "multiple_testing": (f"Holm step-down over the {len(metrics)} "
                             f"metrics tested (p_holm)"),
        "zero_and_tie_handling": ZERO_AND_TIE_HANDLING,
        "not_tested": {"over_promise_gap": "1 - capture_rate; its test is "
                                           "capture_rate's"},
        "bootstrap": {"seed": seed, "resamples": resamples,
                      "method": "percentile, pairs resampled with "
                                "replacement, median difference"},
        "min_p_line": min_p_line(len(pairs)),
    }


def report_lines(paired: Dict) -> List[str]:
    """The plain-text lines printed after the JSON."""
    lines = [paired["min_p_line"]]
    for name, m in paired["metrics"].items():
        ci = m["bootstrap_median_ci"]
        line = (f"{name}: n = {m['n_pairs']}, median difference "
                f"{m['median_difference']:.6g}, Hodges-Lehmann "
                f"{m['hodges_lehmann']:.6g}, exact p {m['p_exact']:.6g}, "
                f"Holm-adjusted p {m['p_holm']:.6g}, rank-biserial "
                f"{m['rank_biserial']}, {int(ci['level'] * 100)}% bootstrap "
                f"CI [{ci['lower']:.6g}, {ci['upper']:.6g}]")
        if m["n_nonzero"] < m["n_pairs"]:
            line += (f"; {m['zeros_dropped']} zero difference(s) dropped: "
                     + min_p_line(m["n_nonzero"]))
        lines.append(line)
    if paired["dropped_incomplete"]:
        lines.append("incomplete pairs dropped: "
                     + "; ".join(paired["dropped_incomplete"]))
    return lines
