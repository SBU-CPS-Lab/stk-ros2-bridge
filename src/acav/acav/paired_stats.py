"""Exact paired statistics for small samples (E2 across seeds).

Pure Python, no scipy (not a dependency of this package). The test
(test_paired_stats.py) checks every p-value against an independent
enumeration of all sign assignments.

Wilcoxon signed-rank, exact, two-sided
--------------------------------------
* Zero differences are dropped before ranking (Wilcoxon's original
  method); the test's n is the number of non-zero differences.
* Tied |differences| get average ranks. The null distribution is the
  exact distribution of W+ over all 2^n sign assignments of those
  average ranks (the exact conditional distribution), computed by
  counting subset sums of the doubled ranks, which are integers.
* p = min(1, 2 * min(P(W+ <= w), P(W+ >= w))).

Without ties the smallest attainable p is 2^(1 - n) (all differences of
one sign): 0.0625 for n = 5, so with 5 or fewer pairs no result can
reach p < 0.05 (min_p_line flags it).

Effect size and location
------------------------
* Matched-pairs rank-biserial r = (W+ - W-) / (W+ + W-), on the
  non-zero differences; None when every difference is zero.
* Hodges-Lehmann estimate: the median of the Walsh averages
  (d_i + d_j) / 2 for i <= j, over all differences, zeros included.
* Percentile bootstrap CI of the median paired difference: pairs are
  resampled with replacement with random.Random(seed); the q-quantile
  of the B sorted replicate medians is element ceil(q * B) - 1.
* Holm's step-down adjustment for several metrics.
"""

from __future__ import annotations

import math
import random
import statistics
from typing import Dict, List, Optional, Sequence

#: Defaults the E2 paired report uses; both are written into its output.
BOOTSTRAP_SEED = 20261017
BOOTSTRAP_RESAMPLES = 10000


def _average_ranks(magnitudes: Sequence[float]) -> List[float]:
    order = sorted(range(len(magnitudes)), key=lambda i: magnitudes[i])
    ranks = [0.0] * len(magnitudes)
    i = 0
    while i < len(order):
        j = i
        while (j + 1 < len(order)
               and magnitudes[order[j + 1]] == magnitudes[order[i]]):
            j += 1
        average = (i + 1 + j + 1) / 2.0
        for k in range(i, j + 1):
            ranks[order[k]] = average
        i = j + 1
    return ranks


def wilcoxon_signed_rank(diffs: Sequence[float]) -> Dict:
    nonzero = [float(d) for d in diffs if d != 0]
    result = {"n": len(diffs), "n_nonzero": len(nonzero),
              "zeros_dropped": len(diffs) - len(nonzero),
              "w_plus": 0.0, "w_minus": 0.0, "ties": False, "p_exact": 1.0}
    if not nonzero:
        return result
    magnitudes = [abs(d) for d in nonzero]
    ranks = _average_ranks(magnitudes)
    result["ties"] = len(set(magnitudes)) < len(magnitudes)
    w_plus = sum(r for r, d in zip(ranks, nonzero) if d > 0)
    result["w_plus"] = w_plus
    result["w_minus"] = sum(ranks) - w_plus
    # Exact null distribution of W+: count subset sums of doubled ranks.
    doubled = [int(round(2 * r)) for r in ranks]
    counts = [0] * (sum(doubled) + 1)
    counts[0] = 1
    for value in doubled:
        for total in range(len(counts) - 1, value - 1, -1):
            counts[total] += counts[total - value]
    observed = int(round(2 * w_plus))
    assignments = 2 ** len(doubled)
    low = sum(counts[:observed + 1]) / assignments
    high = sum(counts[observed:]) / assignments
    result["p_exact"] = min(1.0, 2.0 * min(low, high))
    return result


def rank_biserial(diffs: Sequence[float]) -> Optional[float]:
    test = wilcoxon_signed_rank(diffs)
    total = test["w_plus"] + test["w_minus"]
    if total == 0:
        return None
    return (test["w_plus"] - test["w_minus"]) / total


def hodges_lehmann(diffs: Sequence[float]) -> float:
    values = [float(d) for d in diffs]
    if not values:
        return float("nan")
    walsh = [(values[i] + values[j]) / 2.0
             for i in range(len(values)) for j in range(i, len(values))]
    return statistics.median(walsh)


def bootstrap_median_ci(diffs: Sequence[float],
                        seed: int = BOOTSTRAP_SEED,
                        resamples: int = BOOTSTRAP_RESAMPLES,
                        level: float = 0.95) -> Dict:
    out = {"seed": seed, "resamples": resamples, "level": level,
           "method": "percentile, pairs resampled with replacement",
           "lower": float("nan"), "upper": float("nan")}
    values = [float(d) for d in diffs]
    if not values:
        return out
    rng = random.Random(seed)
    n = len(values)
    medians = sorted(
        statistics.median(values[rng.randrange(n)] for _ in range(n))
        for _ in range(resamples))
    alpha = (1.0 - level) / 2.0
    out["lower"] = medians[math.ceil(alpha * resamples) - 1]
    out["upper"] = medians[math.ceil((1.0 - alpha) * resamples) - 1]
    return out


def holm(pvalues: Dict[str, float]) -> Dict[str, float]:
    """Holm step-down adjusted p-values, keyed like the input."""
    ordered = sorted(pvalues.items(), key=lambda item: item[1])
    m = len(ordered)
    adjusted: Dict[str, float] = {}
    running = 0.0
    for index, (name, p) in enumerate(ordered):
        running = max(running, min(1.0, (m - index) * p))
        adjusted[name] = running
    return adjusted


def min_attainable_p(n: int) -> float:
    """The smallest two-sided exact p with n non-zero untied differences."""
    return 1.0 if n <= 0 else min(1.0, 2.0 ** (1 - n))


def min_p_line(n: int) -> str:
    p = min_attainable_p(n)
    line = (f"minimum attainable two-sided exact p for n = {n} pairs: "
            f"{p:.6g}")
    if p > 0.05:
        line += (" -- ABOVE 0.05: underpowered, no result can reach "
                 "p < 0.05")
    return line


def describe(diffs: Sequence[float], seed: int = BOOTSTRAP_SEED,
             resamples: int = BOOTSTRAP_RESAMPLES) -> Dict:
    """Everything the paired report gives for one metric."""
    values = list(diffs)
    test = wilcoxon_signed_rank(values)
    return {
        "n_pairs": len(values),
        "differences": list(values),
        "median_difference": (statistics.median(values) if values
                              else float("nan")),
        "hodges_lehmann": hodges_lehmann(values),
        "w_plus": test["w_plus"], "w_minus": test["w_minus"],
        "n_nonzero": test["n_nonzero"],
        "zeros_dropped": test["zeros_dropped"], "ties": test["ties"],
        "p_exact": test["p_exact"],
        "rank_biserial": rank_biserial(values),
        "bootstrap_median_ci": bootstrap_median_ci(values, seed, resamples),
        "min_attainable_p": min_attainable_p(test["n_nonzero"]),
    }
