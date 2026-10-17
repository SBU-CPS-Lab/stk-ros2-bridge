"""Exact paired statistics for E2 across seeds (acav/paired_stats.py).

All fixtures here are SYNTHETIC TEST FIXTURES with hand-computed
answers; none is a measurement. scipy is installed on the development
machine but is not a dependency of acav (setup.py install_requires), so
the exact Wilcoxon p-value is checked against a brute-force enumeration
of all 2^n sign assignments written independently below, and against
hand computation.

Conventions under test:
  * zero differences are dropped before ranking (Wilcoxon's original
    method); n_nonzero is the n of the test;
  * tied |differences| get average ranks, and the exact null
    distribution is computed over those average ranks (exact
    conditional distribution), not the untied table;
  * two-sided p = min(1, 2 * min(P(W+ <= w), P(W+ >= w)));
  * matched-pairs rank-biserial r = (W+ - W-) / (W+ + W-), None when
    every difference is zero;
  * Hodges-Lehmann = median of the Walsh averages (d_i + d_j) / 2, i <= j,
    over all differences, zeros included;
  * bootstrap: percentile CI of the median paired difference, pairs
    resampled with replacement, random.Random(seed), the q-quantile of
    the sorted replicates taken as element ceil(q * B) - 1.
"""

import itertools
import math

import pytest

PENDING = pytest.mark.xfail(strict=True,
                            reason="acav.paired_stats not written yet")


def _brute_p(diffs):
    """Independent check: enumerate every sign assignment."""
    nonzero = [d for d in diffs if d != 0]
    if not nonzero:
        return 1.0
    magnitudes = sorted(abs(d) for d in nonzero)
    ranks = {}
    i = 0
    while i < len(magnitudes):
        j = i
        while j + 1 < len(magnitudes) and magnitudes[j + 1] == magnitudes[i]:
            j += 1
        for k in range(i, j + 1):
            ranks.setdefault(magnitudes[k], (i + 1 + j + 1) / 2)
        i = j + 1
    r = [ranks[abs(d)] for d in nonzero]
    observed = sum(rank for rank, d in zip(r, nonzero) if d > 0)
    sums = [sum(rank for rank, s in zip(r, signs) if s)
            for signs in itertools.product((0, 1), repeat=len(r))]
    total = len(sums)
    low = sum(1 for s in sums if s <= observed + 1e-9) / total
    high = sum(1 for s in sums if s >= observed - 1e-9) / total
    return min(1.0, 2 * min(low, high))


@PENDING
def test_six_positive_differences_give_p_0_03125():
    from acav.paired_stats import wilcoxon_signed_rank
    result = wilcoxon_signed_rank([1, 2, 3, 4, 5, 6])
    assert result["n_nonzero"] == 6
    assert result["w_plus"] == 21 and result["w_minus"] == 0
    assert result["p_exact"] == pytest.approx(0.03125, abs=1e-12)


@PENDING
def test_all_zero_differences():
    from acav.paired_stats import (hodges_lehmann, rank_biserial,
                                   wilcoxon_signed_rank)
    result = wilcoxon_signed_rank([0, 0, 0, 0])
    assert result["n_nonzero"] == 0
    assert result["zeros_dropped"] == 4
    assert result["p_exact"] == 1.0
    assert rank_biserial([0, 0, 0, 0]) is None
    assert hodges_lehmann([0, 0, 0, 0]) == 0.0


@PENDING
def test_a_tie_case_computed_by_hand():
    """|d| = 1, 1, 2, 3 -> ranks 1.5, 1.5, 3, 4; W+ = 6, W- = 4.
    Of the 16 sign assignments, 6 have W+ >= 6 and 11 have W+ <= 6,
    so p = 2 * 6/16 = 0.75."""
    from acav.paired_stats import (hodges_lehmann, rank_biserial,
                                   wilcoxon_signed_rank)
    diffs = [1, 1, 2, -3]
    result = wilcoxon_signed_rank(diffs)
    assert (result["w_plus"], result["w_minus"]) == (6.0, 4.0)
    assert result["ties"] is True
    assert result["p_exact"] == pytest.approx(0.75, abs=1e-12)
    assert rank_biserial(diffs) == pytest.approx(0.2)
    # Walsh averages: -3, -1, -1, -0.5, 1, 1, 1, 1.5, 1.5, 2 -> median 1.
    assert hodges_lehmann(diffs) == pytest.approx(1.0)


@PENDING
def test_a_zero_is_dropped_before_ranking():
    from acav.paired_stats import wilcoxon_signed_rank
    with_zero = wilcoxon_signed_rank([0, 1, 2, 3, 4, 5, 6])
    assert with_zero["zeros_dropped"] == 1
    assert with_zero["p_exact"] == pytest.approx(0.03125, abs=1e-12)


@PENDING
@pytest.mark.parametrize("diffs", [
    [1, -2, 3, 4, -5, 6, 7],
    [0.5, 0.5, -0.5, 1.0, 2.0, -2.0, 3.0, 0.0],
    [-1, -2, -3],
    [2, 2, 2, 2, -2],
    [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7, 0.8, -0.9, 1.0],
])
def test_the_exact_p_matches_enumeration(diffs):
    from acav.paired_stats import wilcoxon_signed_rank
    assert wilcoxon_signed_rank(diffs)["p_exact"] == pytest.approx(
        _brute_p(diffs), abs=1e-12)


@PENDING
def test_six_positive_differences_effect_and_location():
    from acav.paired_stats import hodges_lehmann, rank_biserial
    diffs = [1, 2, 3, 4, 5, 6]
    assert rank_biserial(diffs) == 1.0
    assert hodges_lehmann(diffs) == pytest.approx(3.5)


@PENDING
@pytest.mark.parametrize("n, expected", [(3, 0.25), (5, 0.0625),
                                         (6, 0.03125), (10, 2 ** -9)])
def test_minimum_attainable_p(n, expected):
    from acav.paired_stats import min_attainable_p
    assert min_attainable_p(n) == pytest.approx(expected)


@PENDING
def test_minimum_p_lines_flag_underpowered_designs():
    from acav.paired_stats import min_p_line
    assert min_p_line(3) == (
        "minimum attainable two-sided exact p for n = 3 pairs: 0.25 "
        "-- ABOVE 0.05: underpowered, no result can reach p < 0.05")
    assert min_p_line(5) == (
        "minimum attainable two-sided exact p for n = 5 pairs: 0.0625 "
        "-- ABOVE 0.05: underpowered, no result can reach p < 0.05")
    assert min_p_line(6) == (
        "minimum attainable two-sided exact p for n = 6 pairs: 0.03125")
    assert min_p_line(10) == (
        "minimum attainable two-sided exact p for n = 10 pairs: 0.00195312")
    assert min_p_line(0).startswith(
        "minimum attainable two-sided exact p for n = 0 pairs: 1 -- ABOVE")


@PENDING
def test_holm_adjustment():
    from acav.paired_stats import holm
    adjusted = holm({"a": 0.01, "b": 0.04, "c": 0.03, "d": 0.2})
    assert adjusted == pytest.approx({"a": 0.04, "b": 0.09, "c": 0.09,
                                      "d": 0.2})


@PENDING
def test_holm_never_exceeds_one():
    from acav.paired_stats import holm
    # 0.6 * 2 = 1.2 -> capped at 1; 0.7 * 1, raised to the running max.
    assert holm({"a": 0.6, "b": 0.7}) == {"a": 1.0, "b": 1.0}


@PENDING
def test_bootstrap_is_seeded_and_recorded():
    from acav.paired_stats import bootstrap_median_ci
    first = bootstrap_median_ci([1, 2, 3, 4, 5, 6], seed=7, resamples=10000)
    again = bootstrap_median_ci([1, 2, 3, 4, 5, 6], seed=7, resamples=10000)
    assert first == again
    assert first["seed"] == 7 and first["resamples"] == 10000
    assert first["level"] == 0.95
    assert first["lower"] <= 3.5 <= first["upper"]
    constant = bootstrap_median_ci([2, 2, 2], seed=1, resamples=10000)
    assert (constant["lower"], constant["upper"]) == (2, 2)


@PENDING
def test_describe_collects_everything_for_one_metric():
    from acav.paired_stats import describe
    out = describe([1, 2, 3, 4, 5, 6], seed=3, resamples=1000)
    assert out["n_pairs"] == 6
    assert out["differences"] == [1, 2, 3, 4, 5, 6]
    assert out["median_difference"] == 3.5
    assert out["hodges_lehmann"] == pytest.approx(3.5)
    assert out["p_exact"] == pytest.approx(0.03125)
    assert out["rank_biserial"] == 1.0
    assert out["bootstrap_median_ci"]["seed"] == 3
    assert out["min_attainable_p"] == pytest.approx(0.03125)
    assert not math.isnan(out["bootstrap_median_ci"]["lower"])
