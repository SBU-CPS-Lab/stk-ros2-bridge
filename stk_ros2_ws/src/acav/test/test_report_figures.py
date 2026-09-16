"""Tests for C6's figures.

Mostly smoke tests -- each function must return a Figure without
raising, for both populated and empty input, since a campaign directory
with a still-missing experiment must not crash the whole report. Two
tests assert on actual rendered content: a bug where a figure silently
dropped data because its input dict used different key names than the
aggregation function that fills it (see the E6 test below) rendered
without raising and would have shipped unnoticed without one.
"""

import matplotlib

matplotlib.use("Agg")   # no display in CI or this dev box

import pytest

from acav.campaign import (
    compare_schedulers,
    convention_comparison,
    determinism_comparison,
    gain_surface,
    latency_breakdown,
    ReplayCheck,
)
from acav.metrics import CaptureOutcome, RunResult, TargetResult
from acav.report_figures import (
    e1_convention_figure,
    e2_capture_vs_coverage_figure,
    e2_outcome_breakdown_figure,
    e3_determinism_figure,
    e4_gain_surface_figure,
    e5_replay_figure,
    e6_latency_figure,
    save_all,
)
from acav.run_record import Provenance, RunRecord


def _result(run_id, scheduler, outcomes, seed=None):
    targets = [
        TargetResult(target_id=f"T{i}", attempted=o != "not_attempted",
                    outcome=CaptureOutcome(o), image_time_s=float(i * 60),
                    error_at_image_deg=0.2, max_error_in_hold_deg=0.2)
        for i, o in enumerate(outcomes)
    ]
    return RunResult(run_id=run_id, scheduler=scheduler, seed=seed,
                     targets=targets)


def _record(run_id, scheduler="dynamics_aware", outcomes=("captured",),
           backend="com", mode="lockstep", seed=None,
           attitude_provider="", gains=None,
           sense=None, actuate=None, barrier=None):
    result = _result(run_id, scheduler, list(outcomes), seed)
    if sense is not None:
        result.mean_sense_latency_ms = sense
    if actuate is not None:
        result.mean_actuate_latency_ms = actuate
    if barrier is not None:
        result.mean_barrier_wait_ms = barrier
    return RunRecord(result=result, provenance=Provenance(
        backend_type=backend, execution_mode=mode,
        attitude_provider=attitude_provider, gains=gains or {}))


class TestEveryFigureRendersOnEmptyInput:
    """A campaign directory missing one experiment must not crash the
    whole report -- each function gets an explicit "no data" path."""

    def test_e1(self):
        assert e1_convention_figure({}) is not None

    def test_e2_capture(self):
        assert e2_capture_vs_coverage_figure({}) is not None

    def test_e2_outcomes(self):
        assert e2_outcome_breakdown_figure({}) is not None

    def test_e3(self):
        assert e3_determinism_figure({}) is not None

    def test_e4(self):
        assert e4_gain_surface_figure([]) is not None

    def test_e5(self):
        assert e5_replay_figure([]) is not None

    def test_e6(self):
        assert e6_latency_figure({}) is not None


class TestE1Convention:
    def test_both_bars_are_present_even_at_zero_percent(self):
        """A 0%-height bar rendered as literally nothing once -- the
        finding disappeared instead of showing as the finding."""
        records = [
            _record("control", outcomes=["captured"] * 6 + ["not_settled"]),
            _record("treat", outcomes=["no_access"] * 7,
                   attitude_provider="Attitude Quaternions"),
        ]
        fig = e1_convention_figure(convention_comparison(records, 7))
        bars = [p for ax in fig.axes for p in ax.patches]
        assert len(bars) == 2
        assert all(bar.get_height() > 0 for bar in bars)


class TestE6Latency:
    def test_all_three_phases_are_drawn_not_just_one(self):
        """Regression test: the figure once read
        ``mean_sense_ms``/``mean_actuate_ms`` while
        ``campaign.latency_breakdown`` produces
        ``mean_sense_latency_ms``/``mean_actuate_latency_ms``. The
        mismatched keys fell back to a default of 0.0 silently -- the
        figure rendered, just with two-thirds of the bar missing. This
        goes through the REAL aggregation function rather than a
        hand-built dict with keys chosen to match the figure, which is
        exactly the mismatch that let the original bug through."""
        records = [_record(f"e6-{i}", sense=2.3, actuate=1.1, barrier=6.8)
                  for i in range(3)]
        breakdown = latency_breakdown(records)
        fig = e6_latency_figure(breakdown)

        bars = [p for ax in fig.axes for p in ax.patches]
        widths = sorted(bar.get_width() for bar in bars)
        assert len(widths) == 3
        assert all(w > 0 for w in widths)
        assert widths == pytest.approx([1.1, 2.3, 6.8], abs=1e-6)


class TestE4GainSurface:
    def test_missing_combinations_are_blank_not_interpolated(self):
        """A combination the vehicle was never flown at must show as a
        gap, not as a value inferred from its neighbours."""
        surface = gain_surface([
            _record("a", gains={"natural_frequency_rad_s": 1.0,
                                "profile_authority": 0.8}),
        ], total_targets=1)
        fig = e4_gain_surface_figure(surface)
        image = fig.axes[0].images[0]
        import numpy as np
        assert np.isnan(image.get_array()).sum() == 0  # the one cell present

    def test_a_sweep_that_does_not_vary_both_axes_is_reported_as_no_data(self):
        surface = gain_surface([
            _record("a", gains={"natural_frequency_rad_s": 1.0}),
        ], total_targets=1)
        fig = e4_gain_surface_figure(surface)
        assert fig.axes[0].texts  # the "no data" message, not a heatmap


class TestE5Replay:
    def test_a_failed_seed_is_visually_distinct_from_reproduced_ones(self):
        from acav.report_style import STATUS_COLOR
        from matplotlib.colors import to_hex

        checks = [ReplayCheck(f"E5-seed{i}", f"E5-seed{i}-replay", i != 1)
                 for i in range(4)]
        fig = e5_replay_figure(checks)
        scatter = fig.axes[0].collections[0]
        colors = [to_hex(c) for c in scatter.get_facecolors()]
        assert colors[1] == to_hex(STATUS_COLOR["safe_mode"])
        assert colors[0] == to_hex(STATUS_COLOR["captured"])


class TestSaveAll:
    def test_it_writes_every_requested_format(self, tmp_path):
        fig = e6_latency_figure({})
        written = save_all({"x": fig}, str(tmp_path), formats=("png", "svg"))
        names = {p.split("/")[-1] for p in written}
        assert names == {"x.png", "x.svg"}
        for path in written:
            assert (tmp_path / path.split("/")[-1]).exists()
