"""C6's figures -- one function per experiment's question.

Each function takes the aggregation :mod:`acav.campaign` already
computes (``compare_schedulers``, ``gain_surface``,
``determinism_comparison``, ``latency_breakdown``) and returns a
``matplotlib.figure.Figure``. Nothing here re-derives a metric -- that
would risk the exact failure C1 warns about for the slew model, two
independently-written computations of one quantity silently
disagreeing. These functions only lay the numbers out.

Why a Figure, not a saved file
-------------------------------
Returning the object rather than writing to disk keeps this module
testable without touching the filesystem, and lets a caller compose
several figures into one document however it likes. :func:`save_all`
does the saving, once, for the CLI.

Chart-type rationale, briefly (see ``docs/C6_analysis.md`` for the rest)
-------------------------------------------------------------------------
* **E2 -- capture vs coverage**: a *grouped* bar, two measures, not a
  dual-axis chart. Coverage rate has a different denominator to capture
  rate (all targets vs attempted only, see ``acav/metrics.py``), so
  encoding them on two y-scales would let two unrelated ranges appear
  to correlate. Same scale (0-1), two bars, in the same figure.
* **E1 -- convention**: a slope/paired-bar figure across exactly two
  points (control, treatment). The whole finding is the magnitude of
  the drop, so the two bars sit directly adjacent with the gap
  annotated -- not spread across a category axis that invites reading
  it as a trend.
* **E4 -- gain surface**: a heatmap, sequential blue, one hue. This is
  the textbook "magnitude over two continuous axes" case a heatmap is
  for; a 3-D surface plot would look impressive and read worse -- axis
  occlusion on a print page, no way to eyeball the actual chosen point.
* **E3 -- determinism spread**: distinct signature counts are a status
  fact (deterministic or not), so status color; the pointing-error
  spread that is the *magnitude* of any non-determinism is a
  categorical dot compared side by side, not folded into one number.
* **E6 -- latency breakdown**: a single stacked horizontal bar. Three
  parts of one whole (the step), so a stack is the correct form -- a
  grouped bar would misleadingly suggest the three phases are
  comparable alternatives rather than components of the same total.
"""

from __future__ import annotations

import os
from typing import Any, Dict, List, Optional, Sequence

from acav.report_style import (
    CATEGORICAL,
    GRIDLINE,
    INK_MUTED,
    INK_PRIMARY,
    INK_SECONDARY,
    MODE_COLOR,
    SCHEDULER_COLOR,
    SEQUENTIAL_BLUE,
    STATUS_COLOR,
    apply_style,
    scheduler_label,
    status_label,
)


def _figure(width: float = 6.4, height: float = 4.0):
    # apply_style() forces the Agg backend and MUST run before pyplot
    # is imported for the first time in this process, or the backend
    # choice has no effect.
    apply_style()
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(width, height))
    return fig, ax


def _no_data_figure(title: str, message: str):
    fig, ax = _figure(width=6.4, height=2.4)
    ax.set_title(title)
    ax.axis("off")
    ax.text(0.5, 0.5, message, ha="center", va="center",
            color=INK_MUTED, fontsize=10, transform=ax.transAxes)
    return fig


# ---------------------------------------------------------------------------
# E1 -- does convention validation matter?
# ---------------------------------------------------------------------------

def e1_convention_figure(comparison: Dict[str, Any]):
    """Control vs treatment capture rate. The whole finding is the gap.

    ``comparison`` is ``acav.campaign.convention_comparison``'s output:
    keyed by "control" (the validated pairing) and "treatment" (the
    forced, mis-paired one), NOT by scheduler -- E1 holds the scheduler
    fixed and varies the attitude read-back provider instead.
    """
    schedulers = comparison
    keys = [k for k in ("control", "treatment") if k in schedulers]
    if len(keys) < 2:
        return _no_data_figure(
            "E1 -- Convention validation",
            "Need both a control and a treatment run.")

    fig, ax = _figure(width=4.8, height=4.2)
    values = [schedulers[k]["capture_rate"] for k in keys]
    colors = [CATEGORICAL[0], CATEGORICAL[7]]
    # A floor height so a 0% (or near-zero) result still draws as a
    # visible sliver -- a bar chart with an invisible bar reads as a
    # missing data point, not as "this is exactly the finding".
    floor = 0.012
    heights = [max(v, floor) for v in values]
    bars = ax.bar(["Validated\npairing", "Mis-paired\n(inertial vs CBFQuat)"],
                  heights, color=colors, width=0.55)
    ax.set_ylim(0, 1.05)
    ax.set_ylabel("Capture rate")
    ax.set_title("E1 -- A convention error that raises nothing")
    for bar, value in zip(bars, values):
        ax.annotate(f"{value:.0%}", (bar.get_x() + bar.get_width() / 2,
                                     bar.get_height()),
                    xytext=(0, 4), textcoords="offset points",
                    ha="center", color=INK_PRIMARY, fontsize=10,
                    fontweight="bold")
    # The drop, drawn between the two bars' TOPS rather than floated
    # off to one side, so it reads as connecting the two data points
    # rather than as an unrelated callout. The delta label sits near
    # the HIGH end of the connector, well clear of the "0%" value label
    # anchored at the low bar's own top.
    drop = values[0] - values[1]
    ax.annotate("", xy=(1, heights[1]), xytext=(0, heights[0]),
               arrowprops=dict(arrowstyle="-", color=INK_MUTED, lw=0.8,
                              linestyle=(0, (3, 2))))
    label_x, label_y = 0.28, heights[0] - drop * 0.28
    ax.annotate(f"−{drop:.0%}", xy=(label_x, label_y),
               xytext=(10, 0), textcoords="offset points",
               ha="left", va="center", color=INK_SECONDARY, fontsize=9)
    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# E2 -- does kinematic planning over-promise?
# ---------------------------------------------------------------------------

def e2_capture_vs_coverage_figure(schedulers: Dict[str, Any]):
    """Capture rate and coverage rate, side by side, per scheduler.

    Always both, never one alone -- ``acav/metrics.py`` is explicit that
    either metric in isolation is misleading (a scheduler attempting one
    easy target scores 100% capture rate).
    """
    names = [s for s in ("kinematic", "dynamics_aware") if s in schedulers]
    if not names:
        return _no_data_figure("E2 -- Kinematic vs dynamics-aware",
                               "No scheduler comparison in this campaign.")

    fig, ax = _figure(width=6.0, height=4.2)
    x = list(range(len(names)))
    width = 0.32
    capture = [schedulers[n]["capture_rate"] for n in names]
    coverage = [schedulers[n]["coverage_rate"] for n in names]

    ax.bar([i - width / 2 for i in x], capture, width,
          label="Capture rate", color=CATEGORICAL[0])
    ax.bar([i + width / 2 for i in x], coverage, width,
          label="Coverage rate", color=CATEGORICAL[2])

    ax.set_xticks(x)
    ax.set_xticklabels([scheduler_label(n) for n in names])
    ax.set_ylim(0, 1.05)
    ax.set_ylabel("Rate")
    ax.set_title("E2 -- Capacity claimed vs capacity delivered")
    ax.legend(loc="upper right", ncols=1)

    for i, (c, v) in enumerate(zip(capture, coverage)):
        ax.annotate(f"{c:.0%}", (i - width / 2, c), xytext=(0, 4),
                   textcoords="offset points", ha="center", fontsize=8,
                   color=INK_SECONDARY)
        ax.annotate(f"{v:.0%}", (i + width / 2, v), xytext=(0, 4),
                   textcoords="offset points", ha="center", fontsize=8,
                   color=INK_SECONDARY)

    fig.tight_layout()
    return fig


def e2_outcome_breakdown_figure(schedulers: Dict[str, Any]):
    """Why targets were or were not captured, per scheduler.

    Status colors, because CaptureOutcome IS a status -- distinguishing
    "still slewing" from "geometry never permitted it" is C1's own
    stated reason this taxonomy exists, and only the study's finding
    changes if the two are conflated.
    """
    names = [s for s in ("kinematic", "dynamics_aware") if s in schedulers]
    if not names:
        return _no_data_figure("E2 -- Outcome breakdown",
                               "No scheduler comparison in this campaign.")

    order = ["captured", "not_settled", "no_access",
             "no_access_and_not_settled", "safe_mode", "not_attempted"]
    fig, ax = _figure(width=6.4, height=4.2)
    left = [0.0] * len(names)
    for outcome in order:
        counts = [schedulers[n]["outcomes"].get(outcome, 0) for n in names]
        if not any(counts):
            continue
        ax.barh(range(len(names)), counts, left=left,
               color=STATUS_COLOR[outcome], label=status_label(outcome),
               height=0.5)
        left = [a + b for a, b in zip(left, counts)]

    ax.set_yticks(range(len(names)))
    ax.set_yticklabels([scheduler_label(n) for n in names])
    ax.set_xlabel("Targets")
    ax.set_title("E2 -- Outcome breakdown by scheduler")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.18), ncols=3,
             frameon=False)
    ax.grid(False)
    ax.spines["left"].set_visible(False)
    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# E3 -- does lockstep deliver determinism?
# ---------------------------------------------------------------------------

def e3_determinism_figure(determinism: Dict[str, Any]):
    """Distinct-signature count per mode, and the pointing-error spread.

    Two small panels rather than one chart: "how many distinct outcomes"
    is a count (a bar makes sense), "how much do they differ when they
    do differ" is a magnitude on a different scale entirely (degrees),
    and folding both onto one axis would need the dual-axis chart this
    method rules out on sight.
    """
    modes = [m for m in ("lockstep", "free_run") if m in determinism]
    if not modes:
        return _no_data_figure("E3 -- Determinism",
                               "No execution-mode groups in this campaign.")

    apply_style()
    import matplotlib.pyplot as plt
    fig, (ax_count, ax_spread) = plt.subplots(
        1, 2, figsize=(7.6, 3.6),
        gridspec_kw={"wspace": 0.45})

    signatures = [determinism[m]["distinct_signatures"] for m in modes]
    colors = [MODE_COLOR.get(m, INK_MUTED) for m in modes]
    bars = ax_count.bar([m.replace("_", "-") for m in modes], signatures,
                        color=colors, width=0.5)
    ax_count.set_ylabel("Distinct determinism signatures")
    ax_count.set_title("Repeatability")
    ax_count.set_ylim(0, max(signatures + [1]) * 1.3)
    for bar, value in zip(bars, signatures):
        deterministic = value == 1
        label = "deterministic" if deterministic else f"{value} distinct"
        ax_count.annotate(label, (bar.get_x() + bar.get_width() / 2,
                                  bar.get_height()),
                          xytext=(0, 4), textcoords="offset points",
                          ha="center", fontsize=8,
                          color=(STATUS_COLOR["captured"] if deterministic
                                else STATUS_COLOR["safe_mode"]))

    spread = [determinism[m]["pointing_error_stdev_deg"] for m in modes]
    ax_spread.bar([m.replace("_", "-") for m in modes], spread,
                 color=colors, width=0.5)
    ax_spread.set_ylabel("Pointing error std. dev. (deg)")
    ax_spread.set_title("Spread when it varies")

    fig.suptitle("E3 -- Lockstep vs free-run determinism", y=1.03,
                fontweight="bold", color=INK_PRIMARY)
    return fig


# ---------------------------------------------------------------------------
# E4 -- can the loop tune the controller?
# ---------------------------------------------------------------------------

def e4_gain_surface_figure(surface: List[Dict[str, Any]]):
    """Capture rate over (bandwidth, profile authority) -- a heatmap.

    ``surface`` is ``acav.campaign.gain_surface``'s output: one point per
    combination actually run. Missing combinations are left blank (NaN)
    rather than interpolated -- an interpolated cell would show a value
    for a gain pair the vehicle was never actually flown at.
    """
    import numpy as np

    if not surface:
        return _no_data_figure("E4 -- Gain sweep", "No sweep points yet.")

    frequencies = sorted({p["gains"].get("natural_frequency_rad_s")
                          for p in surface
                          if "natural_frequency_rad_s" in p["gains"]})
    authorities = sorted({p["gains"].get("profile_authority")
                          for p in surface
                          if "profile_authority" in p["gains"]})
    if not frequencies or not authorities:
        return _no_data_figure(
            "E4 -- Gain sweep",
            "Points do not vary natural_frequency_rad_s and "
            "profile_authority together.")

    grid = np.full((len(authorities), len(frequencies)), np.nan)
    for point in surface:
        gains = point["gains"]
        if ("natural_frequency_rad_s" not in gains
                or "profile_authority" not in gains):
            continue
        row = authorities.index(gains["profile_authority"])
        col = frequencies.index(gains["natural_frequency_rad_s"])
        value = point["capture_rate"]
        grid[row, col] = value if value == value else np.nan  # keep NaN

    from matplotlib.colors import LinearSegmentedColormap

    cmap = LinearSegmentedColormap.from_list("acav_sequential_blue",
                                             SEQUENTIAL_BLUE)
    cmap.set_bad(color=GRIDLINE)

    fig, ax = _figure(width=6.2, height=4.6)
    ax.grid(False)
    image = ax.imshow(grid, cmap=cmap, vmin=0.0, vmax=1.0, aspect="auto",
                      origin="lower")
    ax.set_xticks(range(len(frequencies)))
    ax.set_xticklabels([f"{f:.1f}" for f in frequencies])
    ax.set_yticks(range(len(authorities)))
    ax.set_yticklabels([f"{a:.1f}" for a in authorities])
    ax.set_xlabel(r"Natural frequency $\omega_n$ (rad/s)")
    ax.set_ylabel("Profile authority")
    ax.set_title("E4 -- Capture rate over the gain sweep")

    colorbar = fig.colorbar(image, ax=ax, fraction=0.046, pad=0.04)
    colorbar.set_label("Capture rate")
    colorbar.outline.set_visible(False)

    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# E5 -- deterministic fault replay?
# ---------------------------------------------------------------------------

def e5_replay_figure(checks: Sequence[Any]):
    """Reproduction outcome per replayed seed -- a status dot strip.

    ``checks`` is a sequence of ``acav.campaign.ReplayCheck``. This is
    deliberately not a bar chart: there is nothing to sum or compare in
    magnitude, only a per-seed pass/fail, which a dot strip shows
    without implying an axis the data does not have.
    """
    if not checks:
        return _no_data_figure("E5 -- Fault replay",
                               "No replay checks in this campaign.")

    fig, ax = _figure(width=max(4.0, 0.5 * len(checks) + 1.5), height=2.6)
    xs = list(range(len(checks)))
    colors = [STATUS_COLOR["captured"] if c.reproduced
             else STATUS_COLOR["safe_mode"] for c in checks]
    ax.scatter(xs, [0] * len(xs), c=colors, s=160, zorder=3,
              edgecolors="white", linewidths=1.0)
    ax.set_yticks([])
    ax.set_xticks(xs)
    ax.set_xticklabels([c.original_run_id.split("-seed")[-1]
                        for c in checks])
    ax.set_xlabel("Seed")
    ax.set_ylim(-1, 1)
    ax.grid(False)
    for spine in ("left", "bottom"):
        ax.spines[spine].set_visible(False)
    reproduced = sum(1 for c in checks if c.reproduced)
    ax.set_title(f"E5 -- Replay reproduced {reproduced}/{len(checks)} seeds")
    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# E6 -- what does it cost?
# ---------------------------------------------------------------------------

def e6_latency_figure(breakdown: Dict[str, Any]):
    """One stacked horizontal bar: sense + actuate + barrier wait.

    Three components of one step, so a stack -- not three separate bars,
    which would invite comparing them as alternatives rather than parts
    of the total the achieved step rate is measured against.
    """
    import math

    phases = [("mean_sense_latency_ms", "Sense", CATEGORICAL[0]),
             ("mean_actuate_latency_ms", "Actuate", CATEGORICAL[2]),
             ("mean_barrier_wait_ms", "Barrier wait", CATEGORICAL[4])]
    values = [breakdown.get(key, float("nan")) for key, _, _ in phases]
    if not values or all(v != v for v in values):
        return _no_data_figure(
            "E6 -- Step latency breakdown",
            "No reportable runs with a measured breakdown yet.")
    values = [0.0 if v != v else v for v in values]

    fig, ax = _figure(width=7.0, height=3.5)
    left = 0.0
    for value, (_, label, color) in zip(values, phases):
        ax.barh([0], [value], left=left, color=color, height=0.45,
               label=f"{label}: {value:.1f} ms")
        left += value

    ax.set_yticks([])
    ax.set_xlim(0, left * 1.05 if left > 0 else 1)
    ax.set_xlabel("Mean wall time per step (ms)")
    total = breakdown.get("mean_step_latency_ms", float("nan"))
    if math.isfinite(total) and total > 0:
        ax.set_title(f"E6 -- measured total {total:.1f} ms/step")
        note = (f"Component sum: {left:.1f} ms; total minus sum: {total-left:.1f} ms\n"
                "Total and components have separate timing/aggregation boundaries.")
    else:
        ax.set_title("E6 -- measured latency components")
        note = f"Component sum: {left:.1f} ms; end-to-end total unavailable."
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.28), ncol=3,
              fontsize=8, frameon=False)
    fig.text(.5, .02, note, ha="center", va="bottom", fontsize=8)
    ax.grid(False)
    for spine in ("left",):
        ax.spines[spine].set_visible(False)
    fig.tight_layout(rect=(0, .15, 1, 1))
    return fig


# ---------------------------------------------------------------------------
# Saving
# ---------------------------------------------------------------------------

def save_all(figures: Dict[str, Any], output_dir: str,
            formats: Sequence[str] = ("pdf", "svg", "png")) -> List[str]:
    """Write every figure in ``figures`` (name -> Figure) to disk.

    Multiple formats by default: PDF/SVG are what a paper's typesetting
    actually wants (vector, no resampling artefacts at print size); PNG
    is what is easiest to preview without opening a PDF viewer -- e.g.
    reading it back here, or pasting it into a quick status update.
    """
    apply_style()
    import matplotlib.pyplot as plt

    os.makedirs(output_dir, exist_ok=True)
    written = []
    for name, figure in figures.items():
        for fmt in formats:
            path = os.path.join(output_dir, f"{name}.{fmt}")
            figure.savefig(path)
            written.append(path)
        plt.close(figure)
    return written
