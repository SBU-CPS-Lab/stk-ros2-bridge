"""Chart styling for C6's figures -- one place, applied consistently.

Every figure in :mod:`acav.report_figures` is a static, print-destined
image (the paper, not a dashboard), so the concerns are different from
an interactive chart's: no hover, no dark mode, no legend that needs to
survive a filter change. What still applies, because it is what makes a
figure look designed rather than defaulted, is the color and mark
discipline: categorical hues assigned in a fixed order and never
recycled, one hue ramp for magnitude, status colors reserved for
outcome states, thin marks, a light surface, and no legend box heavier
than the data it labels.

The hex values below are a validated categorical/sequential/status
palette (colorblind-safe adjacent pairs, checked contrast), not house
colors picked by eye. Matplotlib's own defaults -- the tab10 cycle, a
heavy grid, drop-shadowed legend boxes -- are exactly the "obviously a
default" look this module exists to avoid.
"""

from __future__ import annotations

from typing import Dict

# ---------------------------------------------------------------------------
# The palette -- light surface only; these figures are printed, not viewed
# in a themed app, so there is no dark variant to carry.
# ---------------------------------------------------------------------------

SURFACE = "#fcfcfb"
INK_PRIMARY = "#0b0b0b"
INK_SECONDARY = "#52514e"
INK_MUTED = "#898781"
GRIDLINE = "#e1e0d9"
BASELINE = "#c3c2b7"

#: Categorical hues, in FIXED order. Never cycle past the slot a series
#: was already assigned, and never reassign a hue when a filter changes
#: which series are present -- see this project's schedulers, which
#: always get slots 1/2 regardless of which experiment is plotting them.
CATEGORICAL = [
    "#2a78d6",  # 1 blue       -- dynamics_aware (the vehicle as it is)
    "#eb6834",  # 2 orange     -- kinematic (the assumption under test)
    "#1baf7a",  # 3 aqua
    "#eda100",  # 4 yellow
    "#e87ba4",  # 5 magenta
    "#008300",  # 6 green
    "#4a3aa7",  # 7 violet
    "#e34948",  # 8 red
]

#: Fixed roles, so "dynamics_aware" is always the same color across
#: every figure in a report rather than whatever slot 1 happens to be
#: for that particular plot's series order.
SCHEDULER_COLOR: Dict[str, str] = {
    "dynamics_aware": CATEGORICAL[0],
    "kinematic": CATEGORICAL[1],
}
MODE_COLOR: Dict[str, str] = {
    "lockstep": CATEGORICAL[0],
    "free_run": CATEGORICAL[1],
}

#: Sequential ramp, one hue (blue), light -> dark. For magnitude only
#: (heatmaps); never used for identity.
SEQUENTIAL_BLUE = [
    "#cde2fb", "#b7d3f6", "#9ec5f4", "#86b6ef", "#6da7ec",
    "#5598e7", "#3987e5", "#2a78d6", "#256abf", "#1c5cab",
    "#184f95", "#104281", "#0d366b",
]

#: Status colors -- reserved for CaptureOutcome, never reused as a
#: series color. Fixed regardless of which outcomes are present in a
#: given run, so "captured" is always this green across every figure.
STATUS_COLOR: Dict[str, str] = {
    "captured": "#0ca30c",           # good
    "not_settled": "#fab219",        # warning -- geometry was there,
                                      # dynamics were not
    "no_access": "#ec835a",          # serious -- pointing was fine,
                                      # geometry never permitted it
    "no_access_and_not_settled": "#d03b3b",  # critical -- neither held
    "safe_mode": "#d03b3b",          # critical -- the vehicle tripped
    "not_attempted": "#c3c2b7",      # muted -- a scheduling choice,
                                      # not a failure
    "no_telemetry": "#7d7b70",       # unscored -- the twin was silent
                                      # through the hold window
    "no_access_answer": "#4f4e46",   # unscored -- STK never answered
    "access_unverified": "#a3a196",  # unscored -- the attitude never
                                      # reached the image epoch
}

#: Diverging pair, for a signed quantity (unused by the current figures
#: but kept here so a future one does not invent a second scheme).
DIVERGING = ("#2a78d6", "#e34948")   # blue <-> red
DIVERGING_MIDPOINT = "#f0efec"

FONT_FAMILY = ["DejaVu Sans", "Arial", "Helvetica", "sans-serif"]


def apply_style() -> None:
    """Set matplotlib rcParams once, before any figure in this module
    is built. Idempotent -- safe to call more than once.

    Also forces the non-interactive Agg backend, before anything imports
    ``pyplot``: these figures are rendered by ``run_acav_campaign
    report``, which has no reason to run anywhere with a display, and an
    interactive backend (GTK, Qt) picked up from whatever happens to be
    installed will raise or hang on a headless deploy or CI box with no
    X server. ``matplotlib.use`` must run before the first ``import
    matplotlib.pyplot`` in the process or it has no effect -- safe here
    because every figure function imports pyplot lazily, and this is
    always the first thing any of them calls.
    """
    import matplotlib

    matplotlib.use("Agg", force=True)
    matplotlib.rcParams.update({
        "figure.facecolor": SURFACE,
        "axes.facecolor": SURFACE,
        "savefig.facecolor": SURFACE,
        "font.family": "sans-serif",
        "font.sans-serif": FONT_FAMILY,
        "font.size": 10,
        "text.color": INK_PRIMARY,
        "axes.edgecolor": BASELINE,
        "axes.labelcolor": INK_SECONDARY,
        "axes.titlecolor": INK_PRIMARY,
        "axes.titleweight": "bold",
        "axes.titlesize": 12,
        "axes.labelsize": 10,
        "axes.linewidth": 0.8,
        # Only the axes a reader actually reads: no boxed plot area.
        "axes.spines.top": False,
        "axes.spines.right": False,
        "xtick.color": INK_MUTED,
        "ytick.color": INK_MUTED,
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
        "grid.color": GRIDLINE,
        "grid.linewidth": 0.6,
        "axes.grid": True,
        "axes.grid.axis": "y",
        "axes.axisbelow": True,
        "legend.frameon": False,
        "legend.fontsize": 9,
        "lines.linewidth": 2.0,
        "lines.markersize": 6.0,
        "patch.linewidth": 0.0,
        "figure.dpi": 150,
        "savefig.dpi": 300,        # print resolution on export
        "savefig.bbox": "tight",
    })


def status_label(outcome: str) -> str:
    """A reader-facing label for a CaptureOutcome value."""
    return {
        "captured": "Captured",
        "not_settled": "Not settled",
        "no_access": "No access",
        "no_access_and_not_settled": "No access, not settled",
        "safe_mode": "SAFE mode",
        "not_attempted": "Not attempted",
        "no_telemetry": "No telemetry",
        "no_access_answer": "No access answer",
        "access_unverified": "Access unverified",
    }.get(outcome, outcome.replace("_", " ").title())


def scheduler_label(scheduler: str) -> str:
    return {"dynamics_aware": "Dynamics-aware",
            "kinematic": "Kinematic"}.get(
        scheduler, scheduler.replace("_", " ").title())
