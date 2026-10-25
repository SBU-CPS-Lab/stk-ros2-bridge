"""E2 paper figures from the final E2 records (commit 91f36b9).

Reads results/E2_final_seed{1..10}_91f36b9 and
results/E2_final_paired_91f36b9/paired_report.json, writes PDF/SVG/PNG into
results/E2_final_paired_91f36b9/figures. Uses the project's own figure style
(acav.report_style). Nothing here recomputes a statistic: p values, medians
and CIs are read from paired_report.json; per-seed values come straight from
the records.

Run from stk_ros2_ws:  python results/E2_final_paired_91f36b9/figures/make_e2_figures.py
"""
import json
import os
import sys
from collections import Counter

WS = os.getcwd()
sys.path.insert(0, os.path.join(WS, "src", "acav"))
from acav.report_style import (SCHEDULER_COLOR, STATUS_COLOR, INK_MUTED,  # noqa: E402
                               INK_SECONDARY, GRIDLINE, apply_style)

TAG = "91f36b9"
SEEDS = range(1, 11)
ARMS = ("kinematic", "dynamics_aware")
LABEL = {"kinematic": "Kinematic", "dynamics_aware": "Dynamics-aware"}
RES = os.path.join(WS, "results")
OUT = os.path.join(RES, f"E2_final_paired_{TAG}", "figures")

apply_style()
import matplotlib.pyplot as plt  # noqa: E402


def load():
    data = {}
    for k in SEEDS:
        for arm in ARMS:
            path = os.path.join(RES, f"E2_final_seed{k}_{TAG}", f"E2-{arm}-lockstep.json")
            rec = json.load(open(path, encoding="utf-8"))
            assert rec["reportable"], path
            assert rec["provenance"]["code_version"].startswith(TAG), path
            assert rec["provenance"]["scenario_seed"] == k, path
            r = rec["result"]
            n = len(r["targets"])
            att = r["attempted_count"]
            data[(k, arm)] = {
                "capture_rate": r["captured_count"] / att,
                "coverage_rate": r["captured_count"] / n,
                "wheel_energy_mJ": 1000.0 * r["wheel_energy_J"],
                "wheel_saturation_events": r["wheel_saturation_events"],
                "outcomes": Counter(t["outcome"] for t in r["targets"]),
            }
    paired = json.load(open(os.path.join(RES, f"E2_final_paired_{TAG}", "paired_report.json"),
                            encoding="utf-8"))["paired"]
    return data, paired


def paired_panel(ax, data, paired, metric, key, ylabel, title, ylim=None):
    xs = (0, 1)
    for k in SEEDS:
        ys = [data[(k, a)][key] for a in ARMS]
        ax.plot(xs, ys, color=GRIDLINE, linewidth=1.0, zorder=1)
    for x, arm in zip(xs, ARMS):
        ys = [data[(k, arm)][key] for k in SEEDS]
        ax.scatter([x] * len(ys), ys, s=28, color=SCHEDULER_COLOR[arm], zorder=3,
                   edgecolor="white", linewidth=0.6)
        # Seeds with identical values sit on one dot; say how many.
        for v, c in Counter(round(y, 9) for y in ys).items():
            if c > 1:
                dx = -0.22 if x == 0 else 0.22
                ax.text(x + dx, v, f"\u00d7{c}", ha="right" if x == 0 else "left",
                        va="center", fontsize=8, color=INK_SECONDARY, zorder=5)
        med = sorted(ys)[len(ys) // 2 - 1: len(ys) // 2 + 1]
        m = sum(med) / 2
        ax.hlines(m, x - 0.18, x + 0.18, color=SCHEDULER_COLOR[arm], linewidth=2.2, zorder=4)
    s = paired["metrics"][metric]
    scale = 1000.0 if key == "wheel_energy_mJ" else 1.0
    unit = " mJ" if key == "wheel_energy_mJ" else ""
    ax.set_xticks(xs)
    ax.set_xticklabels([LABEL[a] for a in ARMS])
    ax.set_xlim(-0.45, 1.45)
    if ylim:
        ax.set_ylim(*ylim)
    ax.set_ylabel(ylabel)
    ax.set_title(title, loc="left", fontsize=11)
    ax.text(0.5, 0.02,
            f"median paired diff {scale * s['median_difference']:+.3g}{unit}\n"
            f"Holm p = {s['p_holm']:.3g}, n = {s['n_pairs']} seeds",
            transform=ax.transAxes, ha="center", va="bottom", fontsize=8.5, color=INK_SECONDARY)


def fig_capture_coverage(data, paired):
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.6))
    paired_panel(axes[0], data, paired, "capture_rate", "capture_rate",
                 "Capture rate (captured / attempted)", "(a) Capture rate", (0.6, 1.03))
    paired_panel(axes[1], data, paired, "coverage_rate", "coverage_rate",
                 "Coverage rate (captured / all 22 targets)", "(b) Coverage rate", (0.6, 1.03))
    fig.tight_layout()
    return fig


def fig_resources(data, paired):
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.6))
    paired_panel(axes[0], data, paired, "wheel_energy_J", "wheel_energy_mJ",
                 "Reaction-wheel energy (mJ)", "(a) Wheel energy")
    paired_panel(axes[1], data, paired, "wheel_saturation_events", "wheel_saturation_events",
                 "Wheel saturation events", "(b) Wheel saturation")
    for ax in axes:
        lo, hi = ax.get_ylim()
        ax.set_ylim(lo - 0.15 * (hi - lo), hi)
    fig.tight_layout()
    return fig


def fig_outcomes(data):
    order = ["captured", "not_settled", "no_access", "no_access_and_not_settled",
             "safe_mode", "no_telemetry", "no_access_answer", "access_unverified",
             "not_attempted"]
    names = {"captured": "Captured", "not_settled": "Attempted, not settled",
             "no_access": "No access", "no_access_and_not_settled": "No access and not settled",
             "safe_mode": "Safe mode", "no_telemetry": "No telemetry",
             "no_access_answer": "No access answer", "access_unverified": "Access unverified",
             "not_attempted": "Not attempted (screened out)"}
    totals = {a: sum((data[(k, a)]["outcomes"] for k in SEEDS), Counter()) for a in ARMS}
    fig, ax = plt.subplots(figsize=(7.2, 2.8))
    rows = list(ARMS)
    left = [0] * len(rows)
    for o in order:
        c = [totals[a].get(o, 0) for a in rows]
        if not any(c):
            continue
        bars = ax.barh(range(len(rows)), c, left=left, color=STATUS_COLOR[o],
                       label=names[o], height=0.5)
        for b, v, l in zip(bars, c, left):
            if v >= 8:
                ax.text(l + v / 2, b.get_y() + b.get_height() / 2, str(v), ha="center",
                        va="center", fontsize=8.5, color="white" if o == "captured" else "black")
        left = [a + b for a, b in zip(left, c)]
    ax.set_yticks(range(len(rows)))
    ax.set_yticklabels([LABEL[a] for a in rows])
    ax.set_xlabel("Targets, summed over 10 scenario seeds (22 targets each, 220 total)")
    ax.set_xlim(0, 220)
    ax.grid(False)
    ax.spines["left"].set_visible(False)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.32), ncols=3, frameon=False)
    fig.tight_layout()
    return fig, totals


def main():
    data, paired = load()
    os.makedirs(OUT, exist_ok=True)
    figs = {"E2_capture_coverage_paired": fig_capture_coverage(data, paired),
            "E2_wheel_resources_paired": fig_resources(data, paired)}
    f, totals = fig_outcomes(data)
    figs["E2_outcome_breakdown"] = f
    for name, fig in figs.items():
        for fmt in ("pdf", "svg", "png"):
            fig.savefig(os.path.join(OUT, f"{name}.{fmt}"), dpi=300 if fmt == "png" else None,
                        bbox_inches="tight")
        plt.close(fig)
    with open(os.path.join(OUT, "E2_figure_data.json"), "w", encoding="utf-8") as h:
        json.dump({"per_seed": {f"seed{k}_{a}": {kk: v for kk, v in data[(k, a)].items()
                                                  if kk != "outcomes"} | {"outcomes": dict(data[(k, a)]["outcomes"])}
                                for k in SEEDS for a in ARMS},
                   "outcome_totals": {a: dict(t) for a, t in totals.items()}}, h, indent=2)
    print("written:", sorted(os.listdir(OUT)))
    print("outcome totals:", {a: dict(t) for a, t in totals.items()})


if __name__ == "__main__":
    main()
