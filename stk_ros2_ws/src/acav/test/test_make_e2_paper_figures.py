"""scripts/make_e2_paper_figures.py: the three E2 paper figures.

The script reads results/E2_final_seed{1..10}_91f36b9 and
results/E2_final_paired_91f36b9/paired_report.json under the current
directory. Here it runs on a small fake workspace and must write the
three figures in each format.
"""

import json
import os
import subprocess
import sys

import pytest

pytest.importorskip("matplotlib")

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE = os.path.dirname(HERE)
SCRIPT = os.path.join(PACKAGE, "scripts", "make_e2_paper_figures.py")
TAG = "91f36b9"
FIGURES = ("E2_capture_coverage_paired", "E2_wheel_resources_paired",
           "E2_outcome_breakdown")


def _record(seed, scheduler):
    captured = 20 if scheduler == "dynamics_aware" else 17
    outcomes = (["captured"] * captured + ["not_settled"] * (20 - captured)
                + ["not_attempted"] * 2)
    return {
        "reportable": True,
        "provenance": {"code_version": TAG + "0" * 33, "scenario_seed": seed},
        "result": {
            "targets": [{"outcome": o} for o in outcomes],
            "attempted_count": 20,
            "captured_count": captured,
            "wheel_energy_J": 0.01 * seed + (0.002 if captured == 20 else 0),
            "wheel_saturation_events": seed % 4,
        },
    }


def _workspace(root):
    results = root / "results"
    for seed in range(1, 11):
        folder = results / f"E2_final_seed{seed}_{TAG}"
        folder.mkdir(parents=True)
        for scheduler in ("kinematic", "dynamics_aware"):
            (folder / f"E2-{scheduler}-lockstep.json").write_text(
                json.dumps(_record(seed, scheduler)), encoding="utf-8")
    metric = {"median_difference": 0.1, "p_holm": 0.01, "n_pairs": 10}
    paired = {"paired": {"metrics": {name: dict(metric) for name in (
        "capture_rate", "coverage_rate", "wheel_energy_J",
        "wheel_saturation_events")}}}
    out = results / f"E2_final_paired_{TAG}"
    out.mkdir()
    (out / "paired_report.json").write_text(json.dumps(paired),
                                            encoding="utf-8")
    return out / "figures"


def test_the_three_figures_are_written(tmp_path):
    figures = _workspace(tmp_path)
    env = dict(os.environ, PYTHONPATH=PACKAGE, MPLBACKEND="Agg")
    done = subprocess.run([sys.executable, SCRIPT], cwd=str(tmp_path),
                          env=env, capture_output=True, text=True,
                          timeout=300)
    assert done.returncode == 0, done.stderr
    for name in FIGURES:
        for fmt in ("pdf", "svg", "png"):
            path = figures / f"{name}.{fmt}"
            assert path.is_file() and path.stat().st_size > 0, path
    assert (figures / "E2_figure_data.json").is_file()
