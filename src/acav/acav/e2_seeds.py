"""E2 over the seeded scenarios, in one command (run_acav_campaign run-seeds).

For each seed: if its folder already holds a usable record for every run
of the E2 campaign, made by this commit, skip it; otherwise build and
verify the seeded scenario in the user's STK (build_acav_scenario:
an open scenario is closed first, and graphics are left off and checked
with the run guard's own rule), stop that seed on any non-zero result,
then run the E2 campaign into results/E2_final_seed{k}_{tag}. Records
from another commit in a seed folder are refused by the campaign runner,
not resumed. Then rebuild the reference scenario, run the reference arm
into results/E2_final_ref_{tag}, and produce the reference report and
the paired report over the seed folders (--include-dir). Prints a
summary; the exit code is non-zero if anything failed.

Builds and campaign invocations are injected, so this is testable
without STK; the CLI (campaign_cli, verb run-seeds) checks that the
tree is clean and that the tag is HEAD's before calling it.
"""

from __future__ import annotations

import os
import sys
from typing import Callable, Dict, List, Optional, Sequence

#: The seeded scenarios, relative to the workspace (the current
#: directory): through `ros2 run` this package is the installed copy,
#: which has no generated scenarios beside it.
SCENARIO_DIR = os.path.join("src", "acav", "config", "scenarios",
                            "generated")


def parse_seeds(text: str) -> List[int]:
    """'1-10' or '1,3,5' (or a mix: '1-3,7')."""
    seeds: List[int] = []
    for part in str(text).split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            low, high = (int(v) for v in part.split("-", 1))
            seeds.extend(range(low, high + 1))
        else:
            seeds.append(int(part))
    if not seeds or len(set(seeds)) != len(seeds):
        raise ValueError(f"bad seed list {text!r}")
    return seeds


def seed_scenario(seed: int, scenario_dir: str = "") -> str:
    """Absolute path of seed's scenario: the same path goes to the
    builder, to the campaign (--scenario) and so to {scenario}."""
    return os.path.join(os.path.abspath(scenario_dir or SCENARIO_DIR),
                        f"acav_seed{seed}.yaml")


def usable_runs(out: str, scenario: str, version: Optional[str] = None
                ) -> Sequence[int]:
    """(usable records, requested runs) of the E2 campaign in ``out``;
    a record made by another commit than ``version`` is not usable."""
    from acav.campaign import build_e2_scheduler_comparison
    from acav.campaign_runner import _load_if_usable
    from acav.run_record import scenario_targets

    campaign = build_e2_scheduler_comparison(scenario, "com")
    targets = scenario_targets(scenario)
    usable = 0
    for run in campaign.runs:
        record = _load_if_usable(run, os.path.join(out, run.output_name),
                                 targets)
        if record is not None and (
                version is None or record.provenance.code_version == version):
            usable += 1
    return usable, len(campaign.runs)


def run_seeds(seeds: str, tag: str, results_dir: str, backend: str,
              twin_launch: str, launch: List[str],
              build: Callable[[str], int],
              campaign: Callable[[List[str]], int],
              is_complete: Optional[Callable[[str, str], bool]] = None,
              version: Optional[str] = None,
              scenario_dir: str = "") -> int:
    results_dir = os.path.abspath(results_dir)
    seed_list = parse_seeds(seeds)
    missing = [seed_scenario(s, scenario_dir) for s in seed_list
               if not os.path.isfile(seed_scenario(s, scenario_dir))]
    if missing:
        print("run-seeds: seeded scenario file(s) not found (run from the "
              "workspace root, or give --scenario-dir); nothing was "
              "built:\n  " + "\n  ".join(missing), file=sys.stderr)
        return 2

    def complete(out, scenario):
        if is_complete is not None:
            return is_complete(out, scenario)
        usable, requested = usable_runs(out, scenario, version)
        return usable == requested

    def run_arm(out, scenario):
        argv = ["run", "--experiment", "E2", "--output-dir", out,
                "--backend", backend, "--twin-launch", twin_launch]
        if scenario:
            argv += ["--scenario", scenario]
        return campaign(argv + ["--"] + list(launch))

    rows: List[Dict] = []
    failed = False
    seed_folders: List[str] = []
    for seed in seed_list:
        scenario = seed_scenario(seed, scenario_dir)
        out = os.path.join(results_dir, f"E2_final_seed{seed}_{tag}")
        seed_folders.append(out)
        row = {"arm": f"seed {seed}", "folder": out, "build": "skipped",
               "run": "skipped"}
        rows.append(row)
        if complete(out, scenario):
            row["build"] = row["run"] = "already complete"
            continue
        code = build(scenario)
        row["build"] = "ok" if code == 0 else f"FAILED (exit {code})"
        if code != 0:
            failed = True
            continue
        code = run_arm(out, scenario)
        row["run"] = "ok" if code == 0 else f"FAILED (exit {code})"
        failed = failed or code != 0

    ref_out = os.path.join(results_dir, f"E2_final_ref_{tag}")
    ref = {"arm": "reference", "folder": ref_out}
    rows.append(ref)
    code = build("")
    ref["build"] = "ok" if code == 0 else f"FAILED (exit {code})"
    reports = {}
    if code != 0:
        failed = True
        ref["run"] = "skipped"
    else:
        code = run_arm(ref_out, "")
        ref["run"] = "ok" if code == 0 else f"FAILED (exit {code})"
        failed = failed or code != 0
        reports["reference report"] = campaign(
            ["report", "--experiment", "E2", "--output-dir", ref_out,
             "--no-figures"])
        paired = ["report", "--experiment", "E2", "--no-figures",
                  "--output-dir", seed_folders[0]]
        for folder in seed_folders[1:]:
            paired += ["--include-dir", folder]
        reports["paired report"] = campaign(paired)
        failed = failed or any(c != 0 for c in reports.values())

    print("\nE2 run-seeds summary:")
    for row in rows:
        scenario = (seed_scenario(int(row["arm"].split()[1]), scenario_dir)
                    if row["arm"].startswith("seed") else "")
        try:
            usable, requested = usable_runs(row["folder"], scenario or
                                            _reference(), version)
            runs = f"{usable}/{requested}"
        except Exception:                       # noqa: BLE001 - summary only
            runs = "?"
        print(f"  {row['arm']:10s} build {row['build']:22s} run "
              f"{row['run']:22s} usable runs {runs:5s} {row['folder']}")
    for name, code in reports.items():
        print(f"  {name}: exit {code}")
    if not reports:
        print("  reports: not run (the reference build failed)")
    return 1 if failed else 0


def _reference() -> str:
    from acav.spec import AcavSpec
    return AcavSpec.locate(None)
