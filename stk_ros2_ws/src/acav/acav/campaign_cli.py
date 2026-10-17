"""Command-line driver for the E2-E5 campaigns.

    run_acav_campaign plan   --experiment E3 --output-dir results/E3
    run_acav_campaign run    --experiment E3 --output-dir results/E3 -- \\
        python launch_one_run.py --run {run_id} --mode {execution_mode} \\
                                 --scheduler {scheduler} --out {output}
    run_acav_campaign report --experiment E3 --output-dir results/E3

Three verbs because a campaign has three moments that fail differently.
``plan`` is cheap and answers "is this the matrix I meant?" before an
overnight run commits to it. ``run`` executes it and can be re-issued
after a crash without repeating completed runs. ``report`` reads the
directory afterwards and is the only one that needs no launch command,
so results can be re-analysed on a machine that has neither STK nor
MATLAB -- which is where the analysis for the paper actually happens.

The launch command after ``--`` is the operator's, and deliberately so:
the launch topology (bridge, twin, scheduler, shutdown order) belongs to
the machine with STK on it. See :class:`acav.campaign_runner.
CommandLauncher` for the placeholders it can use.

What the exit code means
------------------------
0  the verb succeeded. For ``report`` this means the campaign's own
   claim held: lockstep was deterministic, or every replay reproduced.
1  the campaign ran but the result does not support its claim, or some
   runs are missing or did not match what was asked for.
2  the invocation was wrong -- an unknown experiment, no launch command,
   an unreadable directory. Nothing was measured either way.
"""

from __future__ import annotations

import argparse
import glob
import json
import logging
import math
import os
import sys
from typing import List, Optional

from acav.campaign import (
    LOCKSTEP,
    Campaign,
    E1_EXPECTED_NULL,
    E1_MISMATCH_TYPES,
    build_e1_matrix,
    build_e3_matrix,
    combined_plan_lines,
    e3_counts,
    e3_formula_line,
    estimate_lines,
    wall_time_lines,
    e1_arm,
    e1_formula_line,
    build_e2_scheduler_comparison,
    build_e6_latency,
    build_e5_fault_campaign,
    compare_schedulers,
    convention_comparison,
    determinism_comparison,
    latency_breakdown,
)
from acav import code_version
from acav.metrics import capture_rate, coverage_rate, outcome_breakdown
from acav.campaign_runner import (
    CampaignRefused,
    CommandLauncher,
    RunnerOptions,
    run_campaign,
)
from acav.run_record import RunRecord, read_record, \
    target_coverage_problems, unreportable
from acav.spec import AcavSpec

LOGGER = logging.getLogger(__name__)

#: E4 (gain sweep) was dropped from the study and removed; records
#: labelled E4 still load (load_results never checks the label).
EXPERIMENTS = ("E1", "E2", "E3", "E5", "E6")


#: --repeats when it is not given: E1 one per arm, E3 and E6 as before.
DEFAULT_REPEATS = {"E1": 1, "E3": 20, "E6": 20}


def build_campaign(experiment: str, spec: AcavSpec, scenario_path: str,
                   backend: str, repeats: Optional[int], seeds: int,
                   e1_types=E1_MISMATCH_TYPES,
                   fault_seeds: int = 0) -> Campaign:
    """The matrix for ``experiment``, sized from the scenario."""
    if repeats is None:
        repeats = DEFAULT_REPEATS.get(experiment, 20)
    if experiment == "E1":
        return build_e1_matrix(tuple(e1_types), repeats, scenario_path,
                               backend)
    if experiment == "E2":
        return build_e2_scheduler_comparison(scenario_path, backend)
    if experiment == "E3":
        return build_e3_matrix(repeats, fault_seeds, spec.window_duration_s,
                               scenario_path, backend)
    if experiment == "E6":
        return build_e6_latency(repeats, scenario_path, backend)
    if experiment == "E5":
        return build_e5_fault_campaign(
            list(range(seeds)), spec.window_duration_s,
            scenario_path=scenario_path, backend=backend)
    raise ValueError(f"unknown experiment {experiment!r}; "
                     f"expected one of {list(EXPERIMENTS)}")


def load_results(output_dir: str) -> List[RunRecord]:
    """Every readable record in ``output_dir``.

    Unreadable ones are reported and skipped rather than aborting: a
    directory with one corrupt file out of a hundred should still yield
    an analysis of the ninety-nine, with the gap stated.
    """
    records = []
    for path in sorted(glob.glob(os.path.join(output_dir, "*.json"))):
        name = os.path.basename(path)
        # campaign.json is the manifest; *.runconfig.json is the run's
        # gain-and-fault input CommandLauncher writes BESIDE each
        # result (see campaign_runner.write_run_config) -- both live in
        # the same directory as the records and match *.json, but
        # neither is one.
        if name == "campaign.json" or name.endswith(".runconfig.json"):
            continue
        try:
            records.append(read_record(path))
        except Exception as error:              # noqa: BLE001
            LOGGER.error("skipping %s: %s", path, error)
    return records


def diagnostic_runs(output_dir: str) -> List[str]:
    """Runs in ``output_dir`` made with the sensing equivalence diagnostic.

    Read from each run's ``<run>.logs/summary.json`` (written by
    run_acav_run.py). Such a run's timings are inflated by the
    diagnostic's extra queries, so it must never be pooled with ordinary
    runs.
    """
    found = []
    for path in sorted(glob.glob(os.path.join(output_dir, "*.logs",
                                              "summary.json"))):
        try:
            with open(path, encoding="utf-8") as handle:
                summary = json.load(handle)
        except (OSError, ValueError):
            continue
        if summary.get("equivalence_diagnostic") is True:
            found.append(str(summary.get("run_id") or path))
    return found


def labelled_runs(output_dir: str) -> List[tuple]:
    """(run_id, label) of runs in ``output_dir`` with a diagnostic label.

    Set by run_acav_run.py --diagnostic-label for one-off measurements
    that must never be pooled with ordinary runs.
    """
    found = []
    for path in sorted(glob.glob(os.path.join(output_dir, "*.logs",
                                              "summary.json"))):
        try:
            with open(path, encoding="utf-8") as handle:
                summary = json.load(handle)
        except (OSError, ValueError):
            continue
        label = str(summary.get("diagnostic_label") or "")
        if label:
            found.append((str(summary.get("run_id") or path), label))
    return found


def effective_real_time_factors(output_dir: str) -> dict:
    """Effective real-time factor -> run ids, for the records in a folder.

    The recorded provenance.real_time_factor when present. A record
    without it (written before it was recorded) counts as 1.0 only if it
    is lockstep -- lockstep time advances by steps, never by the wall
    clock, so the factor cannot have shaped it -- or if its bridge.log
    shows "rtf=1.000"; otherwise it is "unknown", which report refuses.
    """
    import re

    found: dict = {}
    for path in sorted(glob.glob(os.path.join(output_dir, "*.json"))):
        name = os.path.basename(path)
        if name == "campaign.json" or name.endswith(".runconfig.json"):
            continue
        try:
            record = read_record(path)
        except Exception:                       # noqa: BLE001 - load_results
            continue
        value = record.provenance.real_time_factor
        if value is None:
            if record.provenance.execution_mode == "lockstep":
                value = 1.0
            else:
                log = os.path.join(path[:-len(".json")] + ".logs",
                                   "bridge.log")
                try:
                    with open(log, encoding="utf-8", errors="replace") as f:
                        match = re.search(r"time sync started: .*?rtf=([0-9.]+)",
                                          f.read())
                except OSError:
                    match = None
                value = float(match.group(1)) if match else "unknown"
        found.setdefault(value, []).append(record.result.run_id)
    return {value: sorted(ids) for value, ids in found.items()}


def twin_launch_modes(output_dir: str) -> dict:
    """summary.json twin_launch -> run ids ("manual" when absent: every run
    before --twin-launch existed was started by hand)."""
    modes: dict = {}
    for path in sorted(glob.glob(os.path.join(output_dir, "*.logs",
                                              "summary.json"))):
        try:
            with open(path, encoding="utf-8") as handle:
                summary = json.load(handle)
        except (OSError, ValueError):
            continue
        mode = str(summary.get("twin_launch") or "manual")
        modes.setdefault(mode, []).append(
            str(summary.get("run_id") or path))
    return {mode: sorted(ids) for mode, ids in modes.items()}


def stk_start_modes(records: List[RunRecord]) -> dict:
    """provenance.stk_process started_by -> run ids ("" for records
    without one: other backends, and records made before it existed)."""
    from acav.stk_process import start_mode

    modes: dict = {}
    for record in records:
        modes.setdefault(start_mode(record.provenance.stk_process),
                         []).append(record.result.run_id)
    return {mode: sorted(ids) for mode, ids in modes.items()}


def graphics_modes(records: List[RunRecord]) -> dict:
    """provenance.graphics_mode -> run ids ("" for records without one)."""
    modes: dict = {}
    for record in records:
        modes.setdefault(record.provenance.graphics_mode, []).append(
            record.result.run_id)
    return {mode: sorted(ids) for mode, ids in modes.items()}


def code_version_summary(records: List[RunRecord]) -> dict:
    """Which code produced these records, and whether that is one commit.

    ``attributable`` is true only when every record names the same plain
    commit: records from older code (no version recorded), from modified
    code (``+dirty``) or from more than one commit cannot be attributed to
    a single published version, which a paper's methods section needs.
    """
    from acav.code_version import is_clean

    by_version: dict = {}
    for record in records:
        version = record.provenance.code_version or "not recorded"
        by_version.setdefault(version, []).append(record.result.run_id)
    versions = sorted(by_version)
    return {
        "versions": {v: sorted(ids) for v, ids in by_version.items()},
        "attributable": len(versions) == 1 and is_clean(versions[0]),
    }


def analyse(experiment: str, records: List[RunRecord],
            total_targets: int) -> dict:
    """The experiment's own question, answered from its records."""
    report: dict = {
        "experiment": experiment,
        "records": len(records),
        "unreportable": [r.result.run_id for r in unreportable(records)],
        "unreportable_reasons": {r.result.run_id: r.unreportable_reasons
                                 for r in unreportable(records)},
        "code_versions": code_version_summary(records),
    }

    if experiment == "E3":
        report["determinism"] = determinism_comparison(records)
        lockstep = report["determinism"].get(LOCKSTEP, {})
        report["claim_holds"] = bool(lockstep.get("deterministic"))
        report["claim"] = ("lockstep runs share one determinism signature")
    elif experiment == "E2":
        report["schedulers"] = compare_schedulers(records, total_targets)
        report["claim_holds"] = {"kinematic", "dynamics_aware"} <= set(
            report["schedulers"])
        report["claim"] = "both schedulers ran over the same target set"
    elif experiment == "E1":
        forced = [r for r in records if r.provenance.attitude_provider]
        report["mispaired_runs"] = [r.result.run_id for r in forced]
        report["claim_holds"] = len(forced) > 0 and len(records) > len(forced)
        report["claim"] = ("a control and a mis-paired treatment both ran; "
                           "compare their capture rates")
        report["comparison"] = convention_comparison(records, total_targets)
    elif experiment == "E6":
        breakdown = latency_breakdown(records)
        report["latency"] = breakdown
        report["claim_holds"] = breakdown["runs"] > 0
        report["claim"] = ("measured sense/actuate/barrier-wait breakdown "
                           "from reportable runs only")
    elif experiment == "E5":
        faulted = [r for r in records if r.provenance.fault]
        safe = [r for r in faulted if r.result.entered_safe_mode]
        report["faulted_runs"] = len(faulted)
        report["safe_mode_runs"] = len(safe)
        report["did_not_trip"] = [r.result.run_id for r in faulted
                                  if not r.result.entered_safe_mode]
        # A saturation fault must trip the momentum guard. A faulted run
        # that stayed nominal means the fault did not reach the twin --
        # which looks like a clean run rather than a broken experiment,
        # so it is called out rather than averaged over.
        report["claim_holds"] = bool(faulted) and not report["did_not_trip"]
        report["claim"] = ("every seeded fault reached the twin and "
                           "tripped SAFE; use the replay verb for "
                           "reproduction")
    return report


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="run_acav_campaign",
        description="Plan, run and report the ACAV experiment campaigns.")
    parser.add_argument("verb", choices=("plan", "run", "report"))
    parser.add_argument("--experiment", required=True, choices=EXPERIMENTS)
    parser.add_argument("--output-dir", required=True, metavar="DIR")
    parser.add_argument("--scenario", default="", metavar="PATH",
                        help="scenario YAML; defaults to the installed one")
    parser.add_argument("--backend", default="com",
                        help="backend the runs will use. Anything other "
                             "than com/engine/connect produces results "
                             "that may not appear in the paper.")
    parser.add_argument("--repeats", type=int, default=None,
                        help="E1: repeats of every arm (default 1); E3: runs "
                             "per execution mode (default 20); E6: runs "
                             "(default 20)")
    parser.add_argument("--fault-seeds", type=int, default=0,
                        metavar="K",
                        help="E3: seeded wheel-fault cells beside the "
                             "no-fault cell (seeds 0..K-1; default 0)")
    parser.add_argument("--e1-types", default=",".join(E1_MISMATCH_TYPES),
                        metavar="T1,T2,...",
                        help="E1: comma-separated mismatch types (default "
                             "all six: " + ", ".join(E1_MISMATCH_TYPES) + ")")
    parser.add_argument("--seeds", type=int, default=100,
                        help="E5: Monte Carlo seeds (default 100)")
    parser.add_argument("--no-resume", action="store_true",
                        help="re-run everything, including completed runs")
    parser.add_argument("--resume-across-code-versions", action="store_true",
                        help="resume over completed runs that other code "
                             "made (refused by default)")
    parser.add_argument("--allow-dirty", action="store_true",
                        help="run from code that is not a clean commit "
                             "(refused by default)")
    parser.add_argument("--stop-on-error", action="store_true")
    parser.add_argument("--twin-launch", choices=("manual", "matlab-batch"),
                        default="manual",
                        help="run verb: manual (the default; each run asks "
                             "for the twin to be started by hand) or "
                             "matlab-batch (each run starts and stops "
                             "MATLAB itself)")
    parser.add_argument("--json", default="", metavar="PATH",
                        help="write the report as JSON as well as printing it")
    parser.add_argument("--no-figures", action="store_true",
                        help="report verb only: skip rendering a figure")
    parser.add_argument("--include-dir", action="append", default=[],
                        metavar="DIR",
                        help="report verb, E2: also read the records in DIR "
                             "(repeatable). Seeded scenarios are then "
                             "analysed as pairs by seed "
                             "(acav/paired_analysis.py)")
    parser.epilog = ("Everything after -- is the command that executes one "
                     "run; see acav.campaign_runner.CommandLauncher for the "
                     "placeholders it may contain.")
    return parser


def main(argv: Optional[list] = None) -> int:
    # Console output on a background thread: a paused console must not
    # hold the campaign (acav/console.py).
    from acav.console import configure_console_logging
    configure_console_logging(
        logging.INFO, "%(asctime)s %(levelname)-7s %(name)s: %(message)s")
    # Split on -- by hand rather than with argparse.REMAINDER, which
    # grabs everything after the first positional -- including this
    # command's own options -- and reports them as missing.
    argv = list(sys.argv[1:] if argv is None else argv)
    launch: List[str] = []
    if "--" in argv:
        separator = argv.index("--")
        argv, launch = argv[:separator], argv[separator + 1:]

    args = _parser().parse_args(argv)
    args.launch = launch

    try:
        spec = AcavSpec.load(args.scenario or None)
    except Exception as error:                  # noqa: BLE001
        LOGGER.error("could not load the scenario: %s", error)
        return 2

    scenario_path = args.scenario or ""
    try:
        e1_types = tuple(t.strip() for t in args.e1_types.split(",")
                         if t.strip())
        campaign = build_campaign(args.experiment, spec, scenario_path,
                                  args.backend, args.repeats, args.seeds,
                                  e1_types, args.fault_seeds)
    except ValueError as error:
        LOGGER.error("%s", error)
        return 2

    if args.verb == "plan":
        return _plan(campaign, args)
    if args.verb == "run":
        return _run(campaign, args)
    return _report(campaign, args, len(spec.targets))


def _plan(campaign: Campaign, args) -> int:
    run_campaign(campaign, _refuse, RunnerOptions(
        output_dir=args.output_dir, dry_run=True))
    print(f"{campaign.campaign_id}: {len(campaign)} run(s)")
    if campaign.experiment == "E1":
        types = sorted({r.mismatch for r in campaign.runs if r.mismatch},
                       key=E1_MISMATCH_TYPES.index)
        repeats = len({r.seed for r in campaign.runs})
        print(e1_formula_line(types, repeats))
    if campaign.experiment == "E3":
        _plan_e3(campaign, args)
    print(campaign.description)
    for run in campaign.runs:
        print(f"  {run.run_id}")
    print(f"\nmanifest written to "
          f"{os.path.join(args.output_dir, 'campaign.json')}")
    return 0


def _plan_e3(campaign: Campaign, args) -> None:
    """E3's plan: formula, measured times, totals, commands, resume."""
    from acav.campaign_runner import planned_commands

    repeats = (args.repeats if args.repeats is not None
               else DEFAULT_REPEATS["E3"])
    print(e3_formula_line(repeats, args.fault_seeds))
    print("measured wall time per automated run (summary.json, started_at "
          "to finished_at):")
    for line in wall_time_lines():
        print(line)
    lines, low, high, unmeasured = estimate_lines(
        "E3", e3_counts(repeats, args.fault_seeds))
    for line in lines:
        print(line)
    total = f"  E3 measured part: {low / 60:.2f}-{high / 60:.2f} h"
    if unmeasured:
        total += f", plus {unmeasured} runs not measured"
    print(total)
    for line in combined_plan_lines(repeats, args.fault_seeds):
        print(line)
    print("order: lockstep, then free-run rtf 1.0, then free-run rtf 0.5; "
          "the rate is in each run's command, nothing changes between "
          "blocks")
    print("resume: re-issue the same run command; a run whose record "
          "matches it and whose summary.json has no failed step is "
          "skipped, any other is run again and its old files renamed "
          "*.superseded-<time>")
    print("run commands (the campaign passes each as an argument list; "
          "run the campaign, not these lines, from PowerShell):")
    for run, argv in zip(campaign.runs, planned_commands(
            campaign, args.output_dir, "matlab-batch")):
        print(f"  {run.run_id}: {' '.join(argv)}")


def _run(campaign: Campaign, args) -> int:
    launch = list(args.launch)
    if not launch:
        LOGGER.error(
            "no launch command. Put it after --, e.g.\n"
            "  run_acav_campaign run --experiment E3 --output-dir out -- \\\n"
            "      python launch_one_run.py --out {output} --run {run_id}")
        return 2

    options = RunnerOptions(
        output_dir=args.output_dir,
        resume=not args.no_resume,
        stop_on_error=args.stop_on_error,
        code_version=code_version.describe(prefer_env=False),
        allow_code_version_mismatch=args.resume_across_code_versions,
        allow_dirty=args.allow_dirty,
        twin_launch=args.twin_launch)
    try:
        outcome = run_campaign(
            campaign, CommandLauncher(launch, twin_launch=args.twin_launch),
            options)
    except CampaignRefused as refused:
        LOGGER.error("%s", refused)
        return 2

    summary = outcome.summary()
    print(json.dumps(summary, indent=2))
    if not outcome.complete:
        LOGGER.error("%d of %d run(s) are missing or did not match the "
                     "configuration they were asked for",
                     len(campaign.runs) - len(outcome.records),
                     len(campaign.runs))
        return 1
    return 0


def make_figure(experiment: str, report: dict, records: List[RunRecord],
                total_targets: int):
    """The one figure ``experiment``'s question calls for, or None.

    Takes the SAME aggregation dicts ``analyse`` already computed --
    never re-derives a metric from the raw records -- so a figure and
    the JSON report next to it can never disagree about a number.
    """
    from acav import report_figures as rf

    if experiment == "E1":
        return rf.e1_convention_figure(report.get("comparison", {}))
    if experiment == "E2":
        schedulers = report.get("schedulers", {})
        return {"capture_coverage": rf.e2_capture_vs_coverage_figure(schedulers),
               "outcomes": rf.e2_outcome_breakdown_figure(schedulers)}
    if experiment == "E3":
        return rf.e3_determinism_figure(report.get("determinism", {}))
    if experiment == "E5":
        from acav.campaign import replay_matches

        # A replay's run_id is the original's with "-replay" appended --
        # see RunSpec.run_id and build_e5_replay. Pairs are found that
        # way rather than from provenance.fault alone, since matching
        # faults is exactly what a genuine reproduction is supposed to
        # produce and would pair even two UNRELATED runs that happened
        # to share a fault. A directory holding only the original
        # faulted runs (no replay verb run yet) has nothing to compare
        # and reports "no data", not an error.
        by_id = {r.result.run_id: r for r in records}
        checks = [
            replay_matches(record, by_id[f"{record.result.run_id}-replay"])
            for record in records
            if f"{record.result.run_id}-replay" in by_id
        ]
        return rf.e5_replay_figure(checks)
    if experiment == "E6":
        return rf.e6_latency_figure(report.get("latency", {}))
    return None


def _merged(per_directory: List) -> dict:
    """Several directories' {value: [run ids]} as one."""
    merged: dict = {}
    for found in per_directory:
        for key, ids in found.items():
            merged.setdefault(key, []).extend(ids)
    return {key: sorted(ids) for key, ids in merged.items()}


def _report(campaign: Campaign, args, total_targets: int) -> int:
    directories = [args.output_dir] + list(
        getattr(args, "include_dir", None) or [])
    records = []
    for directory in directories:
        found = load_results(directory)
        if len(directories) > 1:
            # Seeded E2 folders reuse run ids; name records by folder.
            folder = os.path.basename(os.path.normpath(directory))
            for record in found:
                record.result.run_id = f"{folder}/{record.result.run_id}"
        records.extend(found)
    if not records:
        LOGGER.error("no readable records in %s", ", ".join(directories))
        return 2

    diagnostic = [run for directory in directories
                  for run in diagnostic_runs(directory)]
    if diagnostic:
        LOGGER.error(
            "refusing to report: %s ran with the sensing equivalence "
            "diagnostic on, so its timings are inflated and it must not be "
            "pooled with ordinary runs. Move it to its own directory.",
            diagnostic)
        return 1

    labelled = [run for directory in directories
                for run in labelled_runs(directory)]
    if labelled:
        LOGGER.error(
            "refusing to report: diagnostic run(s) %s must not be pooled "
            "with ordinary runs. Move them to their own directory.",
            labelled)
        return 1

    factors = _merged([effective_real_time_factors(directory)
                       for directory in directories])
    # E3's matrix holds two real-time factors on purpose; its cell check
    # below refuses anything it did not plan.
    if "unknown" in factors or (len(factors) > 1
                                and args.experiment != "E3"):
        LOGGER.error(
            "refusing to report: these records do not share one real-time "
            "factor, which sets how fast free-run time follows the wall "
            "clock: %s. Move each value to its own directory.",
            "; ".join(f"{value!r}: {', '.join(ids)}"
                      for value, ids in factors.items()))
        return 1

    execution = {}
    for record in records:
        execution.setdefault(record.provenance.execution_mode, []).append(
            record.result.run_id)
    if len(execution) > 1 and args.experiment != "E3":
        LOGGER.error(
            "refusing to report: these records mix execution modes, which "
            "only E3 compares: %s. Move each mode to its own directory.",
            "; ".join(f"{mode!r}: {', '.join(sorted(ids))}"
                      for mode, ids in sorted(execution.items())))
        return 1
    if args.experiment == "E3":
        planned = {_e3_cell(run.execution_mode, run.real_time_factor,
                            None if run.fault is None
                            else run.fault.to_dict())
                   for run in campaign.runs}
        stray = sorted(r.result.run_id for r in records if _e3_cell(
            r.provenance.execution_mode, r.provenance.real_time_factor,
            r.provenance.fault) not in planned)
        if stray:
            LOGGER.error(
                "refusing to report: records outside the planned E3 cells "
                "(execution mode, real-time factor, fault): %s",
                ", ".join(stray))
            return 1

    modes = graphics_modes(records)
    labelled_modes = {mode: ids for mode, ids in modes.items()
                      if mode in ("mixed", "unverified")}
    if labelled_modes:
        LOGGER.error(
            "refusing to report: the STK graphics state of these runs is "
            "not one condition (changed during the run, or its last sample "
            "is missing), so their timing belongs to none: %s. Re-run them.",
            "; ".join(f"{mode!r}: {', '.join(ids)}"
                      for mode, ids in sorted(labelled_modes.items())))
        return 1
    if len(modes) > 1:
        LOGGER.error(
            "refusing to report: these records were made in different STK "
            "graphics modes, which change the step time and must not be "
            "pooled: %s. Move each mode to its own directory.",
            "; ".join(f"{mode or 'not recorded'!r}: {', '.join(ids)}"
                      for mode, ids in sorted(modes.items())))
        return 1

    launch_modes = _merged([twin_launch_modes(directory)
                            for directory in directories])
    if len(launch_modes) > 1:
        # The twin's code, config and results in lockstep are the same;
        # but a GUI session with the model open and a batch session load
        # the host differently, and free-run results and every timing
        # depend on that load -- the same reason graphics modes are kept
        # apart.
        LOGGER.error(
            "refusing to report: these records were made with different "
            "twin launch modes, which load the host differently: %s. Move "
            "each mode to its own directory.",
            "; ".join(f"{mode!r}: {', '.join(ids)}"
                      for mode, ids in sorted(launch_modes.items())))
        return 1

    start_modes = stk_start_modes(records)
    if len(start_modes) > 1:
        # E6 on an STK started through COM stepped at 95.7 ms against
        # about 60 ms (P10 live session); a record that does not say
        # which STK it had could be either, so it is kept apart too.
        LOGGER.error(
            "refusing to report: these records were made on STK Desktop "
            "instances started differently (operator, through COM, or not "
            "recorded), which must not be pooled: %s. Move each to its own "
            "directory.",
            "; ".join(f"{mode or 'not recorded'!r}: {', '.join(ids)}"
                      for mode, ids in sorted(start_modes.items())))
        return 1

    # Each record against its own scenario, found by digest among the
    # reference, the generated scenarios and --scenario.
    from acav.scenarios import known_scenarios, scenario_of

    known = known_scenarios([args.scenario] if args.scenario else [])
    uncovered, used = {}, {}
    for r in records:
        scenario = scenario_of(r, known)
        if scenario is None:
            named = r.provenance.scenario_sha256 or "none"
            uncovered[r.result.run_id] = [
                f"names scenario {named[:12]}..., which is not among the "
                f"known scenario files (the reference, "
                f"config/scenarios/generated, --scenario)"]
            continue
        used.setdefault(scenario.sha256, (scenario, []))[1].append(
            r.result.run_id)
        problems = target_coverage_problems(r, scenario)
        if problems:
            uncovered[r.result.run_id] = problems
    if uncovered:
        LOGGER.error(
            "refusing to report: these records do not cover exactly the "
            "targets of their scenario, so every count in them is wrong: "
            "%s. Re-run them.",
            "; ".join(f"{run_id} ({'; '.join(problems)})"
                      for run_id, problems in sorted(uncovered.items())))
        return 1
    if args.experiment == "E2" and (len(used) > 1 or len(directories) > 1):
        return _report_pairs(records, args)
    if len(used) > 1:
        LOGGER.error(
            "refusing to report: these records come from %d scenarios; "
            "report each scenario from its own directory: %s", len(used),
            "; ".join(f"{scenario.path}: {', '.join(sorted(ids))}"
                      for scenario, ids in used.values()))
        return 1
    if used:
        total_targets = len(next(iter(used.values()))[0].target_ids)

    report = analyse(args.experiment, records, total_targets)
    fail_stops: List[str] = []
    if args.experiment == "E3":
        report["lockstep_replay"] = _lockstep_replay(records)
        report["free_run_spread"] = _free_run_spread(records)
        report["claim_holds"] = bool(report["lockstep_replay"]) and all(
            cell["reproduced"] for cell in report["lockstep_replay"].values())
        report["claim"] = ("each lockstep cell's repeats share one "
                           "determinism signature (fault replay included)")
    if args.experiment == "E1":
        report["arms"], fail_stops, report["missing_runs"] = _e1_arms(
            campaign, records, directories, total_targets)
        report["claim_holds"] = (not report["missing_runs"]
                                 and report["arms"].get("control", {})
                                 .get("runs", 0) > 0)
        report["claim"] = ("every planned E1 arm ran or stopped (fail-stop, "
                           "counted from its summary.json); compare each "
                           "arm with the control")
    text = json.dumps(report, indent=2, default=str)
    print(text)
    if args.json:
        with open(args.json, "w", encoding="utf-8") as handle:
            handle.write(text)
            handle.write("\n")

    if not args.no_figures:
        _write_figures(args.experiment, report, records, total_targets,
                       args.output_dir)

    if report["unreportable"]:
        LOGGER.error(
            "%d run(s) in this campaign are NOT reportable: %s",
            len(report["unreportable"]), "; ".join(
                f"{run_id} ({', '.join(reasons)})" for run_id, reasons
                in report["unreportable_reasons"].items()))
        return 1

    if not report["code_versions"]["attributable"]:
        # Reported, not fatal: the numbers are still the numbers. But the
        # paper must cite the code that produced them, and this campaign
        # cannot be cited as one commit.
        LOGGER.warning(
            "these records do not come from one clean commit: %s. Re-run "
            "from a single committed version before citing the results.",
            sorted(report["code_versions"]["versions"]))

    missing = len(campaign.runs) - len(records) - len(fail_stops)
    if missing > 0:
        LOGGER.error("%d of %d run(s) are missing from %s",
                     missing, len(campaign.runs), args.output_dir)
        return 1

    return 0 if report.get("claim_holds", False) else 1


def _report_pairs(records: List[RunRecord], args) -> int:
    """E2 across seeded scenarios: one pair per seed (paired_analysis)."""
    from acav.paired_analysis import (analyse_pairs, paired_refusals,
                                      report_lines)

    problems = paired_refusals(records)
    if problems:
        LOGGER.error("refusing to analyse these records as pairs: %s",
                     " | ".join(problems))
        return 1
    paired = analyse_pairs(records)
    text = json.dumps({"experiment": "E2", "paired": paired}, indent=2,
                      default=str)
    print(text)
    for line in report_lines(paired):
        print(line)
    if args.json:
        with open(args.json, "w", encoding="utf-8") as handle:
            handle.write(text)
            handle.write("\n")
    return 0


def _e3_cell(mode: str, rate: Optional[float], fault: Optional[dict]):
    rate = 1.0 if rate is None and mode == "lockstep" else rate
    return (mode, rate, json.dumps(fault, sort_keys=True))


def _fault_label(fault: Optional[dict]) -> str:
    return "none" if not fault else f"fault seed {fault.get('seed')}"


def _lockstep_replay(records: List[RunRecord]) -> dict:
    """Per lockstep fault cell: do the repeats share one signature?"""
    cells: dict = {}
    for record in records:
        if record.provenance.execution_mode != "lockstep":
            continue
        cells.setdefault(_fault_label(record.provenance.fault), []).append(
            record)
    out = {}
    for label, group in sorted(cells.items()):
        signatures = {r.signature for r in group}
        out[label] = {"runs": sorted(r.result.run_id for r in group),
                      "distinct_signatures": len(signatures),
                      "reproduced": len(group) >= 2 and len(signatures) == 1}
    return out


def _free_run_spread(records: List[RunRecord]) -> dict:
    """Per (rate, fault) free-run cell: spread of existing fields."""
    cells: dict = {}
    for record in records:
        if record.provenance.execution_mode != "free_run":
            continue
        rate = record.provenance.real_time_factor
        label = (f"rtf {rate:.1f}, " if rate is not None else "rtf ?, ") \
            + _fault_label(record.provenance.fault)
        cells.setdefault(label, []).append(record)
    out = {}
    for label, group in sorted(cells.items()):
        saturations = [r.result.wheel_saturation_events for r in group]
        captured = [r.result.captured_count for r in group]
        out[label] = {
            "runs": len(group),
            "saturation_events": {"min": min(saturations),
                                  "max": max(saturations)},
            "captured": {"min": min(captured), "max": max(captured)},
            "entered_safe_mode": sum(1 for r in group
                                     if r.result.entered_safe_mode),
            "distinct_signatures": len({r.signature for r in group}),
        }
    return out


def _e1_arms(campaign: Campaign, records: List[RunRecord],
             directories: List[str], total_targets: int):
    """E1's arms: planned runs, records, fail-stops and missing runs.

    A planned run with no record but a summary.json is a fail-stop (the
    run stopped: a guard-on attitude mismatch, or the time offset's held
    twin) and is counted in its arm with the steps that failed. A planned
    run with neither is missing.
    """
    by_id = {r.result.run_id.split("/")[-1]: r for r in records}
    arms: dict = {}
    fail_stops: List[str] = []
    missing: List[str] = []
    for run in campaign.runs:
        arm = e1_arm(run.mismatch, run.mismatch_guard, run.attitude_provider)
        entry = arms.setdefault(arm, {"runs": 0, "records": [],
                                      "fail_stops": []})
        if run.mismatch in E1_EXPECTED_NULL:
            entry["role"] = (
                "expected-null control: the twin takes STK's body rate only "
                "to initialise, then integrates its own; no harm is claimed")
        record = by_id.get(run.run_id)
        if record is not None:
            entry["runs"] += 1
            entry["records"].append(record)
            continue
        summary, logs = None, ""
        for directory in directories:
            logs = os.path.join(directory, f"{run.run_id}.logs")
            summary = _read_summary(os.path.join(logs, "summary.json"))
            if summary is not None:
                break
        if summary is None:
            missing.append(run.run_id)
            continue
        fail_stops.append(run.run_id)
        stop = {"run_id": run.run_id,
                "failed_steps": [s["step"] for s in summary.get("steps", [])
                                 if not s.get("ok")]}
        if run.mismatch == "time_offset":
            stop["cause"] = _time_offset_cause(
                summary, logs, _control_ack_s(campaign, directories))
        entry["fail_stops"].append(stop)
    for entry in arms.values():
        group = entry.pop("records")
        entry["capture_rate"] = _mean_of(capture_rate(r.result)
                                         for r in group)
        entry["coverage_rate"] = _mean_of(
            coverage_rate(r.result, total_targets) for r in group)
        entry["outcomes"] = _sum_counts(outcome_breakdown(r.result)
                                        for r in group)
    return arms, fail_stops, missing


def _control_ack_s(campaign: Campaign, directories: List[str]
                   ) -> Optional[float]:
    """The control arm's recorded twin-ack wait (summary.json), or None."""
    for run in campaign.runs:
        if run.mismatch or run.attitude_provider:
            continue
        for directory in directories:
            summary = _read_summary(os.path.join(
                directory, f"{run.run_id}.logs", "summary.json"))
            wait = (summary or {}).get("twin_ack_wait") or {}
            if wait.get("stopped") == "acked" and wait.get("waited_s") \
                    is not None:
                return float(wait["waited_s"])
    return None


def _time_offset_cause(summary: dict, logs: str,
                       control_ack: Optional[float]) -> str:
    """Whether a time_offset fail-stop's own records show the twin alive
    and never ready; a slow MATLAB start also ends in a timeout.

    Read only from what the run recorded: summary.json (twin_ack_wait,
    matlab_session, the bridge_connected step) and topics_summary.json.
    """
    wait = summary.get("twin_ack_wait") or {}
    timeout = wait.get("timeout_s")
    session = summary.get("matlab_session") or {}
    steps = {s.get("step"): s.get("ok") for s in summary.get("steps", [])}
    topics = _read_summary(os.path.join(logs, "topics_summary.json"))
    counts = (topics or {}).get("received_messages") or {}
    attitude = sum(n for topic, n in counts.items()
                   if topic.startswith("/stk/") and topic.endswith("/attitude"))
    checks = [
        ("MATLAB alive at timeout", wait.get("stopped") == "timeout"
         and wait.get("exit_code") is None),
        ("simulation running", bool(session.get("running_at"))),
        ("bridge connected", steps.get("bridge_connected") is True),
        ("topics_summary.json", topics is not None),
        ("no twin status", topics is not None
         and counts.get("/acav/twin_status", 0) == 0),
        ("no twin command", topics is not None
         and counts.get("/stk/cmd/attitude", 0) == 0),
        ("bridge published attitude", attitude > 0),
        ("control acked within the bound", control_ack is not None
         and timeout is not None and control_ack < float(timeout)),
    ]
    control = ("control arm ack time not recorded" if control_ack is None
               else f"control arm acked in {control_ack:.1f} s")
    missing = [name for name, ok in checks if not ok]
    if missing:
        return (f"timeout; cause not independently confirmed ({control}); "
                f"missing: {', '.join(missing)}")
    return (f"confirmed: MATLAB alive at the {float(timeout):.1f} s timeout "
            f"and its simulation running, 0 twin statuses and 0 attitude "
            f"commands while the bridge published {attitude} attitude "
            f"samples; {control}")


def _read_summary(path: str) -> Optional[dict]:
    try:
        with open(path, encoding="utf-8") as handle:
            return json.load(handle)
    except (OSError, ValueError):
        return None


def _mean_of(values) -> float:
    numbers = [v for v in values if not math.isnan(v)]
    return float("nan") if not numbers else sum(numbers) / len(numbers)


def _sum_counts(counts) -> dict:
    total: dict = {}
    for count in counts:
        for key, value in count.items():
            total[key] = total.get(key, 0) + value
    return total


def _write_figures(experiment: str, report: dict, records: List[RunRecord],
                   total_targets: int, output_dir: str) -> None:
    """Render this experiment's figure(s), or say clearly why not.

    Never fatal to the report itself: a colleague running this to see
    the JSON should not lose that because a plotting library is not
    installed on their machine, or because matplotlib itself raised on
    a genuinely malformed input. Either is logged and the verb's own
    exit code -- which is about whether the experiment's CLAIM held --
    is unaffected.
    """
    try:
        from acav import report_figures as rf
    except ImportError:
        LOGGER.warning(
            "matplotlib is not installed, so no figure was rendered. "
            "pip install 'acav[figures]' (or plain matplotlib) to get "
            "one; the JSON report above is unaffected.")
        return

    try:
        figure = make_figure(experiment, report, records, total_targets)
    except Exception as error:              # noqa: BLE001
        LOGGER.warning("could not render a figure for %s: %s",
                       experiment, error)
        return
    if figure is None:
        return

    figures_dir = os.path.join(output_dir, "figures")
    if isinstance(figure, dict):
        named = {f"{experiment}_{name}": fig for name, fig in figure.items()}
    else:
        named = {experiment: figure}
    written = rf.save_all(named, figures_dir)
    LOGGER.info("wrote %d figure file(s) to %s", len(written), figures_dir)


def _refuse(run, campaign, path):
    raise AssertionError("a dry run must not launch anything")


if __name__ == "__main__":
    sys.exit(main())
