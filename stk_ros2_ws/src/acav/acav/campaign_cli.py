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
import os
import sys
from typing import List, Optional

from acav.campaign import (
    LOCKSTEP,
    Campaign,
    build_e1_convention,
    build_e2_scheduler_comparison,
    build_e6_latency,
    build_e3_determinism,
    build_e4_gain_sweep,
    build_e5_fault_campaign,
    compare_schedulers,
    convention_comparison,
    determinism_comparison,
    gain_surface,
    latency_breakdown,
)
from acav.campaign_runner import (
    CommandLauncher,
    RunnerOptions,
    run_campaign,
)
from acav.run_record import RunRecord, read_record, unreportable
from acav.spec import AcavSpec

LOGGER = logging.getLogger(__name__)

EXPERIMENTS = ("E1", "E2", "E3", "E4", "E5", "E6")


def build_campaign(experiment: str, spec: AcavSpec, scenario_path: str,
                   backend: str, repeats: int, seeds: int) -> Campaign:
    """The matrix for ``experiment``, sized from the scenario."""
    if experiment == "E1":
        return build_e1_convention(scenario_path, backend)
    if experiment == "E2":
        return build_e2_scheduler_comparison(scenario_path, backend)
    if experiment == "E3":
        return build_e3_determinism(repeats, scenario_path, backend)
    if experiment == "E4":
        return build_e4_gain_sweep(scenario_path=scenario_path,
                                   backend=backend)
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
        # campaign.json is the manifest; *.runconfig.json is the E4/E5
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
        "code_versions": code_version_summary(records),
    }

    if experiment == "E3":
        report["determinism"] = determinism_comparison(records)
        lockstep = report["determinism"].get(LOCKSTEP, {})
        report["claim_holds"] = bool(lockstep.get("deterministic"))
        report["claim"] = ("lockstep runs share one determinism signature")
    elif experiment == "E4":
        report["surface"] = gain_surface(records, total_targets)
        report["claim_holds"] = len(records) > 0
        report["claim"] = "every gain combination produced a result"
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
    parser.add_argument("--repeats", type=int, default=20,
                        help="E3: runs per execution mode (default 20)")
    parser.add_argument("--seeds", type=int, default=100,
                        help="E5: Monte Carlo seeds (default 100)")
    parser.add_argument("--no-resume", action="store_true",
                        help="re-run everything, including completed runs")
    parser.add_argument("--stop-on-error", action="store_true")
    parser.add_argument("--json", default="", metavar="PATH",
                        help="write the report as JSON as well as printing it")
    parser.add_argument("--no-figures", action="store_true",
                        help="report verb only: skip rendering a figure")
    parser.epilog = ("Everything after -- is the command that executes one "
                     "run; see acav.campaign_runner.CommandLauncher for the "
                     "placeholders it may contain.")
    return parser


def main(argv: Optional[list] = None) -> int:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s")
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
        campaign = build_campaign(args.experiment, spec, scenario_path,
                                  args.backend, args.repeats, args.seeds)
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
    print(campaign.description)
    for run in campaign.runs:
        print(f"  {run.run_id}")
    print(f"\nmanifest written to "
          f"{os.path.join(args.output_dir, 'campaign.json')}")
    return 0


def _run(campaign: Campaign, args) -> int:
    launch = list(args.launch)
    if not launch:
        LOGGER.error(
            "no launch command. Put it after --, e.g.\n"
            "  run_acav_campaign run --experiment E3 --output-dir out -- \\\n"
            "      python launch_one_run.py --out {output} --run {run_id}")
        return 2

    outcome = run_campaign(campaign, CommandLauncher(launch), RunnerOptions(
        output_dir=args.output_dir,
        resume=not args.no_resume,
        stop_on_error=args.stop_on_error))

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
    if experiment == "E4":
        return rf.e4_gain_surface_figure(report.get("surface", []))
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


def _report(campaign: Campaign, args, total_targets: int) -> int:
    records = load_results(args.output_dir)
    if not records:
        LOGGER.error("no readable records in %s", args.output_dir)
        return 2

    diagnostic = diagnostic_runs(args.output_dir)
    if diagnostic:
        LOGGER.error(
            "refusing to report: %s ran with the sensing equivalence "
            "diagnostic on, so its timings are inflated and it must not be "
            "pooled with ordinary runs. Move it to its own directory.",
            diagnostic)
        return 1

    report = analyse(args.experiment, records, total_targets)
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
            "%d run(s) in this campaign are NOT reportable -- they did not "
            "come from a real STK backend, and the case study spec commits "
            "the paper to publishing no number the mock produced: %s",
            len(report["unreportable"]), report["unreportable"])
        return 1

    if not report["code_versions"]["attributable"]:
        # Reported, not fatal: the numbers are still the numbers. But the
        # paper must cite the code that produced them, and this campaign
        # cannot be cited as one commit.
        LOGGER.warning(
            "these records do not come from one clean commit: %s. Re-run "
            "from a single committed version before citing the results.",
            sorted(report["code_versions"]["versions"]))

    missing = len(campaign.runs) - len(records)
    if missing > 0:
        LOGGER.error("%d of %d run(s) are missing from %s",
                     missing, len(campaign.runs), args.output_dir)
        return 1

    return 0 if report.get("claim_holds", False) else 1


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
