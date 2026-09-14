"""Executing a campaign -- the thin half of C5.

:mod:`acav.campaign` decides what runs to do and what the results mean.
This drives them, and deliberately knows as little as possible: it does
not know how to start STK, a bridge or MATLAB. Launching a run is a
callable supplied by the caller, which is what lets the whole loop --
resume, validation, aggregation -- be tested here without either.

That boundary is also honest about where the real thing runs. The launch
topology (bridge with the right backend and execution mode, the Simulink
twin, the scheduler, shutdown in the right order) belongs to the Windows
machine that has STK on it. Encoding a guess at it here would produce a
supervisor that looks finished, cannot be exercised anywhere it is
written, and fails at 2 am in the middle of a hundred-run campaign.
:class:`CommandLauncher` therefore runs whatever command the operator
gives it, per run, with the run's configuration substituted in.

What this layer is actually responsible for
-------------------------------------------
* **Resuming.** A 100-run E5 campaign that dies at run 73 must not start
  over. Output names are a function of the run's configuration (see
  ``RunSpec.run_id``), so a completed run is identifiable on disk.
* **Refusing to accept a result that does not match the run.** A record
  whose seed or fault differs from the run that was requested is a
  harness fault, and it is the specific harness fault that would make
  E5 report reproduction of something other than what it replayed. It
  is reported as a failure, not merged into the campaign.
* **Never treating a missing file as an empty result.** A launcher that
  exits cleanly without writing anything is a failure; scoring it as a
  run that captured nothing would put launch problems into the study's
  headline metric.
"""

from __future__ import annotations

import logging
import os
import subprocess
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence

from acav.campaign import Campaign, RunSpec
from acav.run_record import RecordError, RunRecord, read_record

LOGGER = logging.getLogger(__name__)

#: A launcher executes one run and is expected to leave a readable
#: record at ``output_path``. It may raise; the runner records that as a
#: failure and carries on unless asked to stop.
Launcher = Callable[[RunSpec, Campaign, str], None]


@dataclass
class RunOutcome:
    """What happened to one run of a campaign."""

    run_id: str
    status: str                     # "completed" | "skipped" | "failed"
    output_path: str = ""
    record: Optional[RunRecord] = None
    detail: str = ""
    #: Differences between what was asked for and what the record says.
    mismatches: List[str] = field(default_factory=list)

    @property
    def usable(self) -> bool:
        return self.record is not None and not self.mismatches


@dataclass
class CampaignOutcome:
    """The campaign's results, and an honest account of what is missing."""

    campaign: Campaign
    outcomes: List[RunOutcome] = field(default_factory=list)

    @property
    def records(self) -> List[RunRecord]:
        """Only the runs that completed and matched what was requested."""
        return [o.record for o in self.outcomes if o.usable and o.record]

    @property
    def failed(self) -> List[RunOutcome]:
        return [o for o in self.outcomes
                if o.status == "failed" or o.mismatches]

    @property
    def complete(self) -> bool:
        return len(self.records) == len(self.campaign.runs)

    def summary(self) -> Dict[str, object]:
        counts: Dict[str, int] = {}
        for outcome in self.outcomes:
            counts[outcome.status] = counts.get(outcome.status, 0) + 1
        return {
            "campaign_id": self.campaign.campaign_id,
            "experiment": self.campaign.experiment,
            "requested": len(self.campaign.runs),
            "usable": len(self.records),
            "by_status": counts,
            "failures": [
                {"run_id": o.run_id, "detail": o.detail,
                 "mismatches": o.mismatches}
                for o in self.failed
            ],
        }


@dataclass
class RunnerOptions:
    output_dir: str
    #: Skip runs whose output is already on disk and readable.
    resume: bool = True
    #: Stop the campaign at the first failure instead of carrying on.
    #: Off by default: one launch failure in a hundred-run overnight
    #: campaign should cost one run, not the night.
    stop_on_error: bool = False
    #: Write the manifest and report what would run, launching nothing.
    dry_run: bool = False


def run_campaign(campaign: Campaign, launcher: Launcher,
                 options: RunnerOptions) -> CampaignOutcome:
    """Execute ``campaign``, one run at a time, and collect the records."""
    os.makedirs(options.output_dir, exist_ok=True)
    write_manifest(campaign, options.output_dir)

    outcome = CampaignOutcome(campaign=campaign)
    for run in campaign.runs:
        path = os.path.join(options.output_dir, run.output_name)

        if options.dry_run:
            outcome.outcomes.append(RunOutcome(
                run.run_id, "skipped", path, detail="dry run"))
            continue

        if options.resume:
            existing = _load_if_usable(run, path)
            if existing is not None:
                LOGGER.info("resume: %s already complete", run.run_id)
                outcome.outcomes.append(RunOutcome(
                    run.run_id, "skipped", path, record=existing,
                    detail="already on disk"))
                continue

        outcome.outcomes.append(_execute(run, campaign, launcher, path))
        if options.stop_on_error and outcome.outcomes[-1].status == "failed":
            LOGGER.error("stopping: %s failed and stop_on_error is set",
                         run.run_id)
            break

    return outcome


def _execute(run: RunSpec, campaign: Campaign, launcher: Launcher,
             path: str) -> RunOutcome:
    LOGGER.info("running %s", run.run_id)
    try:
        launcher(run, campaign, path)
    except Exception as error:                      # noqa: BLE001
        LOGGER.exception("launch failed for %s", run.run_id)
        return RunOutcome(run.run_id, "failed", path,
                          detail=f"launcher raised: {error}")

    if not os.path.exists(path):
        # Deliberately a failure rather than an empty result: a launcher
        # that exits without writing has not produced a run, and scoring
        # it as one that captured nothing would put launch problems into
        # the capture rate.
        return RunOutcome(run.run_id, "failed", path,
                          detail="the launcher wrote no record")

    try:
        record = read_record(path)
    except (RecordError, ValueError, KeyError, OSError) as error:
        return RunOutcome(run.run_id, "failed", path,
                          detail=f"unreadable record: {error}")

    mismatches = check_record(run, record)
    return RunOutcome(run.run_id, "completed", path, record=record,
                      mismatches=mismatches,
                      detail="" if not mismatches
                             else "record does not match the requested run")


def _load_if_usable(run: RunSpec, path: str) -> Optional[RunRecord]:
    """An existing record, if it is readable and matches the run.

    A record on disk that does not match the run is not resumed over --
    it is left for :func:`_execute` to re-run and report, because
    treating it as done would silently keep a result from a different
    configuration in the campaign.
    """
    if not os.path.exists(path):
        return None
    try:
        record = read_record(path)
    except (RecordError, ValueError, KeyError, OSError) as error:
        LOGGER.warning("ignoring unreadable record %s: %s", path, error)
        return None
    if check_record(run, record):
        LOGGER.warning("ignoring %s: it does not match the requested run",
                       path)
        return None
    return record


def check_record(run: RunSpec, record: RunRecord) -> List[str]:
    """Ways ``record`` disagrees with the run that was requested.

    Empty means the record describes the run asked for. Every check here
    is one that would otherwise corrupt an experiment silently: a wrong
    seed or fault breaks E5's replay claim, a wrong scheduler breaks
    E2's comparison, and wrong gains break E4's surface.

    Execution mode is NOT checked here. A bridge that fell back from
    lockstep to free-run has produced a real run in the mode it reports,
    and E3 groups by the observed mode precisely so that such a run is
    counted where it belongs rather than discarded or miscounted.
    """
    problems: List[str] = []
    result, provenance = record.result, record.provenance

    if result.run_id and result.run_id != run.run_id:
        problems.append(
            f"run_id {result.run_id!r}, expected {run.run_id!r}")
    if result.seed != run.seed:
        problems.append(f"seed {result.seed!r}, expected {run.seed!r}")
    if result.scheduler and result.scheduler != run.scheduler:
        problems.append(
            f"scheduler {result.scheduler!r}, expected {run.scheduler!r}")

    expected_fault = None if run.fault is None else run.fault.to_dict()
    if provenance.fault != expected_fault:
        problems.append(
            f"fault {provenance.fault!r}, expected {expected_fault!r}")

    for key, value in run.gains.items():
        recorded = provenance.gains.get(key)
        if recorded is None or abs(recorded - value) > 1e-12:
            problems.append(
                f"gain {key} {recorded!r}, expected {value!r}")
    return problems


def write_manifest(campaign: Campaign, output_dir: str) -> str:
    """Write the campaign definition next to its results.

    So a directory of records is self-describing months later: what was
    asked for, in what order, with which seeds and faults. Reconstructing
    that from the filenames afterwards is possible and is exactly the
    kind of reconstruction that produces a confident wrong answer.
    """
    import json

    path = os.path.join(output_dir, "campaign.json")
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(campaign.to_dict(), handle, indent=2)
        handle.write("\n")
    return path


class CommandLauncher:
    """Run an external command per run, with the run's config substituted.

    The command is the operator's, because the launch topology belongs
    to the machine with STK on it. Placeholders available in the
    template: ``{run_id}``, ``{experiment}``, ``{scheduler}``,
    ``{execution_mode}``, ``{seed}``, ``{output}``, ``{scenario}``,
    ``{backend}``, ``{gains_json}``, ``{fault_json}``.

    Passed as a list of arguments rather than a shell string: a
    scenario path with a space in it is otherwise an injection waiting
    to happen, and the operator writing this template is not thinking
    about quoting at the time.
    """

    def __init__(self, argv: Sequence[str], timeout_s: Optional[float] = None,
                 cwd: str = "", env: Optional[Dict[str, str]] = None):
        if not argv:
            raise ValueError("a launch command is required")
        self._argv = list(argv)
        self._timeout_s = timeout_s
        self._cwd = cwd or None
        self._env = env

    def __call__(self, run: RunSpec, campaign: Campaign,
                 output_path: str) -> None:
        import json

        substitutions = {
            "run_id": run.run_id,
            "experiment": run.experiment,
            "scheduler": run.scheduler,
            "execution_mode": run.execution_mode,
            "seed": "" if run.seed is None else str(run.seed),
            "output": output_path,
            "scenario": campaign.scenario_path,
            "backend": campaign.backend,
            "gains_json": json.dumps(run.gains, sort_keys=True),
            "fault_json": json.dumps(
                None if run.fault is None else run.fault.to_dict(),
                sort_keys=True),
        }
        argv = [arg.format(**substitutions) for arg in self._argv]

        LOGGER.debug("launching: %s", argv)
        completed = subprocess.run(argv, cwd=self._cwd, env=self._env,
                                   timeout=self._timeout_s, check=False)
        if completed.returncode != 0:
            raise RuntimeError(
                f"launch command exited {completed.returncode} for "
                f"{run.run_id}")
