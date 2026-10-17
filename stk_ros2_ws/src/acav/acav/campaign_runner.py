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
* **Never resuming over records made by other code.** When the caller
  says which code is running (``RunnerOptions.code_version``), a record
  that would be resumed over must come from the same code; otherwise the
  campaign is refused before anything runs, unless the operator
  explicitly allows the mixture.
"""

from __future__ import annotations

import logging
import os
import subprocess
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence

from acav.campaign import Campaign, RunSpec
from acav.run_record import (RecordError, RunRecord, ScenarioTargets,
                              read_record, scenario_targets,
                              target_coverage_problems)

LOGGER = logging.getLogger(__name__)


class CampaignRefused(Exception):
    """The campaign was not started; the message says why and what to do."""


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
    #: The code running this campaign (acav.code_version.describe). When
    #: set, resuming over a record from other code is refused; None keeps
    #: the guard off for programmatic callers.
    code_version: Optional[str] = None
    #: Resume over records from other code anyway (logged per run).
    allow_code_version_mismatch: bool = False
    #: Run even though code_version is not a clean commit (logged).
    allow_dirty: bool = False
    #: Resolves a commit to its git tree; see code_version.same_code.
    tree_of: Optional[Callable[[str], Optional[str]]] = None
    #: "manual" or "matlab-batch" (deploy_run_cli --twin-launch). With
    #: matlab-batch, twin processes the previous run left running are
    #: killed before the next run, and recorded in twin_cleanup.jsonl.
    twin_launch: str = "manual"
    #: (pids) -> [(pid, image name)]; twin_launcher.processes_of.
    find_processes: Optional[Callable] = None
    #: pid -> None; twin_launcher.kill_process_tree.
    kill_tree: Optional[Callable[[int], None]] = None


def run_campaign(campaign: Campaign, launcher: Launcher,
                 options: RunnerOptions) -> CampaignOutcome:
    """Execute ``campaign``, one run at a time, and collect the records."""
    os.makedirs(options.output_dir, exist_ok=True)
    write_manifest(campaign, options.output_dir, options.twin_launch)

    if not options.dry_run:
        _refuse_unclean_code(options)
    if not options.dry_run and options.resume:
        _refuse_resume_across_code(campaign, options)

    outcome = CampaignOutcome(campaign=campaign)
    scenario = None if options.dry_run else scenario_targets(
        campaign.scenario_path)
    previous = None          # the last run launched by this invocation
    for run in campaign.runs:
        path = os.path.join(options.output_dir, run.output_name)

        if options.dry_run:
            outcome.outcomes.append(RunOutcome(
                run.run_id, "skipped", path, detail="dry run"))
            continue

        if options.resume:
            existing = _load_if_usable(run, path, scenario)
            if existing is not None:
                LOGGER.info("resume: %s already complete", run.run_id)
                outcome.outcomes.append(RunOutcome(
                    run.run_id, "skipped", path, record=existing,
                    detail="already on disk"))
                continue

        try:
            _supersede_previous_attempt(path)
        except OSError as error:
            LOGGER.error("not launching %s: %s", run.run_id, error)
            outcome.outcomes.append(RunOutcome(
                run.run_id, "failed", path,
                detail=f"could not move the previous attempt aside: "
                       f"{error}"))
        else:
            if options.twin_launch == "matlab-batch" and previous:
                _clean_up_twin(previous, options)
            outcome.outcomes.append(_execute(run, campaign, launcher, path,
                                             scenario))
            previous = (run.run_id, os.path.splitext(path)[0] + ".logs")
        if options.stop_on_error and outcome.outcomes[-1].status == "failed":
            LOGGER.error("stopping: %s failed and stop_on_error is set",
                         run.run_id)
            break

    return outcome


def _clean_up_twin(previous, options: RunnerOptions) -> None:
    """Kill twin processes the previous run left running, and record it.

    Only processes that run recorded (its launcher's and MATLAB's PIDs)
    or that those started, and only MATLAB / libmwros2server images:
    never another MATLAB instance."""
    import json

    from acav import twin_launcher

    run_id, log_dir = previous
    leftovers = twin_launcher.leftover_twin_processes(
        log_dir, options.find_processes or twin_launcher.processes_of)
    if not leftovers:
        return
    kill = options.kill_tree or twin_launcher.kill_process_tree
    for pid, _name in leftovers:
        kill(pid)
    LOGGER.warning("%s left twin processes running; killed %s", run_id,
                   leftovers)
    with open(os.path.join(options.output_dir, "twin_cleanup.jsonl"), "a",
              encoding="utf-8") as handle:
        handle.write(json.dumps({
            "after_run": run_id,
            "pids": sorted(twin_launcher.session_pids(log_dir)),
            "killed": [[pid, name] for pid, name in leftovers]}) + "\n")


def _failed_steps(output_path: str) -> List[str]:
    """Steps the run's summary.json records as failed (none if absent)."""
    import json

    summary = os.path.splitext(output_path)[0] + ".logs"
    try:
        with open(os.path.join(summary, "summary.json"),
                  encoding="utf-8") as handle:
            steps = json.load(handle).get("steps") or []
    except (OSError, ValueError, AttributeError):
        return []
    return [str(s.get("step")) for s in steps
            if isinstance(s, dict) and s.get("ok") is False]


def _supersede_previous_attempt(output_path: str) -> None:
    """Move a previous attempt's record, run config and logs aside.

    Left in place, an old record would be read as the new attempt's
    result if the launcher exited cleanly without writing one. The files
    are renamed, never deleted, to ``<name>.superseded-<time>[-<n>]``:
    names that neither ``report`` (``*.json``) nor the diagnostic scan
    (``*.logs``) picks up.
    """
    import time

    base = os.path.splitext(output_path)[0]
    previous = [p for p in (output_path, run_config_path(output_path),
                            f"{base}.logs")
                if os.path.exists(p)]
    if not previous:
        return
    suffix = f".superseded-{time.strftime('%Y%m%dT%H%M%S')}"
    attempt = 1
    while any(os.path.exists(p + suffix) for p in previous):
        attempt += 1
        suffix = f".superseded-{time.strftime('%Y%m%dT%H%M%S')}-{attempt}"
    for path in previous:
        os.replace(path, path + suffix)
        LOGGER.info("previous attempt kept as %s", path + suffix)


def _refuse_unclean_code(options: RunnerOptions) -> None:
    """Refuse to start from modified or unidentifiable code.

    Every record of such a campaign would carry that version, and a
    campaign that cannot be attributed to one commit cannot be cited.
    """
    from acav.code_version import is_clean

    current = options.code_version
    if current is None or is_clean(current):
        return
    if options.allow_dirty:
        LOGGER.warning("running from code that is not a clean commit: %s",
                       current)
        return
    raise CampaignRefused(
        f"the code running this campaign is not a clean commit "
        f"({current}), so no record it produces could be cited. Commit or "
        f"stash the changes, or pass --allow-dirty to run anyway.")


def _refuse_resume_across_code(campaign: Campaign,
                               options: RunnerOptions) -> None:
    """Refuse before anything runs if resuming would mix code versions.

    Only records that would actually be resumed over count: a record that
    is unreadable or does not match its run is re-run anyway.
    """
    from acav import code_version

    current = options.code_version
    if current is None:
        return
    tree_of = options.tree_of or code_version.tree_of
    conflicts = []
    for run in campaign.runs:
        record = _load_if_usable(
            run, os.path.join(options.output_dir, run.output_name))
        if record is None:
            continue
        recorded = record.provenance.code_version
        if not code_version.same_code(recorded, current, tree_of=tree_of):
            conflicts.append((run.run_id, recorded or "not recorded"))
    if not conflicts:
        return
    if options.allow_code_version_mismatch:
        for run_id, recorded in conflicts:
            LOGGER.warning("resuming over a record from other code: %s "
                           "(recorded %s, running %s)",
                           run_id, recorded, current)
        return
    listing = "\n".join(f"  {run_id}: {recorded}"
                        for run_id, recorded in conflicts)
    raise CampaignRefused(
        f"{len(conflicts)} completed run(s) in {options.output_dir} were "
        f"made by other code than the code running now ({current}):\n"
        f"{listing}\n"
        f"Resuming would mix code versions in one campaign. Use a new "
        f"output directory, re-run everything with --no-resume, or pass "
        f"--resume-across-code-versions to keep them anyway.")


def run_config_path(output_path: str) -> str:
    """Where a run's gain/fault overrides live, given its output path."""
    return f"{os.path.splitext(output_path)[0]}.runconfig.json"


def e1_guard_ablated(experiment: str, attitude_provider: str,
                     mismatch_guard: str) -> bool:
    """The twin's attitude guard is ablated only for an E1 run with a
    forced read-back provider and no explicit guard-on (+adcs/config.m
    accepts the ablation only then)."""
    return (experiment == "E1" and bool(attitude_provider)
            and mismatch_guard != "on")


def run_config_payload_for(run: RunSpec) -> dict:
    """The run configuration write_run_config writes for ``run``."""
    return {
        "run_id": run.run_id,
        "gains": dict(run.gains),
        "fault": None if run.fault is None else run.fault.to_dict(),
        "experiment": run.experiment,
        "attitude_provider": run.attitude_provider,
        # E1's mismatch type; read by no one in the twin (+adcs/config.m
        # reads only the keys it knows), kept so the file says what ran.
        "e1_mismatch": run.mismatch,
        "code_version": _code_version(),
        # Selects the twin's integration (+adcs/config.m).
        "execution_mode": run.execution_mode,
        # Explicit guard ablation, never enabled by an ordinary run.
        "attitude_mismatch_policy": (
            "e1_readback_ablation" if e1_guard_ablated(
                run.experiment, run.attitude_provider, run.mismatch_guard)
            else "strict"),
    }


def write_run_config(run: RunSpec, output_path: str) -> str:
    """Write the per-run gain and fault overrides beside the result.

    These are the two things a campaign varies about the *twin* rather
    than about the scenario -- per-run gains and E5's fault -- and the twin
    is MATLAB, so they have to cross a process boundary. A file beats
    stuffing JSON through a shell argument: it survives quoting, it is
    inspectable afterwards next to the record it produced, and the
    operator's launch script only has to pass a path along.

    Field names are the run record's, so one spelling travels from the
    campaign through the twin and back into the record. Two spellings
    for one quantity is how a gain sweep ends up flat.
    """
    import json

    path = run_config_path(output_path)
    payload = run_config_payload_for(run)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)
        handle.write("\n")
    return path


def _execute(run: RunSpec, campaign: Campaign, launcher: Launcher,
             path: str, scenario: Optional[ScenarioTargets] = None
             ) -> RunOutcome:
    LOGGER.info("running %s", run.run_id)
    write_run_config(run, path)
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

    mismatches = check_record(run, record, scenario)
    return RunOutcome(run.run_id, "completed", path, record=record,
                      mismatches=mismatches,
                      detail="" if not mismatches
                             else "record does not match the requested run")


def _load_if_usable(run: RunSpec, path: str,
                    scenario: Optional[ScenarioTargets] = None
                    ) -> Optional[RunRecord]:
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
    failed = _failed_steps(path)
    if failed:
        # A failed run is re-run, whatever failed -- a MATLAB session that
        # exited early or stopped uncleanly among them; its bundle is
        # kept as .superseded-*.
        LOGGER.warning("not resuming over %s: its run recorded failed "
                       "step(s) %s; it will be re-run", path,
                       ", ".join(failed))
        return None
    if check_record(run, record, scenario):
        LOGGER.warning("ignoring %s: it does not match the requested run",
                       path)
        return None
    return record


def check_record(run: RunSpec, record: RunRecord,
                 scenario: Optional[ScenarioTargets] = None) -> List[str]:
    """Ways ``record`` disagrees with the run that was requested.

    Empty means the record describes the run asked for. Every check here
    is one that would otherwise corrupt an experiment silently: a wrong
    seed or fault breaks E5's replay claim, a wrong scheduler breaks
    E2's comparison, and wrong gains break any run that sets them. A record whose
    targets are not exactly its scenario's (``scenario``, by default the
    installed one) has every count short or padded.

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

    if provenance.attitude_provider != run.attitude_provider:
        problems.append(
            f"attitude_provider {provenance.attitude_provider!r}, expected "
            f"{run.attitude_provider!r}")
    from acav.campaign import e1_arm
    recorded_arm = e1_arm(provenance.mismatch, provenance.mismatch_guard,
                          provenance.attitude_provider)
    expected_arm = e1_arm(run.mismatch, run.mismatch_guard,
                          run.attitude_provider)
    if recorded_arm != expected_arm:
        problems.append(f"E1 arm {recorded_arm!r}, expected "
                        f"{expected_arm!r}")
    if run.real_time_factor is not None and (
            provenance.real_time_factor is None
            or abs(provenance.real_time_factor - run.real_time_factor)
            > 1e-9):
        problems.append(
            f"real_time_factor {provenance.real_time_factor!r}, expected "
            f"{run.real_time_factor!r}")
    if run.experiment == "E1" and not provenance.attitude_readback_provider:
        # E1's comparison is meaningless unless the bridge said which
        # provider it read through; a requested one proves nothing.
        problems.append(
            "attitude read-back provider not observed from the bridge "
            "(BridgeStatus.attitude_provider); cannot confirm which arm ran")

    expected_fault = None if run.fault is None else run.fault.to_dict()
    if provenance.fault != expected_fault:
        problems.append(
            f"fault {provenance.fault!r}, expected {expected_fault!r}")

    for key, value in run.gains.items():
        recorded = provenance.gains.get(key)
        if recorded is None or abs(recorded - value) > 1e-12:
            problems.append(
                f"gain {key} {recorded!r}, expected {value!r}")

    problems.extend(target_coverage_problems(
        record, scenario if scenario is not None else scenario_targets()))
    return problems


def _code_version() -> str:
    from acav.code_version import describe

    return describe()


def write_manifest(campaign: Campaign, output_dir: str,
                   twin_launch: str = "manual") -> str:
    """Write the campaign definition next to its results.

    So a directory of records is self-describing months later: what was
    asked for, in what order, with which seeds and faults. Reconstructing
    that from the filenames afterwards is possible and is exactly the
    kind of reconstruction that produces a confident wrong answer.
    """
    import json

    path = os.path.join(output_dir, "campaign.json")
    manifest = campaign.to_dict()
    manifest["code_version"] = _code_version()
    if twin_launch != "manual":
        manifest["twin_launch"] = twin_launch
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(manifest, handle, indent=2)
        handle.write("\n")
    return path


#: The run command every campaign in docs/08_deploy_runbook.md uses, as
#: an argument list (the campaign substitutes the placeholders in Python;
#: no shell, so a JSON value or a path with spaces arrives intact).
STANDARD_RUN_TEMPLATE = (
    "python", "src\\acav\\scripts\\run_acav_run.py",
    "--run-id", "{run_id}", "--experiment", "{experiment}",
    "--scheduler", "{scheduler}", "--execution-mode", "{execution_mode}",
    "--seed", "{seed}", "--output", "{output}", "--scenario", "{scenario}",
    "--backend", "{backend}", "--attitude-provider", "{attitude_provider}",
    "--run-config", "{run_config}", "--gains-json", "{gains_json}",
    "--fault-json", "{fault_json}", "--mismatch", "{mismatch}",
    "--mismatch-guard", "{mismatch_guard}",
    "--real-time-factor", "{real_time_factor}",
    "--bridge-config", "src\\acav\\config\\bridge_params_acav.yaml")


def planned_commands(campaign: Campaign, output_dir: str,
                     twin_launch: str = "matlab-batch") -> List[List[str]]:
    """Every run's argv, as CommandLauncher with STANDARD_RUN_TEMPLATE
    would run it; nothing is executed."""
    commands: List[List[str]] = []
    launcher = CommandLauncher(STANDARD_RUN_TEMPLATE, twin_launch=twin_launch,
                               record_only=commands)
    for run in campaign.runs:
        launcher(run, campaign, os.path.join(output_dir, run.output_name))
    return commands


class CommandLauncher:
    """Run an external command per run, with the run's config substituted.

    The command is the operator's, because the launch topology belongs
    to the machine with STK on it. Placeholders available in the
    template: ``{run_id}``, ``{experiment}``, ``{scheduler}``,
    ``{execution_mode}``, ``{seed}``, ``{output}``, ``{scenario}``,
    ``{backend}``, ``{attitude_provider}``, ``{mismatch}``,
    ``{mismatch_guard}``, ``{real_time_factor}``, ``{run_config}``,
    ``{gains_json}``, ``{fault_json}``, ``{twin_launch}``. With a
    twin_launch other than "manual", a template without
    ``{twin_launch}`` gets ``--twin-launch <value>`` appended.

    ``{run_config}`` is the file the runner wrote with this run's gain
    and fault overrides; hand it to the twin build, which reads it via
    ``adcs.config``. The ``_json`` placeholders carry the same values
    inline for the scheduler's own parameters.

    Passed as a list of arguments rather than a shell string: a
    scenario path with a space in it is otherwise an injection waiting
    to happen, and the operator writing this template is not thinking
    about quoting at the time.
    """

    def __init__(self, argv: Sequence[str], timeout_s: Optional[float] = None,
                 cwd: str = "", env: Optional[Dict[str, str]] = None,
                 twin_launch: str = "manual",
                 record_only: Optional[List[List[str]]] = None):
        if not argv:
            raise ValueError("a launch command is required")
        #: When a list: each run's argv is appended to it instead of run
        #: (planned_commands), so a plan shows exactly what would run.
        self._record_only = record_only
        self._argv = list(argv)
        self._twin_launch = twin_launch
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
            "attitude_provider": run.attitude_provider,
            "mismatch": run.mismatch,
            "mismatch_guard": run.mismatch_guard,
            # Lockstep ignores it; 1.0 when the run sets none.
            "real_time_factor": ("1.0" if run.real_time_factor is None
                                 else repr(float(run.real_time_factor))),
            "run_config": run_config_path(output_path),
            "gains_json": json.dumps(run.gains, sort_keys=True),
            "fault_json": json.dumps(
                None if run.fault is None else run.fault.to_dict(),
                sort_keys=True),
        }
        substitutions["twin_launch"] = self._twin_launch
        argv = [arg.format(**substitutions) for arg in self._argv]
        # Automated: every run starts its own twin, unless the template
        # places the option itself. Manual commands are left as written.
        if (self._twin_launch != "manual"
                and not any("{twin_launch}" in arg for arg in self._argv)):
            argv += ["--twin-launch", self._twin_launch]

        if self._record_only is not None:
            self._record_only.append(argv)
            return
        LOGGER.debug("launching: %s", argv)
        completed = subprocess.run(argv, cwd=self._cwd, env=self._env,
                                   timeout=self._timeout_s, check=False)
        if completed.returncode != 0:
            raise RuntimeError(
                f"launch command exited {completed.returncode} for "
                f"{run.run_id}")
