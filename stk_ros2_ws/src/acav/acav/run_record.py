"""Run provenance and serialisation -- the C5/C6 contract.

A :class:`~acav.metrics.RunResult` says what happened. It does not say
*where it came from*, and for this study that second question decides
whether a number may be published at all: C1 section 8 commits the paper
to reporting nothing the mock backend produced. A result file that
records only outcomes puts the burden of remembering which backend ran
on whoever assembles the figures, months later, across several hundred
files. That is not a burden a person can carry reliably, so it is
carried here instead.

So every run is stored as a :class:`RunRecord`: the result, plus the
provenance needed to decide whether it counts, plus the determinism
signature E3 compares. C6 reads these and nothing else.

Three properties this module is responsible for
-----------------------------------------------
1. **The backend is observed, not assumed.** ``backend_type`` is
   whatever the bridge reported on ``BridgeStatus``, not what the
   campaign intended to launch. A run configured for COM that silently
   fell back to the mock must not be indistinguishable from a real one
   in the record, because that is precisely the confusion C1 section 8
   exists to prevent.
2. **Reportability is derived, never stored.** :attr:`RunRecord.
   reportable` is computed from the backend every time it is asked. A
   stored flag can be edited, copied from a neighbouring file, or go
   stale when a record is regenerated; a derived one cannot.
3. **The signature is verified on read.** It is written into the file
   for readability, and recomputed from the result on load. If the two
   disagree the file has been hand-edited or truncated, and this module
   raises rather than letting a corrupted run into a campaign that then
   reports a determinism result about it.

On NaN
------
``error_at_image_deg`` is legitimately NaN for a target nothing was
attempted against. ``json.dump`` writes that as a bare ``NaN`` token,
which is not valid JSON and which stricter readers -- including MATLAB's
``jsondecode`` -- reject. Floats are therefore written as ``null`` and
read back as NaN, which is valid JSON and round-trips exactly.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

from acav.metrics import (
    CaptureOutcome,
    RunResult,
    TargetResult,
    determinism_signature,
)

#: Backends whose numbers may appear in the paper. The mock is missing
#: from this set on purpose and must stay missing: it is an in-process
#: Python object with no STK in it, and C1 section 8 commits the study
#: to reporting nothing it produced.
REPORTABLE_BACKENDS = frozenset({"com", "engine", "connect"})

#: Schema version of the record format itself. Bumped when the meaning
#: of a field changes, so C6 can refuse a campaign written by an older
#: harness rather than silently misreading it.
RECORD_VERSION = 1


class RecordError(ValueError):
    """A record could not be read, or contradicts itself."""


@dataclass(frozen=True)
class Provenance:
    """Where a result came from, and under what configuration.

    Everything here is either observed from the running system or is a
    configuration input that changes what the run means. Anything that
    can be derived from the result itself belongs in
    :mod:`acav.metrics`, not here.
    """

    #: As reported by the bridge on BridgeStatus, NOT as configured.
    #: "unknown" when no status was seen, which is itself a finding: a
    #: run whose backend could not be established is not reportable.
    backend_type: str = "unknown"
    #: BridgeStatus.stk_version_string, e.g. "12.6.0".
    stk_version: str = ""
    #: "lockstep" | "free_run" | "paced" | "external" | "unknown".
    #: E3's independent variable, so it is recorded rather than inferred.
    execution_mode: str = "unknown"
    #: SHA-256 of the scenario YAML as loaded. Two runs claiming to be
    #: the same experiment while reading different scenarios is the kind
    #: of error that produces a publishable-looking contradiction.
    scenario_sha256: str = ""
    #: Controller gains in force, for E4's sweep.
    gains: Dict[str, float] = field(default_factory=dict)
    #: The injected fault for this run, if any (see acav.fault_injection).
    fault: Optional[Dict[str, Any]] = None
    #: E1's treatment: the attitude read-back provider forced for this
    #: run, empty for the validated pairing every other run uses. Added
    #: after RECORD_VERSION 1 without a bump, because its absence in an
    #: older record has exactly one correct reading -- the validated
    #: pairing -- so nothing can be misinterpreted.
    attitude_provider: str = ""
    #: The read-back provider the bridge reported using
    #: (BridgeStatus.attitude_provider), forced or validated; empty when
    #: no status carried one -- every record written before it was
    #: observed. ``attitude_provider`` above is derived from the same
    #: observation when there is one. No RECORD_VERSION bump: absence
    #: reads as "not observed" and changes nothing else in the record.
    attitude_readback_provider: str = ""
    #: The code that produced the run (see acav.code_version): a git
    #: commit, ``<commit>+dirty``, or ``unknown (...)``. Empty in records
    #: written before it was recorded -- added without a RECORD_VERSION
    #: bump for the same reason as attitude_provider: absence has one
    #: reading ("not recorded") and changes nothing else in the record.
    code_version: str = ""
    #: The twin identity the run orchestrator checked and accepted
    #: ("v1 <run-config sha256> <build fp> <runtime fp>", see
    #: acav.twin_identity); empty when not checked -- every record written
    #: before the check existed. No RECORD_VERSION bump, as above.
    twin_identity: str = ""
    #: STK's graphics state for the whole run, written by the run
    #: orchestrator from the bridge's samples: "off", "on", "mixed" or
    #: "unverified" (stk_bridge/core/graphics_state.py). It changes the
    #: step time, so timing is never pooled across values; empty in
    #: records written before it was recorded. No RECORD_VERSION bump.
    graphics_mode: str = ""
    #: The bridge's time.real_time_factor for this run, written by the
    #: run orchestrator; None in records written before it was recorded
    #: (campaign_cli.effective_real_time_factors says how those count).
    real_time_factor: Optional[float] = None
    started_utc: str = ""
    finished_utc: str = ""
    #: Free-text, e.g. which machine, which STK licence features.
    notes: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "backend_type": self.backend_type,
            "stk_version": self.stk_version,
            "execution_mode": self.execution_mode,
            "scenario_sha256": self.scenario_sha256,
            "gains": dict(self.gains),
            "fault": self.fault,
            "attitude_provider": self.attitude_provider,
            "attitude_readback_provider": self.attitude_readback_provider,
            "code_version": self.code_version,
            "twin_identity": self.twin_identity,
            "graphics_mode": self.graphics_mode,
            "real_time_factor": self.real_time_factor,
            "started_utc": self.started_utc,
            "finished_utc": self.finished_utc,
            "notes": self.notes,
        }

    @classmethod
    def from_dict(cls, raw: Dict[str, Any]) -> "Provenance":
        return cls(
            backend_type=str(raw.get("backend_type", "unknown")),
            stk_version=str(raw.get("stk_version", "")),
            execution_mode=str(raw.get("execution_mode", "unknown")),
            scenario_sha256=str(raw.get("scenario_sha256", "")),
            gains={str(k): float(v)
                   for k, v in (raw.get("gains") or {}).items()},
            fault=raw.get("fault"),
            attitude_provider=str(raw.get("attitude_provider", "") or ""),
            attitude_readback_provider=str(
                raw.get("attitude_readback_provider", "") or ""),
            code_version=str(raw.get("code_version", "") or ""),
            twin_identity=str(raw.get("twin_identity", "") or ""),
            graphics_mode=str(raw.get("graphics_mode", "") or ""),
            real_time_factor=(None if raw.get("real_time_factor") is None
                              else float(raw["real_time_factor"])),
            started_utc=str(raw.get("started_utc", "")),
            finished_utc=str(raw.get("finished_utc", "")),
            notes=str(raw.get("notes", "")),
        )


@dataclass(frozen=True)
class RunRecord:
    """One run: what happened, and where it came from."""

    result: RunResult
    provenance: Provenance = field(default_factory=Provenance)
    #: Which experiment this run belongs to, e.g. "E3". Free-form so a
    #: one-off diagnostic run is not forced to claim membership of one.
    experiment: str = ""

    @property
    def signature(self) -> str:
        """E3's comparison key. Always recomputed from the result."""
        return determinism_signature(self.result)

    @property
    def reportable(self) -> bool:
        """May this run's numbers appear in the paper?

        Derived from the observed backend and the result on every access
        rather than stored, so it cannot be inherited from a copied file
        or left behind by a regeneration. C6 asserts on this.
        """
        return not self.unreportable_reasons

    @property
    def unreportable_reasons(self) -> List[str]:
        """Why this run may not be reported; empty when it may.

        - The backend is not a real STK one (C1 section 8).
        - A lockstep run has a target with no twin telemetry in its hold
          window: the barrier makes the twin report every step, so the
          harness failed. (In free-run that is a coupling outcome.)
        - A target's access answer never arrived, or the written attitude
          never reached its image epoch, in any mode.
        """
        reasons: List[str] = []
        backend = self.provenance.backend_type
        if backend not in REPORTABLE_BACKENDS:
            reasons.append(f"backend {backend!r} is not a real STK backend")
        silent = [t.target_id for t in self.result.targets
                  if t.outcome is CaptureOutcome.NO_TELEMETRY]
        if silent and self.provenance.execution_mode == "lockstep":
            reasons.append(
                f"lockstep run with no twin telemetry in the hold window of "
                f"{', '.join(silent)}: the barrier guarantees a sample every "
                f"step, so the harness failed")
        unanswered = [t.target_id for t in self.result.targets
                      if t.outcome is CaptureOutcome.NO_ACCESS_ANSWER]
        if unanswered:
            reasons.append(
                f"no access answer for {', '.join(unanswered)}")
        unverified = [t.target_id for t in self.result.targets
                      if t.outcome is CaptureOutcome.ACCESS_UNVERIFIED]
        if unverified:
            reasons.append(
                f"access never computed over a written attitude for "
                f"{', '.join(unverified)}")
        return reasons

    def to_dict(self) -> Dict[str, Any]:
        return {
            "record_version": RECORD_VERSION,
            "experiment": self.experiment,
            "provenance": self.provenance.to_dict(),
            "reportable": self.reportable,
            "unreportable_reasons": self.unreportable_reasons,
            "determinism_signature": self.signature,
            "result": _result_to_dict(self.result),
        }

    @classmethod
    def from_dict(cls, raw: Dict[str, Any]) -> "RunRecord":
        version = raw.get("record_version")
        if version != RECORD_VERSION:
            raise RecordError(
                f"record version {version!r}, this harness writes "
                f"{RECORD_VERSION}. Re-run the campaign rather than "
                f"reading it with a schema that may mean something else.")

        record = cls(
            result=_result_from_dict(raw.get("result") or {}),
            provenance=Provenance.from_dict(raw.get("provenance") or {}),
            experiment=str(raw.get("experiment", "")))

        stored = raw.get("determinism_signature")
        if stored is not None and stored != record.signature:
            raise RecordError(
                "the stored determinism signature does not match the "
                "result in the same file. The record has been edited or "
                "truncated; a campaign that reported E3 from it would be "
                "reporting a determinism property of a file rather than "
                "of a run.")
        return record


# ---------------------------------------------------------------------------
# JSON, with NaN handled explicitly
# ---------------------------------------------------------------------------

def _num(value: float) -> Optional[float]:
    """NaN -> null. Valid JSON, and round-trips exactly."""
    return None if value is None or math.isnan(value) else float(value)


def _from_num(value: Any) -> float:
    return float("nan") if value is None else float(value)


def _result_to_dict(result: RunResult) -> Dict[str, Any]:
    return {
        "run_id": result.run_id,
        "scheduler": result.scheduler,
        "seed": result.seed,
        "attempted_count": result.attempted_count,
        "captured_count": result.captured_count,
        "peak_momentum_fraction": [_num(v)
                                   for v in result.peak_momentum_fraction],
        "wheel_saturation_events": result.wheel_saturation_events,
        "entered_safe_mode": result.entered_safe_mode,
        "wheel_energy_J": _num(result.wheel_energy_J),
        "dropped_attitude_samples": result.dropped_attitude_samples,
        "mean_step_latency_ms": _num(result.mean_step_latency_ms),
        "mean_sense_latency_ms": _num(result.mean_sense_latency_ms),
        "mean_actuate_latency_ms": _num(result.mean_actuate_latency_ms),
        "mean_barrier_wait_ms": _num(result.mean_barrier_wait_ms),
        "targets": [_target_to_dict(t) for t in result.targets],
    }


def _target_to_dict(t: TargetResult) -> Dict[str, Any]:
    entry = {
        "target_id": t.target_id,
        "attempted": t.attempted,
        "outcome": t.outcome.value,
        "image_time_s": _num(t.image_time_s),
        "error_at_image_deg": _num(t.error_at_image_deg),
        "max_error_in_hold_deg": _num(t.max_error_in_hold_deg),
        "settle_time_s": _num(t.settle_time_s),
        "slew_magnitude_deg": _num(t.slew_magnitude_deg),
        "slew_duration_s": _num(t.slew_duration_s),
    }
    # Only when there is one, so a scored target is written exactly as
    # before the field existed.
    if t.reason:
        entry["reason"] = t.reason
    for key in ("access_wait_s", "access_wait_wall_s"):
        if not math.isnan(getattr(t, key)):
            entry[key] = getattr(t, key)
    return entry


def _result_from_dict(raw: Dict[str, Any]) -> RunResult:
    momentum = [_from_num(v) for v in (raw.get("peak_momentum_fraction")
                                       or [0.0, 0.0, 0.0])]
    while len(momentum) < 3:
        momentum.append(0.0)

    targets: List[TargetResult] = []
    for entry in raw.get("targets") or []:
        targets.append(TargetResult(
            target_id=str(entry["target_id"]),
            attempted=bool(entry["attempted"]),
            outcome=CaptureOutcome(entry["outcome"]),
            image_time_s=_from_num(entry.get("image_time_s")),
            error_at_image_deg=_from_num(entry.get("error_at_image_deg")),
            max_error_in_hold_deg=_from_num(
                entry.get("max_error_in_hold_deg")),
            settle_time_s=_from_num(entry.get("settle_time_s")),
            slew_magnitude_deg=_from_num(entry.get("slew_magnitude_deg")),
            slew_duration_s=_from_num(entry.get("slew_duration_s")),
            reason=str(entry.get("reason", "") or ""),
            access_wait_s=_from_num(entry.get("access_wait_s")),
            access_wait_wall_s=_from_num(entry.get("access_wait_wall_s")),
        ))

    seed = raw.get("seed")
    return RunResult(
        run_id=str(raw.get("run_id", "")),
        scheduler=str(raw.get("scheduler", "")),
        targets=targets,
        peak_momentum_fraction=(momentum[0], momentum[1], momentum[2]),
        wheel_saturation_events=int(raw.get("wheel_saturation_events", 0)),
        entered_safe_mode=bool(raw.get("entered_safe_mode", False)),
        wheel_energy_J=_from_num(raw.get("wheel_energy_J")),
        dropped_attitude_samples=int(raw.get("dropped_attitude_samples", 0)),
        mean_step_latency_ms=_from_num(raw.get("mean_step_latency_ms")),
        mean_sense_latency_ms=_from_num(raw.get("mean_sense_latency_ms")),
        mean_actuate_latency_ms=_from_num(
            raw.get("mean_actuate_latency_ms")),
        mean_barrier_wait_ms=_from_num(raw.get("mean_barrier_wait_ms")),
        seed=None if seed is None else int(seed),
    )


def write_record(path: str, record: RunRecord) -> None:
    """Write one run to ``path``, atomically enough for a campaign.

    Written to a neighbouring temporary file and renamed, so a campaign
    interrupted mid-write leaves either the previous record or none --
    never a half-written one that reads as a valid run with missing
    targets. Campaigns resume by looking for completed records, and a
    truncated file would be indistinguishable from a finished run that
    happened to score nothing.
    """
    import os

    payload = json.dumps(record.to_dict(), indent=2, allow_nan=False)
    temporary = f"{path}.partial"
    with open(temporary, "w", encoding="utf-8") as handle:
        handle.write(payload)
        handle.write("\n")
    os.replace(temporary, path)


def read_record(path: str) -> RunRecord:
    with open(path, encoding="utf-8") as handle:
        return RunRecord.from_dict(json.load(handle))


def read_records(paths: Sequence[str]) -> List[RunRecord]:
    return [read_record(path) for path in paths]


def unreportable(records: Sequence[RunRecord]) -> List[RunRecord]:
    """Records whose numbers must not reach the paper.

    C6 calls this and refuses to plot if it returns anything. Offered as
    a list rather than a bool so the failure names the offending runs --
    "some run in this campaign used the mock" is not actionable at 3 am
    with two hundred files.
    """
    return [r for r in records if not r.reportable]


def scenario_digest(path: str) -> str:
    """SHA-256 of a scenario file, for :attr:`Provenance.scenario_sha256`.

    Hashing the bytes rather than the parsed spec on purpose: a comment
    change is a change to the document under review even when it does
    not alter a value, and the point of the digest is to answer "were
    these runs produced from the same file?" rather than "would they
    have behaved the same?".
    """
    import hashlib

    with open(path, "rb") as handle:
        return hashlib.sha256(handle.read()).hexdigest()


@dataclass(frozen=True)
class ScenarioTargets:
    """The target set a record must cover, from one scenario file."""

    path: str
    sha256: str
    target_ids: Tuple[str, ...]


def scenario_targets(path: str = "") -> ScenarioTargets:
    """The targets of the scenario at ``path`` (default: the installed one).

    Read from the file, never hard-coded, and paired with its digest so a
    record can be checked against the scenario it names.
    """
    from acav.spec import AcavSpec

    located = AcavSpec.locate(path or None)
    spec = AcavSpec.load(located)
    return ScenarioTargets(path=located, sha256=scenario_digest(located),
                           target_ids=tuple(t.id for t in spec.targets))


def target_coverage_problems(record: RunRecord,
                             scenario: ScenarioTargets) -> List[str]:
    """Why ``record.result.targets`` is not exactly ``scenario``'s set.

    Empty when it is. A record from another scenario, or naming none,
    cannot be checked against this one and is refused as such: its
    target set cannot be established.
    """
    named = record.provenance.scenario_sha256
    if not named:
        return ["names no scenario (scenario_sha256 is empty), so its "
                "target set cannot be established"]
    if named != scenario.sha256:
        return [f"was made from another scenario (sha256 {named[:12]}..., "
                f"not {scenario.sha256[:12]}... of {scenario.path})"]
    ids = [t.target_id for t in record.result.targets]
    expected = set(scenario.target_ids)
    problems = []
    missing = [i for i in scenario.target_ids if i not in ids]
    if missing:
        problems.append(f"targets do not cover the scenario: missing "
                        f"{', '.join(missing)}")
    extra = sorted({i for i in ids if i not in expected})
    if extra:
        problems.append(f"targets do not match the scenario: extra "
                        f"{', '.join(extra)}")
    duplicate = sorted({i for i in ids if ids.count(i) > 1})
    if duplicate:
        problems.append(f"targets do not match the scenario: duplicate "
                        f"{', '.join(duplicate)}")
    return problems
