"""Offline timing and coupling analysis of a run, from its logs only.

Reads a run's record and its ``<run>.logs`` folder -- the topic
captures (cmd.txt, attitude.txt, twin_status.txt, status.txt),
bridge.log and summary.json -- without STK or ROS, and computes what
the E3 free-run pilot analysis used:

* ``control_rate_hz``: twin commands per scenario second (``window_s``);
* command-gap distribution: p50, p90, max, count and total over 0.6 s;
* ``image_ages``: for each attempted target, the image epoch minus the
  stamp of the newest twin status at or before it (the sample the twin
  had used), with that status's boresight error;
* ``n_gt1_count``/``max_n``: samples integrated over more than one step,
  from the twin's own statistics (summary.json "twin_substeps") when
  present, else from the command spacing;
* command-stamp offset: the step boundary the twin reached minus the
  stamp t + step it was sent with, counted from the first command;
* write lag: the bridge time at which a command was written (the next
  bridge sample after its source sample) minus its stamp;
* COM round-trip stalls over 300 ms once the loop is closed (status.txt
  stamped after the first command);
* STK read-back: read-backs at a written stamp, and those beyond the
  newest written sample with their angle to it;
* run conditions: execution mode, graphics mode, real-time factor,
  start-gate wait and free-run pacing;
* the twin's loop (free-run): its ready calls (calls with matched
  telemetry stamps, summary.json twin_substeps.samples) per wall second
  -- the window spans window_s / rtf wall seconds, since free-run time
  is wall time since the gate times rtf -- the share of them that were
  holds (n = 0; every fresh call publishes one command), and the
  requested polling rate from the twin identity ("v2 ... <rate>Hz");
* bridge samples consumed: distinct bridge sample stamps (attitude.txt)
  and how many the twin answered with a command stamped one step later.

Percentiles are ``sorted[n // 2]`` and ``sorted[int(0.9 * n)]``, as in the
pilot analysis. A lockstep run gives 10 Hz, age 0 and deviation 0.
"""

from __future__ import annotations

import bisect
import csv
import glob
import json
import math
import os
import re
from typing import Dict, List, Optional, Tuple

_STAMP = re.compile(r"stamp:\s*\n\s*sec:\s*(-?\d+)\s*\n\s*nanosec:\s*(\d+)")
_ORIENTATION = re.compile(
    r"orientation:\s*\n\s*x: ([-0-9.eE+]+)\s*\n\s*y: ([-0-9.eE+]+)\s*\n"
    r"\s*z: ([-0-9.eE+]+)\s*\n\s*w: ([-0-9.eE+]+)")
_TIME_SYNC = re.compile(
    r"time sync started: mode=(\w+) epoch=([0-9.]+) step=([0-9.]+)s "
    r"rtf=([0-9.]+)")
_PACING = re.compile(r"free-run pacing summary: cycles (\d+), overruns (\d+)")
GAP_S = 0.6
STALL_MS = 300.0


def _documents(path: str) -> List[str]:
    try:
        with open(path, encoding="utf-8", errors="replace") as handle:
            return handle.read().split("\n---\n")
    except OSError:
        return []


def _stamp(document: str, epoch: float) -> Optional[float]:
    """Scenario seconds. Integer seconds are subtracted first: at
    ~1.7e9 s a float sum keeps only ~2.4e-7 s, enough to move an on-grid
    stamp across a step boundary in the offset test."""
    match = _STAMP.search(document)
    if not match:
        return None
    whole = math.floor(epoch)
    return ((int(match.group(1)) - whole) + int(match.group(2)) * 1e-9
            - (epoch - whole))


def _number(document: str, key: str) -> Optional[float]:
    match = re.search(key + r": ([-0-9.eE+]+)", document)
    return float(match.group(1)) if match else None


def read_attitudes(path: str, epoch: float
                   ) -> List[Tuple[float, Tuple[float, ...]]]:
    """(scenario time, quaternion xyzw) from cmd.txt or attitude.txt."""
    rows = []
    for document in _documents(path):
        t = _stamp(document, epoch)
        match = _ORIENTATION.search(document)
        if t is not None and match:
            rows.append((t, tuple(float(v) for v in match.groups())))
    return rows


def read_values(path: str, epoch: float, key: str
                ) -> List[Tuple[float, float]]:
    rows = []
    for document in _documents(path):
        t, value = _stamp(document, epoch), _number(document, key)
        if t is not None and value is not None:
            rows.append((t, value))
    return rows


def bridge_log_facts(text: str) -> Dict[str, object]:
    facts: Dict[str, object] = {}
    match = _TIME_SYNC.search(text)
    if match:
        facts.update(mode=match.group(1), epoch=float(match.group(2)),
                     step_s=float(match.group(3)),
                     real_time_factor=float(match.group(4)))
    pacing = _PACING.findall(text)
    if pacing:
        cycles, overruns = pacing[-1]
        facts.update(pacing_cycles=int(cycles), pacing_overruns=int(overruns))
    return facts


def _angle_deg(a, b) -> float:
    dot = abs(sum(x * y for x, y in zip(a, b)))
    return math.degrees(2.0 * math.acos(min(1.0, dot)))


def _p50(values: List[float]) -> Optional[float]:
    return sorted(values)[len(values) // 2] if values else None


def _p90(values: List[float]) -> Optional[float]:
    return sorted(values)[int(0.9 * len(values))] if values else None


def analyze_run(record_path: str, window_s: float) -> Dict[str, object]:
    from acav.run_record import read_record

    record = read_record(record_path)
    logs = record_path[:-len(".json")] + ".logs"
    try:
        with open(os.path.join(logs, "bridge.log"), encoding="utf-8",
                  errors="replace") as handle:
            facts = bridge_log_facts(handle.read())
    except OSError:
        facts = {}
    try:
        with open(os.path.join(logs, "summary.json"),
                  encoding="utf-8") as handle:
            summary = json.load(handle)
    except (OSError, ValueError):
        summary = {}
    epoch = float(facts.get("epoch", 0.0))
    step = float(facts.get("step_s", 0.1))
    provenance = record.provenance

    cmd = read_attitudes(os.path.join(logs, "cmd.txt"), epoch)
    att = read_attitudes(os.path.join(logs, "attitude.txt"), epoch)
    twin = read_values(os.path.join(logs, "twin_status.txt"), epoch,
                       "boresight_error_rad")
    status = read_values(os.path.join(logs, "status.txt"), epoch,
                         "last_backend_round_trip_ms")
    cmd_t = [t for t, _ in cmd]
    att_t = [t for t, _ in att]

    gaps = [b - a for a, b in zip(cmd_t, cmd_t[1:])]
    long_gaps = [g for g in gaps if g > GAP_S]

    bridge_samples = sorted({round(t, 6) for t in att_t if t > -1e-9})
    consumed = 0
    for stamp in cmd_t:
        k = bisect.bisect_left(bridge_samples, stamp - step - 1e-6)
        if k < len(bridge_samples) and abs(
                bridge_samples[k] - (stamp - step)) < 1e-6:
            consumed += 1

    substeps = summary.get("twin_substeps")
    if substeps:
        n_source = "twin_substeps"
        n_gt1, max_n = int(substeps["events"]), int(substeps["max_n"])
    else:
        n_source = "commands"
        n_gt1 = sum(1 for g in gaps if g > 1.5 * step)
        max_n = max([1] + [int(round(g / step)) for g in gaps])

    image_ages = []
    for target in sorted(record.result.targets, key=lambda t: t.target_id):
        if not target.attempted:
            continue
        before = [(t, e) for t, e in twin
                  if t <= target.image_time_s + 1e-9]
        if before:
            t, error = before[-1]
            image_ages.append({"target": target.target_id,
                               "image_time_s": target.image_time_s,
                               "age_s": target.image_time_s - t,
                               "error_deg": math.degrees(error)})

    offsets = []
    if cmd_t:
        t0 = cmd_t[0] - step
        for stamp in cmd_t:
            reached = t0 + (math.floor((stamp - step - t0) / step + 1e-6)
                            + 1) * step
            offsets.append(reached - stamp)
    affected = [o for o in offsets if abs(o) > 1e-6]

    lags = []
    for stamp in cmd_t:
        index = bisect.bisect_right(att_t, stamp - step + 1e-6)
        if index < len(att_t):
            lags.append(att_t[index] - stamp)

    at_written, after = [], []
    for t, q in att:
        index = bisect.bisect_right(cmd_t, t + 1e-6) - 1
        if index < 0:
            continue
        deviation = _angle_deg(q, cmd[index][1])
        (at_written if abs(cmd_t[index] - t) < 1e-6 else after).append(
            deviation)

    # After the first command: the loop is closed. Earlier statuses are
    # the startup wait's (lockstep republishes its first step) or the
    # free-run gate's, with the start-up round trips.
    first = cmd_t[0] if cmd_t else 0.0
    round_trips = [ms for t, ms in status if t > first + 1e-6]
    gate = summary.get("free_run_start_gate") or {}
    pacing = summary.get("free_run_pacing") or {}
    rtf = summary.get("real_time_factor", provenance.real_time_factor)
    if rtf is None:
        rtf = facts.get("real_time_factor")
    ready_calls = int(substeps["samples"]) if substeps and         "samples" in substeps else None
    wall_span = window_s / rtf if rtf and window_s else None
    return {
        "run_id": record.result.run_id,
        "execution_mode": provenance.execution_mode,
        "graphics_mode": provenance.graphics_mode
        or summary.get("graphics_mode", ""),
        "real_time_factor": rtf,
        "gate_wait_s": gate.get("wait_s"),
        "pacing_cycles": pacing.get("cycles", facts.get("pacing_cycles")),
        "pacing_overruns": pacing.get("overruns",
                                      facts.get("pacing_overruns")),
        "captured": record.result.captured_count,
        "attempted": record.result.attempted_count,
        "commands": len(cmd_t),
        "control_rate_hz": len(cmd_t) / window_s if window_s else None,
        "gap_p50_s": _p50(gaps),
        "gap_p90_s": _p90(gaps),
        "gap_max_s": max(gaps) if gaps else None,
        "gaps_over_0_6": len(long_gaps),
        "gap_time_over_0_6_s": sum(long_gaps),
        "n_source": n_source,
        "n_gt1_count": n_gt1,
        "max_n": max_n,
        "image_ages": image_ages,
        "image_age_max_s": max((a["age_s"] for a in image_ages),
                               default=None),
        "offset_affected": len(affected),
        "offset_max_abs_s": -min(offsets) if offsets else None,
        "offset_mean_s": sum(offsets) / len(offsets) if offsets else None,
        "write_lag_behind": sum(1 for x in lags if x > 1e-6),
        "write_lag_p50_s": _p50(lags),
        "write_lag_p90_s": _p90(lags),
        "write_lag_max_s": max(lags) if lags else None,
        "com_stalls_over_300ms": sum(1 for ms in round_trips
                                     if ms > STALL_MS),
        "com_round_trip_max_ms": max(round_trips) if round_trips else None,
        "readback_at_written": len(at_written),
        "readback_at_written_median_deg": _p50(at_written),
        "readback_after_newest": len(after),
        "readback_after_median_deg": _p50(after),
        "readback_after_p90_deg": _p90(after),
        "readback_after_max_deg": max(after) if after else None,
        "twin_polling_hz_requested": _polling_hz(provenance.twin_identity),
        "twin_ready_calls": ready_calls,
        "twin_ready_calls_per_wall_s": (
            ready_calls / wall_span if ready_calls is not None and wall_span
            else None),
        "twin_fresh_calls": len(cmd_t),
        "twin_hold_share": (
            (ready_calls - len(cmd_t)) / ready_calls if ready_calls
            else None),
        "bridge_samples": len(bridge_samples),
        "bridge_samples_consumed": consumed,
        "consumed_share": (consumed / len(bridge_samples)
                           if bridge_samples else None),
    }


def _polling_hz(identity: str) -> Optional[float]:
    """The requested polling rate in a v2 twin identity, else None."""
    from acav.twin_identity import parse_identity

    try:
        return parse_identity(identity).polling_hz
    except ValueError:
        return None


def analyze_folder(folder: str, out_dir: str, window_s: float
                   ) -> List[Dict[str, object]]:
    """Analyse every record in ``folder``; write one JSON per run and a
    CSV of the scalar fields to ``out_dir``."""
    rows = []
    for path in sorted(glob.glob(os.path.join(folder, "*.json"))):
        name = os.path.basename(path)
        if name == "campaign.json" or name.endswith(".runconfig.json"):
            continue
        if not os.path.isdir(path[:-len(".json")] + ".logs"):
            continue
        rows.append(analyze_run(path, window_s))
    rows.sort(key=lambda r: r["run_id"])
    os.makedirs(out_dir, exist_ok=True)
    for row in rows:
        with open(os.path.join(out_dir, f"{row['run_id']}.coupling.json"),
                  "w", encoding="utf-8") as handle:
            json.dump(row, handle, indent=2)
            handle.write("\n")
    scalars = [k for k in (rows[0] if rows else {}) if k != "image_ages"]
    with open(os.path.join(out_dir, "coupling.csv"), "w", encoding="utf-8",
              newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=scalars,
                                extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    return rows
