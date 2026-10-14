"""Opt-in sensing equivalence diagnostic (``sensing.equivalence_diagnostic``).

Observes, never changes what is published. At each lockstep step it
compares, at the same epoch, the values the sensing loop just read with
the values two proposed optimisations would use instead
(``docs/dev/phase1_e6_impact_analysis.md`` sections 2, 4 and 9):

* (B) the quaternion columns of the 'Attitude Segment Quaternions'
  result the rate read already fetched, against 'Attitude Quaternions'
  -- no extra query;
* (C) the row for this epoch from an orbit block (two interval queries
  per block of ``block_samples`` epochs, :mod:`core.orbit_block`),
  against the instantaneous position and velocity.

One JSON line per step goes to ``equivalence_diagnostic.jsonl``; a
summary (max, mean, count, exact zeros, first epoch over each tolerance)
to ``equivalence_diagnostic_summary.json``, rewritten every block and at
the terminal epoch because the run owner may hard-terminate the bridge.
The extra queries and file writes inflate the bridge's timings: a run
with the diagnostic on is never an E6 measurement.
"""

from __future__ import annotations

import json
import logging
import math
import os
from typing import Dict, List, Optional

from stk_bridge.backends import connect_protocol as proto
from stk_bridge.backends.base import StkTime
from stk_bridge.core import conversions as conv
from stk_bridge.core import data_providers as dp
from stk_bridge.core.orbit_block import (
    OrbitBlock,
    block_epochs,
    epoch_ms,
    read_orbit_block,
)

LOGGER = logging.getLogger(__name__)

#: The agreed bar for "no change in results" (analysis, decision Q6).
TOLERANCES = {"angle_rad": 1e-12, "dr_m": 1e-6, "dv_mps": 1e-9}

JSONL_NAME = "equivalence_diagnostic.jsonl"
SUMMARY_NAME = "equivalence_diagnostic_summary.json"


def _norm(a, b) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def sign_aligned_angle(a, b) -> float:
    """Rotation angle between two orientations, radians, resolvable to 1e-12.

    ``4*atan2(|a - s*b|, |a + s*b|)`` with ``s = sign(a.b)``, so q and -q
    compare as identical. Algebraically the same angle as
    ``conversions.quaternion_angle_between`` (``2*acos(|a.b|)``), but
    that form cannot resolve below about 3e-8 rad in double precision
    (acos is flat at 1), four orders of magnitude above the 1e-12 rad
    tolerance this diagnostic must test.
    """
    s = 1.0 if sum(x * y for x, y in zip(a, b)) >= 0.0 else -1.0
    difference = math.sqrt(sum((x - s * y) ** 2 for x, y in zip(a, b)))
    total = math.sqrt(sum((x + s * y) ** 2 for x, y in zip(a, b)))
    return 4.0 * math.atan2(difference, total)


def _quaternion(raw) -> Optional[tuple]:
    """(x, y, z, w) from a stored (spec, data, mapping) result, or None."""
    if raw is None:
        return None
    spec, data, mapping = raw
    if data is None or data.row_count == 0:
        return None
    return conv.quaternion_from_stk_elements(
        *(dp.first_value(data, spec, mapping, f) for f in
          ("q1", "q2", "q3", "q4")))


class _Metric:
    def __init__(self, name: str, tolerance: Optional[float]):
        self.name, self.tolerance = name, tolerance
        self.count = 0
        self.total = 0.0
        self.maximum: Optional[float] = None
        self.exact_zero = 0
        self.first_exceeding: Optional[Dict] = None

    def add(self, value: Optional[float], step: int, t: float) -> None:
        if value is None:
            return
        magnitude = abs(value)
        self.count += 1
        self.total += magnitude
        self.maximum = (magnitude if self.maximum is None
                        else max(self.maximum, magnitude))
        if magnitude == 0:
            self.exact_zero += 1
        if (self.tolerance is not None and self.first_exceeding is None
                and magnitude > self.tolerance):
            self.first_exceeding = {"step": step, "t": t, "value": value}

    def to_dict(self) -> Dict:
        return {"count": self.count, "max": self.maximum,
                "mean": self.total / self.count if self.count else None,
                "exact_zero": self.exact_zero, "tolerance": self.tolerance,
                "first_exceeding": self.first_exceeding}


class EquivalenceDiagnostic:
    """Per-step comparison of the current sensing path with (B) and (C)."""

    def __init__(self, output_dir: str, epoch: StkTime,
                 stop_time: Optional[StkTime], step_s: float,
                 block_samples: int = 100, raw_steps: int = 20):
        if not output_dir:
            raise ValueError(
                "sensing.equivalence_diagnostic needs "
                "sensing.equivalence_diagnostic_dir (run_acav_run.py sets "
                "it to the run's log directory)")
        os.makedirs(output_dir, exist_ok=True)
        self._dir = output_dir
        self._epoch = epoch
        self._stop = stop_time
        self._step_s = step_s
        self._block_samples = block_samples
        self._raw_steps = raw_steps
        self._blocks: Dict[tuple, OrbitBlock] = {}
        self._metrics = {name: _Metric(name, TOLERANCES.get(name))
                         for name in ("angle_rad", "dr_m", "dv_mps",
                                      "time_mismatch_ms")}
        self._flags: Dict[str, int] = {}
        self._steps = 0
        self._jsonl = open(os.path.join(output_dir, JSONL_NAME), "w",
                           encoding="utf-8", buffering=1)

    # -- per step -------------------------------------------------------

    def observe(self, backend, object_path: str, frame: str, when: StkTime,
                attitude_raw, rate_raw, orbit) -> None:
        """Compare one step. Never raises into the sensing loop's caller
        for data reasons; the pipeline also guards the call."""
        t = when - self._epoch
        step = int(round(t / self._step_s))
        flags: List[str] = []
        terminal = (self._stop is not None
                    and epoch_ms(when) >= epoch_ms(self._stop))

        angle, raw = self._compare_attitude(attitude_raw, rate_raw, flags)
        dr = dv = mismatch = None
        if orbit is None:
            flags.append("no_orbit_sample")
        elif self._stop is None:
            flags.append("stop_time_unknown")
        elif terminal:
            flags.append("terminal_epoch")
        else:
            dr, dv, mismatch = self._compare_orbit(
                backend, object_path, frame, when, orbit, flags)

        line = {"step": step, "t": t, "angle_rad": angle, "dr_m": dr,
                "dv_mps": dv, "time_mismatch_ms": mismatch, "flags": flags}
        if (step <= self._raw_steps or terminal
                or "no_segment_result" in flags):
            line["raw"] = raw
        self._jsonl.write(json.dumps(line) + "\n")

        self._steps += 1
        for name, value in (("angle_rad", angle), ("dr_m", dr),
                            ("dv_mps", dv), ("time_mismatch_ms", mismatch)):
            self._metrics[name].add(value, step, t)
        for flag in flags:
            key = flag.split(":", 1)[0]
            self._flags[key] = self._flags.get(key, 0) + 1
        if terminal or self._steps % self._block_samples == 0:
            self.write_summary()

    def _compare_attitude(self, attitude_raw, rate_raw, flags):
        q_att = q_seg = None
        if attitude_raw is None:
            flags.append("no_attitude_result")
        elif attitude_raw[0] is not dp.ATTITUDE_QUATERNION:
            flags.append("attitude_provider_not_inertial:"
                         + attitude_raw[0].provider_name)
        else:
            q_att = _quaternion(attitude_raw)
        if rate_raw is None or rate_raw[0] is not dp.ATTITUDE_SEGMENT:
            flags.append("no_segment_result")
        else:
            q_seg = _quaternion(rate_raw)
            if q_seg is None:
                flags.append("no_segment_result")
        if attitude_raw is not None and q_att is None:
            raw_att = _quaternion(attitude_raw)
        else:
            raw_att = q_att
        raw = {"attitude_quaternions": list(raw_att) if raw_att else None,
               "segment_quaternions": list(q_seg) if q_seg else None}
        if q_att is None or q_seg is None:
            return None, raw
        return sign_aligned_angle(q_att, q_seg), raw

    def _compare_orbit(self, backend, object_path, frame, when, orbit,
                       flags):
        key = (object_path, frame)
        block = self._blocks.get(key)
        if block is None or block.index_of(when) is None:
            epochs = block_epochs(when, self._step_s, self._block_samples,
                                  self._stop)
            try:
                block = read_orbit_block(backend, object_path, frame,
                                         epochs, self._step_s)
            except Exception as exc:              # noqa: BLE001 - measure it
                self._blocks.pop(key, None)
                flags.append(f"block_query_failed:{exc}")
                return None, None, None
            self._blocks[key] = block
            if block.issues:
                LOGGER.warning("equivalence diagnostic: orbit block issues "
                               "at t=%.3f s: %s", when - self._epoch,
                               "; ".join(block.issues))
        if block.issues:
            flags.append("block_invalid:" + "; ".join(block.issues))
        if not block.aligned:
            return None, None, None
        index = block.index_of(when)
        dr = _norm(block.positions[index], orbit.position_m)
        dv = _norm(block.velocities[index], orbit.velocity_mps)
        returned = block.position_time_ms[index] \
            if index < len(block.position_time_ms) else None
        mismatch = None if returned is None else returned - epoch_ms(when)
        return dr, dv, mismatch

    # -- output ----------------------------------------------------------

    def summary(self) -> Dict:
        return {
            "equivalence_diagnostic": True,
            "note": ("diagnostic run: timings are inflated by the extra "
                     "queries and file writes; never an E6 measurement"),
            "tolerances": dict(TOLERANCES),
            "block_samples": self._block_samples,
            "stop_time_utcg": (proto.format_utcg(self._stop)
                               if self._stop is not None else None),
            "steps": self._steps,
            "metrics": {name: metric.to_dict()
                        for name, metric in self._metrics.items()},
            "flags": dict(self._flags),
        }

    def write_summary(self) -> None:
        path = os.path.join(self._dir, SUMMARY_NAME)
        temporary = path + ".partial"
        with open(temporary, "w", encoding="utf-8") as handle:
            json.dump(self.summary(), handle, indent=2)
            handle.write("\n")
        os.replace(temporary, path)

    def close(self) -> None:
        if self._jsonl.closed:
            return
        self.write_summary()
        self._jsonl.close()
