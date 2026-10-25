"""Lockstep orbit reads served from audited STK interval blocks.

Each lockstep step reads position and velocity from STK. HPOP has already
propagated the orbit, so asking for the next ``samples`` epochs in two
interval queries returns the same STK rows with far fewer COM round
trips. HPOP stays the source: nothing is interpolated or propagated
here, and a row is used only for the exact millisecond STK returned it
for.

A block (``core/orbit_block.py``) is used only if

* the object's propagator is confirmed HPOP or TwoBody,
* its UTCG grid, row counts and values validate, and
* both of its ends agree with instantaneous STK reads within the audit
  tolerances (strict defaults: 1e-6 m, 1e-9 m/s).

Otherwise the block's epochs are read instantaneously, exactly as
without batching, and the fallback is logged. Blocks never contain an
epoch at or after the scenario stop time, so the terminal epoch is
always read instantaneously.

Rows are kept per (object, frame) for one backend revision
(``StkBackend.revision``): scenario or object lifecycle, propagation,
orbit writes, SetAttitude Standard / ClearData and raw Connect commands
drop them; attitude samples do not. That is valid only while no force
depends on attitude (spherical drag and SRP models do not). External
edits to the scenario in STK do not change the revision: disable
batching for runs that need them, or for an attitude-dependent force
model.
"""

from __future__ import annotations

import logging
import math
from typing import Callable, Dict, Optional, Set, Tuple

from stk_bridge.backends.base import StkTime
from stk_bridge.core.orbit_block import (
    OrbitBlock,
    OrbitBlockError,
    block_epochs,
    epoch_ms,
    read_orbit_block,
)
from stk_bridge.exceptions import QueryError

LOGGER = logging.getLogger(__name__)

DEFAULT_AUDIT_POSITION_M = 1e-6
DEFAULT_AUDIT_VELOCITY_MPS = 1e-9

#: AgEVePropagatorType names whose ephemeris batching may read.
BATCHABLE_PROPAGATORS = ("ePropagatorHPOP", "ePropagatorTwoBody")

Vector3 = Tuple[float, float, float]
#: ``read_instantaneous(when) -> (position_m, velocity_mps)``.
InstantReader = Callable[[StkTime], Tuple[Vector3, Vector3]]


def stop_time_problem(epoch: StkTime, stop: Optional[StkTime],
                      expected_window_s: Optional[float]) -> Optional[str]:
    """Why batching must stay off for this stop time, or None.

    The stop time comes from STK; when the scenario window is known it
    must agree to the millisecond.
    """
    if stop is None:
        return "the scenario stop time could not be read from STK"
    if expected_window_s is None:
        return None
    window_ms = epoch_ms(stop) - epoch_ms(epoch)
    if window_ms != int(round(expected_window_s * 1000.0)):
        return (f"STK's scenario runs {window_ms / 1000.0:.3f} s but the "
                f"scenario specification's window is "
                f"{expected_window_s:.3f} s")
    return None


def _distance(a, b) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


class OrbitBatch:
    """Serves orbit samples for lockstep epochs from audited blocks."""

    def __init__(self, samples: int, step_s: float, stop_time: StkTime,
                 audit_position_m: float = DEFAULT_AUDIT_POSITION_M,
                 audit_velocity_mps: float = DEFAULT_AUDIT_VELOCITY_MPS):
        if samples < 2:
            raise ValueError("orbit batching needs at least 2 samples")
        self.samples = samples
        self.step_s = step_s
        self.stop_time = stop_time
        self.audit_position_m = audit_position_m
        self.audit_velocity_mps = audit_velocity_mps
        #: (object, frame) -> (block, revision it was read at)
        self._blocks: Dict[tuple, Tuple[OrbitBlock, int]] = {}
        #: (object, frame) -> milliseconds a failed block covered
        self._failed: Dict[tuple, Tuple[int, Set[int]]] = {}
        #: (object, revision) -> propagator confirmed batchable
        self._eligible: Dict[tuple, bool] = {}
        self.counts = {
            "blocks_loaded": 0, "audits_passed": 0, "audits_failed": 0,
            "validation_failed": 0, "query_failed": 0, "fallback_blocks": 0,
            "invalidations_revision": 0, "epochs_cached": 0,
            "epochs_instantaneous": 0, "epochs_terminal": 0,
            "epochs_unsupported": 0}

    # -- serving -----------------------------------------------------------

    def serve(self, backend, object_path: str, frame: str, when: StkTime,
              read_instantaneous: InstantReader
              ) -> Optional[Tuple[Vector3, Vector3]]:
        """(position, velocity) for ``when``, or None: read it directly."""
        revision = getattr(backend, "revision", None)
        if revision is None or not self._is_eligible(backend, object_path,
                                                     revision):
            self.counts["epochs_unsupported"] += 1
            return None
        if epoch_ms(when) >= epoch_ms(self.stop_time):
            self.counts["epochs_terminal"] += 1
            return None

        key = (object_path, frame)
        cached = self._blocks.get(key)
        if cached is not None and cached[1] != revision:
            self.counts["invalidations_revision"] += 1
            LOGGER.info("orbit batch: backend revision changed (%s -> %s); "
                        "dropping the cached block for %s/%s",
                        cached[1], revision, object_path, frame)
            del self._blocks[key]
            cached = None
        failed = self._failed.get(key)
        if failed is not None and (failed[0] != revision
                                   or epoch_ms(when) not in failed[1]):
            del self._failed[key]
            failed = None

        if cached is None or cached[0].index_of(when) is None:
            if failed is not None:
                self.counts["epochs_instantaneous"] += 1
                return None
            block = self._load(backend, object_path, frame, when,
                               read_instantaneous, revision)
            if block is None:
                self.counts["epochs_instantaneous"] += 1
                return None
            cached = (block, revision)
            self._blocks[key] = cached

        block = cached[0]
        index = block.index_of(when)
        self.counts["epochs_cached"] += 1
        return block.positions[index], block.velocities[index]

    def _is_eligible(self, backend, object_path: str, revision: int) -> bool:
        key = (object_path, revision)
        if key not in self._eligible:
            try:
                propagator = backend.get_propagator_type(object_path)
            except Exception:                     # noqa: BLE001
                propagator = None
            eligible = propagator in BATCHABLE_PROPAGATORS
            self._eligible[key] = eligible
            if not eligible:
                LOGGER.warning("orbit batching off for %s: propagator %r is "
                               "not one of %s", object_path, propagator,
                               BATCHABLE_PROPAGATORS)
        return self._eligible[key]

    def _load(self, backend, object_path, frame, when, read_instantaneous,
              revision) -> Optional[OrbitBlock]:
        epochs = block_epochs(when, self.step_s, self.samples,
                              self.stop_time)
        try:
            block = read_orbit_block(backend, object_path, frame, epochs,
                                     self.step_s)
        except QueryError as exc:
            self.counts["query_failed"] += 1
            return self._fall_back(object_path, frame, epochs, revision,
                                   f"interval query failed: {exc}")
        try:
            block.raise_if_invalid()
        except OrbitBlockError as exc:
            self.counts["validation_failed"] += 1
            return self._fall_back(object_path, frame, epochs, revision,
                                   str(exc))
        problem = self._audit(block, epochs, read_instantaneous)
        if problem:
            self.counts["audits_failed"] += 1
            return self._fall_back(object_path, frame, epochs, revision,
                                   problem)
        self.counts["audits_passed"] += 1
        self.counts["blocks_loaded"] += 1
        return block

    def _audit(self, block: OrbitBlock, epochs, read_instantaneous
               ) -> Optional[str]:
        """Both ends of the block against instantaneous STK reads."""
        for index in (0, len(epochs) - 1):
            try:
                position, velocity = read_instantaneous(epochs[index])
            except QueryError as exc:
                return f"audit read failed at row {index}: {exc}"
            dr = _distance(block.positions[index], position)
            dv = _distance(block.velocities[index], velocity)
            if dr > self.audit_position_m or dv > self.audit_velocity_mps:
                return (f"audit at row {index}: |dr| {dr:.3e} m (limit "
                        f"{self.audit_position_m:.0e}), |dv| {dv:.3e} m/s "
                        f"(limit {self.audit_velocity_mps:.0e})")
        return None

    def _fall_back(self, object_path, frame, epochs, revision, reason):
        self.counts["fallback_blocks"] += 1
        self._failed[(object_path, frame)] = (
            revision, {epoch_ms(e) for e in epochs})
        LOGGER.warning("orbit batch fallback for %s/%s at %s (%d epochs "
                       "read instantaneously): %s", object_path, frame,
                       epochs[0], len(epochs), reason)
        return None

    # -- reporting ---------------------------------------------------------

    def summary_line(self) -> str:
        c = self.counts
        return (f"orbit batch summary: {self.samples} samples per block; "
                f"blocks loaded {c['blocks_loaded']}; audits passed "
                f"{c['audits_passed']}, failed {c['audits_failed']}; "
                f"validation failures {c['validation_failed']}; query "
                f"failures {c['query_failed']}; fallback blocks "
                f"{c['fallback_blocks']}; invalidations "
                f"{c['invalidations_revision']} (revision changes); epochs "
                f"cached {c['epochs_cached']}, instantaneous "
                f"{c['epochs_instantaneous']} (+ terminal "
                f"{c['epochs_terminal']}, unsupported "
                f"{c['epochs_unsupported']})")
