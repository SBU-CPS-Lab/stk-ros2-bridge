"""Read a block of orbit states with two interval queries, and validate it.

The equivalence diagnostic uses this to compare interval reads with the
instantaneous reads the sensing loop makes; HPOP orbit batching (C) is to
reuse it unchanged. It never interpolates or propagates: every row is
STK's own, and the block is only usable if STK returned exactly the
requested epochs.

The epoch grid
--------------
Lockstep time is ``epoch + sim_time`` with ``sim_time += step`` each step
(``core/time_sync.py``), and every STK request is UTCG text rounded to the
millisecond (``connect_protocol.format_utcg``). A block therefore names
its epochs by millisecond (:func:`epoch_ms`), and a row belongs to a
step when both name the same millisecond. Blocks never contain an epoch
at or after the scenario stop time: the terminal epoch is read
instantaneously, as it is today.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from stk_bridge.backends import connect_protocol as proto
from stk_bridge.backends.base import StkBackend, StkTime
from stk_bridge.core import data_providers as dp

Vector3 = Tuple[float, float, float]


def epoch_ms(when) -> int:
    """The millisecond an epoch names in a UTCG request."""
    seconds = when.epoch_seconds if isinstance(when, StkTime) else when
    return int(round(seconds * 1000.0))


def block_epochs(start: StkTime, step_s: float, max_samples: int,
                 stop: StkTime) -> List[StkTime]:
    """``start + k*step`` for k < max_samples, every one before ``stop``."""
    stop_ms = epoch_ms(stop)
    epochs: List[StkTime] = []
    for k in range(max_samples):
        when = start + k * step_s
        if epoch_ms(when) >= stop_ms:
            break
        epochs.append(when)
    return epochs


class OrbitBlockError(ValueError):
    """An orbit block that cannot stand in for instantaneous reads."""


@dataclass
class OrbitBlock:
    """STK's rows for a block of epochs, and what is wrong with them."""

    object_path: str
    frame: str
    requested_ms: List[int]
    position_time_ms: List[Optional[int]] = field(default_factory=list)
    velocity_time_ms: List[Optional[int]] = field(default_factory=list)
    positions: List[Vector3] = field(default_factory=list)
    velocities: List[Vector3] = field(default_factory=list)
    issues: List[str] = field(default_factory=list)

    def index_of(self, when) -> Optional[int]:
        """Row index for an epoch of this block, by requested millisecond."""
        try:
            return self._index[epoch_ms(when)]
        except KeyError:
            return None

    def __post_init__(self) -> None:
        self._index: Dict[int, int] = {
            ms: i for i, ms in enumerate(self.requested_ms)}

    @property
    def aligned(self) -> bool:
        """Rows can be matched to epochs by position (counts agree)."""
        n = len(self.requested_ms)
        return len(self.positions) == n and len(self.velocities) == n

    def raise_if_invalid(self) -> None:
        if self.issues:
            raise OrbitBlockError(
                f"orbit block for {self.object_path}/{self.frame} at "
                f"{self.requested_ms[0] if self.requested_ms else '?'} ms: "
                + "; ".join(self.issues))


def _times_ms(values) -> Tuple[List[Optional[int]], Optional[str]]:
    out: List[Optional[int]] = []
    for value in values:
        if not isinstance(value, str):
            return out, ("time column is not UTCG text "
                         f"(got {type(value).__name__})")
        try:
            out.append(epoch_ms(proto.parse_utcg(value)))
        except Exception as exc:                  # noqa: BLE001
            return out, f"time {value!r} is not UTCG: {exc}"
    return out, None


def _read(backend: StkBackend, spec: dp.ProviderSpec, names, object_path,
          frame, epochs: Sequence[StkTime], step_s: float, block: OrbitBlock):
    data, mapping = dp.execute(backend, spec, object_path, frame=frame,
                               start=epochs[0], stop=epochs[-1],
                               step_s=step_s)
    label = spec.provider_name
    if data.row_count != len(epochs):
        block.issues.append(f"{label}: {data.row_count} rows for "
                            f"{len(epochs)} requested epochs")
    times: List[Optional[int]] = []
    column = mapping.get("time")
    if column is None:
        block.issues.append(f"{label}: no time column")
    else:
        times, problem = _times_ms(data.columns.get(column, []))
        if problem:
            block.issues.append(f"{label}: {problem}")
    values = [dp.column_values(data, spec, mapping, name) for name in names]
    rows = list(zip(*values))
    if not all(math.isfinite(v) for row in rows for v in row):
        block.issues.append(f"{label}: not every value is finite")
    return times, [tuple(float(v) for v in row) for row in rows]


def read_orbit_block(backend: StkBackend, object_path: str, frame: str,
                     epochs: Sequence[StkTime], step_s: float) -> OrbitBlock:
    """Two interval queries (position, velocity) over ``epochs``.

    ``epochs`` is a :func:`block_epochs` result. Problems are collected in
    ``OrbitBlock.issues`` rather than raised, so a diagnostic can still
    measure a block that (C) would reject; (C) calls
    :meth:`OrbitBlock.raise_if_invalid`. A failed STK query raises
    ``QueryError`` as usual.
    """
    if not epochs:
        raise ValueError("an orbit block needs at least one epoch")
    block = OrbitBlock(object_path=object_path, frame=frame,
                       requested_ms=[epoch_ms(e) for e in epochs])
    block.position_time_ms, block.positions = _read(
        backend, dp.CARTESIAN_POSITION, ("x", "y", "z"), object_path,
        frame, epochs, step_s, block)
    block.velocity_time_ms, block.velocities = _read(
        backend, dp.CARTESIAN_VELOCITY, ("vx", "vy", "vz"), object_path,
        frame, epochs, step_s, block)
    for label, times in (("Cartesian Position", block.position_time_ms),
                         ("Cartesian Velocity", block.velocity_time_ms)):
        for index, (want, got) in enumerate(zip(block.requested_ms, times)):
            if want != got:
                block.issues.append(
                    f"{label}: time grid mismatch at row {index}: requested "
                    f"{want} ms, returned {got} ms")
                break
    return block
