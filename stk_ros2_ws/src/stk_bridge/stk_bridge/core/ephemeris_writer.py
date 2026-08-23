"""Ephemeris file writers for the batch (non-real-time) path.

The live control loop injects state sample-by-sample via ``SetPosition``.
This module serves the other case: a trajectory already computed in
Simulink that should be handed to STK whole, via the ``StkExternal``
propagator.

When to use which
-----------------
Real-time injection wins for closed-loop control -- it is incremental and
needs no file I/O. Batch ephemeris wins when:

* the trajectory is known in advance and will be replayed repeatedly,
* the vehicle must participate in **Coverage** analysis, which excludes
  objects on the RealTime propagator, or
* the run needs to survive a bridge restart, since injected real-time
  samples do not persist.

Format notes
------------
STK ``.e`` is written with ``EphemerisTimePosVel``: time in seconds from
the file epoch, then position and velocity. Units are metres and
metres/second, matching the SI unit preferences the backends force, so
no scaling happens here. ``CoordinateSystem`` is written explicitly
rather than left to default -- an ephemeris file whose frame is implicit
is a frame bug waiting to happen.
"""

from __future__ import annotations

import logging
import os
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from typing import Iterable, List, Optional, Sequence, Tuple

from stk_bridge.backends.base import StkTime
from stk_bridge.backends import connect_protocol as proto

LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True)
class EphemerisPoint:
    time: StkTime
    position_m: Tuple[float, float, float]
    velocity_mps: Tuple[float, float, float] = (0.0, 0.0, 0.0)


def _validate(points: Sequence[EphemerisPoint]) -> List[EphemerisPoint]:
    """Sort and reject duplicate epochs.

    STK's external ephemeris reader requires monotonically increasing
    times, and a duplicate epoch produces a file it accepts but
    interpolates strangely. Sorting rather than erroring on out-of-order
    input is deliberate: a batch trajectory assembled from several ROS
    messages can legitimately arrive unordered, unlike the real-time
    path where ordering carries information about lateness.
    """
    if not points:
        raise ValueError("cannot write an ephemeris file with no points")

    ordered = sorted(points, key=lambda p: p.time.epoch_seconds)
    deduped: List[EphemerisPoint] = []
    for point in ordered:
        if deduped and abs(point.time.epoch_seconds
                           - deduped[-1].time.epoch_seconds) < 1e-9:
            LOGGER.warning("dropping duplicate ephemeris epoch %.6f",
                           point.time.epoch_seconds)
            continue
        deduped.append(point)
    return deduped


def write_stk_ephemeris(path: str, points: Sequence[EphemerisPoint],
                        coordinate_system: str = "ICRF",
                        central_body: str = "Earth",
                        interpolation_order: int = 5) -> str:
    """Write an STK ``.e`` ephemeris file. Returns the path written."""
    ordered = _validate(points)
    epoch = ordered[0].time

    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)

    lines: List[str] = [
        "stk.v.12.0",
        "",
        "BEGIN Ephemeris",
        "",
        f"NumberOfEphemerisPoints {len(ordered)}",
        f"ScenarioEpoch            {proto.format_utcg(epoch)}",
        f"CentralBody              {central_body}",
        f"CoordinateSystem         {coordinate_system}",
        f"InterpolationMethod      Lagrange",
        f"InterpolationOrder       {interpolation_order}",
        "",
        "EphemerisTimePosVel",
        "",
    ]

    for point in ordered:
        offset = point.time.epoch_seconds - epoch.epoch_seconds
        px, py, pz = point.position_m
        vx, vy, vz = point.velocity_mps
        lines.append(
            f"{offset!r} {px!r} {py!r} {pz!r} {vx!r} {vy!r} {vz!r}"
        )

    lines += ["", "END Ephemeris", ""]

    with open(path, "w", encoding="ascii", newline="\n") as handle:
        handle.write("\n".join(lines))

    LOGGER.info("wrote %d ephemeris points to %s (%s, %s)",
                len(ordered), path, coordinate_system, central_body)
    return path


def write_ccsds_oem(path: str, points: Sequence[EphemerisPoint],
                    object_name: str = "SSS-1",
                    object_id: str = "SSS-1",
                    center_name: str = "EARTH",
                    ref_frame: str = "ICRF",
                    originator: str = "stk_bridge") -> str:
    """Write a CCSDS OEM (502.0-B) ephemeris file.

    Offered alongside the native ``.e`` because OEM is the interchange
    format for anything that must also be read by a tool other than STK
    -- flight dynamics teams, ESA ground systems, other simulators.

    OEM positions are kilometres by convention, so this is the one place
    in the bridge that converts away from SI. The conversion is local and
    explicit for exactly that reason.
    """
    ordered = _validate(points)

    def iso(when: StkTime) -> str:
        moment = (datetime(1970, 1, 1, tzinfo=timezone.utc)
                  + timedelta(seconds=when.epoch_seconds))
        return moment.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3]

    now = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S")

    lines: List[str] = [
        "CCSDS_OEM_VERS = 2.0",
        f"CREATION_DATE  = {now}",
        f"ORIGINATOR     = {originator}",
        "",
        "META_START",
        f"OBJECT_NAME          = {object_name}",
        f"OBJECT_ID            = {object_id}",
        f"CENTER_NAME          = {center_name}",
        f"REF_FRAME            = {ref_frame}",
        "TIME_SYSTEM          = UTC",
        f"START_TIME           = {iso(ordered[0].time)}",
        f"STOP_TIME            = {iso(ordered[-1].time)}",
        "META_STOP",
        "",
    ]

    for point in ordered:
        px, py, pz = (v / 1000.0 for v in point.position_m)
        vx, vy, vz = (v / 1000.0 for v in point.velocity_mps)
        lines.append(
            f"{iso(point.time)} {px:.9f} {py:.9f} {pz:.9f} "
            f"{vx:.9f} {vy:.9f} {vz:.9f}"
        )

    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, "w", encoding="ascii", newline="\n") as handle:
        handle.write("\n".join(lines) + "\n")

    LOGGER.info("wrote %d OEM points to %s (%s, km)",
                len(ordered), path, ref_frame)
    return path


def load_into_stk(backend, object_path: str, file_path: str) -> None:
    """Point an object's StkExternal propagator at a written file.

    Note this leaves the object on ``StkExternal``, not ``RealTime`` --
    the two are mutually exclusive, and switching to external discards
    any real-time samples already injected.
    """
    backend.load_external_ephemeris(object_path, os.path.abspath(file_path))
    LOGGER.info("%s now propagating from %s", object_path, file_path)
