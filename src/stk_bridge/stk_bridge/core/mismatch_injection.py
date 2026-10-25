"""Interface-mismatch injection on the published telemetry.

A fault-injection aid for testing how consumers react to convention
errors. Opt-in (``sensing.inject_mismatch``), off in every shipped
configuration, and applied only to what
``SatellitePublishers.publish_samples`` publishes. What the bridge writes to and reads from
STK is untouched, and the startup validation never sees it: each type
models a consumer that read a correct interface with the wrong
convention.

    quaternion_order      (x, y, z, w) published as (w, x, y, z):
                          scalar-last read as scalar-first
    quaternion_conjugate  the conjugate: inertial->body read as
                          body->inertial
    angle_rate_units      body rate in deg/s where rad/s is expected
    distance_units        position and velocity in km and km/s where
                          m and m/s are expected
    time_offset           telemetry stamped TIME_OFFSET_S late (the
                          TAI - UTC offset); /clock and ScenarioTime
                          are not shifted

A frame-pairing mismatch is not injected here: it is a read-back provider
choice (``sensing.force_attitude_provider``).
"""

from __future__ import annotations

import dataclasses
import math
from typing import Dict

from stk_bridge.backends.base import StkTime

MISMATCH_TYPES = ("quaternion_order", "quaternion_conjugate",
                  "angle_rate_units", "distance_units", "time_offset")

#: TAI - UTC since 2017-01-01: the offset a time-scale mix-up produces.
TIME_OFFSET_S = 37.0


def check(kind: str) -> str:
    """``kind`` if it is a known type or empty; ValueError otherwise."""
    kind = (kind or "").strip()
    if kind and kind not in MISMATCH_TYPES:
        raise ValueError(f"unknown mismatch {kind!r}; expected one of "
                         f"{list(MISMATCH_TYPES)} or empty")
    return kind


def _later(when: StkTime) -> StkTime:
    return StkTime(when.epoch_seconds + TIME_OFFSET_S)


def inject(kind: str, samples: Dict[str, object]) -> Dict[str, object]:
    """``samples`` with mismatch ``kind`` applied; the same dict when
    ``kind`` is empty (nothing copied, nothing changed)."""
    kind = check(kind)
    if not kind:
        return samples
    out = dict(samples)
    attitude = out.get("attitude")
    rate = out.get("angular_rate")
    orbit = out.get("orbit")
    if kind == "quaternion_order" and attitude is not None:
        x, y, z, w = attitude.quaternion_xyzw
        out["attitude"] = dataclasses.replace(attitude,
                                              quaternion_xyzw=(w, x, y, z))
    elif kind == "quaternion_conjugate" and attitude is not None:
        x, y, z, w = attitude.quaternion_xyzw
        out["attitude"] = dataclasses.replace(attitude,
                                              quaternion_xyzw=(-x, -y, -z, w))
    elif kind == "angle_rate_units" and rate is not None:
        out["angular_rate"] = dataclasses.replace(
            rate, angular_velocity_radps=tuple(
                math.degrees(v) for v in rate.angular_velocity_radps))
    elif kind == "distance_units" and orbit is not None:
        out["orbit"] = dataclasses.replace(
            orbit, position_m=tuple(v / 1000.0 for v in orbit.position_m),
            velocity_mps=tuple(v / 1000.0 for v in orbit.velocity_mps))
    elif kind == "time_offset":
        for key in ("orbit", "attitude", "angular_rate"):
            if out.get(key) is not None:
                out[key] = dataclasses.replace(out[key],
                                               time=_later(out[key].time))
    return out
