"""Seeded wheel faults for the fault-replay runs (E5, part of E3).

Fault replay asks whether the closed loop gives *deterministic fault
replay*: inject a wheel fault at a random epoch, observe the twin trip
SAFE, then re-run the seed that failed and reproduce it exactly. The
claim is about reproducibility, so the fault schedule itself has to be
reproducible before anything downstream of it can be.

That is a narrower requirement than "use a random number generator":

* **The seed must fully determine the fault.** Not the seed plus the
  order runs happened to execute in, not the seed plus a module-level
  RNG some other component also draws from. ``plan_fault`` takes a seed
  and returns a fault; nothing else influences it, and it never touches
  the global :mod:`random` state.
* **The fault must be a value, not an event.** A fault decided by
  sampling at the moment of injection cannot be written into the record
  before the run, cannot be diffed between two runs, and cannot be
  replayed on a machine whose RNG differs. Here the whole schedule is
  computed up front, stored in the run record, and applied by lookup.
* **Replay must not depend on this module staying the same.** A record
  carries the resolved fault, so re-running an old seed against a newer
  harness uses the fault that actually ran rather than whatever the
  current sampling rule would now produce. :func:`fault_from_dict` is
  what replay uses; ``plan_fault`` is only for generating new seeds.

What is injected
----------------
One reaction wheel, at one epoch, in one of two ways. Both reuse
machinery the twin already has, deliberately: a fault needing its own
physics would measure the fault model rather than the loop's ability to
replay it.

``SATURATION`` (the default, and what the case study injects)
drives the wheel's stored momentum to capacity. The momentum guard sees
a full wheel on the next mode evaluation and the chart trips SAFE. The
wheel is not jammed there -- the actuator model lets a saturated wheel
torque back towards zero, so this stays a fault injection rather than a
termination.

``FAILURE`` derates the wheel instead, via the per-axis ``health``
``adcs.wheels`` already takes, so the axis delivers no torque at all.

A dead wheel does not make momentum build on the remaining axes until
the momentum guard trips. Flying it shows the pointing error growing
without bound (23 deg to 178 deg over a minute) while the healthy wheels
stay near zero momentum and no guard fires, because the error is about
the dead axis and the controller asks the other two for almost nothing.
That is a gap in the vehicle's fault detection; it is not the fault the
replay study injects, which is why SATURATION is the default.
"""

from __future__ import annotations

import random
from dataclasses import dataclass
from typing import Any, Dict, Optional, Sequence, Tuple

#: Axis names in the order ``adcs.wheels`` takes them.
AXIS_NAMES = ("x", "y", "z")

#: Health value for a wheel that has failed outright. A partial derate
#: is expressible too (see :attr:`WheelFault.health`), but the default
#: campaign uses total failure: a gradual degradation makes pointing
#: decay smoothly and reads as a tuning problem rather than a fault,
#: which is the same reasoning that made wheel saturation a hard clamp.
DEAD = 0.0

#: Drive the wheel's stored momentum to capacity, tripping the
#: momentum guard. The default fault.
SATURATION = "saturation"

#: Derate the wheel so it delivers no torque. Degrades pointing badly
#: and trips nothing -- see this module's own docstring.
FAILURE = "failure"

KINDS = (SATURATION, FAILURE)


@dataclass(frozen=True)
class WheelFault:
    """One wheel degrading at one epoch.

    ``axis`` is 0/1/2 for x/y/z. ``at_s`` is seconds from the window
    start, matching every other time in this study. ``health`` is the
    derate applied from ``at_s`` onwards: 0.0 is a dead wheel, 1.0 is a
    healthy one, and values between are a partial failure.
    """

    axis: int
    at_s: float
    health: float = DEAD
    seed: Optional[int] = None
    #: ``SATURATION`` or ``FAILURE``; see the module docstring for what
    #: each one actually does to the vehicle, as measured.
    kind: str = SATURATION

    def __post_init__(self) -> None:
        if self.axis not in (0, 1, 2):
            raise ValueError(f"axis must be 0, 1 or 2; got {self.axis!r}")
        if self.kind not in KINDS:
            raise ValueError(
                f"kind must be one of {list(KINDS)}; got {self.kind!r}")
        if not 0.0 <= self.health <= 1.0:
            raise ValueError(
                f"health is a derate in [0, 1]; got {self.health!r}")
        if self.at_s < 0.0:
            raise ValueError(f"at_s must not be negative; got {self.at_s!r}")

    @property
    def axis_name(self) -> str:
        return AXIS_NAMES[self.axis]

    def health_at(self, time_s: float) -> Tuple[float, float, float]:
        """Per-axis health at ``time_s``, for ``adcs.wheels``.

        Always healthy for a ``SATURATION`` fault: that one fills the
        wheel rather than breaking it, and a wheel at capacity can still
        deliver torque that unloads it.

        A step rather than a ramp, and applied from ``at_s`` *inclusive*
        so that a fault scheduled exactly on a step boundary is applied
        on that step. Leaving it exclusive would make the injection
        epoch depend on floating-point comparison at the boundary, which
        could make two replays diverge by one step.
        """
        healthy = [1.0, 1.0, 1.0]
        if self.kind == FAILURE and time_s >= self.at_s:
            healthy[self.axis] = self.health
        return (healthy[0], healthy[1], healthy[2])

    def saturates_at(self, time_s: float) -> bool:
        """Whether a SATURATION fault is due by ``time_s``.

        The caller applies it once, tracking that in its own state --
        this only answers whether the epoch has arrived, so the answer
        stays a pure function of (fault, time).
        """
        return self.kind == SATURATION and time_s >= self.at_s

    def to_dict(self) -> Dict[str, Any]:
        return {
            "axis": self.axis,
            "axis_name": self.axis_name,
            "at_s": self.at_s,
            "health": self.health,
            "seed": self.seed,
            "kind": self.kind,
        }


def fault_from_dict(raw: Optional[Dict[str, Any]]) -> Optional[WheelFault]:
    """Rebuild a fault from a run record, for replay.

    This -- not :func:`plan_fault` -- is what re-running a failing seed
    goes through. The record carries the fault that actually ran, so a
    replay reproduces that run even if the sampling rule below has since
    changed. Re-deriving the fault from the seed instead would make
    every earlier result depend on the current version of this module.
    """
    if not raw:
        return None
    seed = raw.get("seed")
    return WheelFault(
        axis=int(raw["axis"]),
        at_s=float(raw["at_s"]),
        health=float(raw.get("health", DEAD)),
        seed=None if seed is None else int(seed),
        kind=str(raw.get("kind", SATURATION)))


def plan_fault(seed: int, window_duration_s: float,
               earliest_s: float = 0.0,
               latest_s: Optional[float] = None,
               health: float = DEAD,
               axes: Sequence[int] = (0, 1, 2),
               kind: str = SATURATION) -> WheelFault:
    """The fault for ``seed``, drawn from a private generator.

    Uses ``random.Random(seed)`` rather than the module-level functions
    on purpose: the global RNG is shared with anything else in the
    process that draws from it, so a fault planned through it would
    depend on how many other draws happened first. That is reproducible
    only by accident, and stops being reproducible the moment an
    unrelated component starts using randomness.

    The epoch is quantised to milliseconds so that the value written
    into the record is the value applied, with no float formatting in
    between. An epoch that survived a JSON round trip differently from
    how it was drawn would give a replay that differs from its original
    by one control step.
    """
    if window_duration_s <= 0.0:
        raise ValueError("window_duration_s must be positive")
    if not axes:
        raise ValueError("at least one axis must be eligible to fail")

    end = window_duration_s if latest_s is None else latest_s
    if not 0.0 <= earliest_s < end <= window_duration_s:
        raise ValueError(
            f"need 0 <= earliest_s ({earliest_s}) < latest_s ({end}) <= "
            f"window_duration_s ({window_duration_s})")

    rng = random.Random(seed)
    axis = rng.choice(list(axes))
    at_s = round(rng.uniform(earliest_s, end), 3)
    return WheelFault(axis=axis, at_s=at_s, health=health, seed=seed,
                      kind=kind)


def plan_campaign_faults(seeds: Sequence[int], window_duration_s: float,
                         **kwargs) -> Dict[int, WheelFault]:
    """One fault per seed, as a mapping, for a Monte Carlo campaign.

    Each seed is planned independently, so adding, removing or
    re-ordering seeds cannot change the fault any other seed gets. Drawing
    sequentially from one generator would renumber every fault when a
    seed was inserted, and a replay would then reproduce a different
    fault than the one that failed.
    """
    return {seed: plan_fault(seed, window_duration_s, **kwargs)
            for seed in seeds}
