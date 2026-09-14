"""Seeded wheel faults for E5.

E5 asks whether a closed digital thread gives *deterministic fault
replay*: inject a wheel failure at a random epoch, observe the twin trip
SAFE, then re-run the seed that failed and reproduce it exactly. The
claim is about reproducibility, so the fault schedule itself has to be
reproducible before anything downstream of it can be.

That is a narrower requirement than "use a random number generator", and
the difference is where this kind of harness usually goes wrong:

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
A single reaction wheel degrading at one epoch, because that is the
fault the twin already models end to end: ``adcs.wheels`` takes a
per-axis ``health`` derate, a dead axis makes the controller unable to
deliver commanded torque about it, momentum builds on the remaining
axes, and the Stateflow chart trips SAFE on the momentum guard. Nothing
new has to be modelled for E5, which is deliberate -- a fault that
needed its own physics would be measuring the fault model rather than
the loop's ability to replay it.
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

    def __post_init__(self) -> None:
        if self.axis not in (0, 1, 2):
            raise ValueError(f"axis must be 0, 1 or 2; got {self.axis!r}")
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

        A step rather than a ramp, and applied from ``at_s`` *inclusive*
        so that a fault scheduled exactly on a step boundary is applied
        on that step. Leaving it exclusive would make the injection
        epoch depend on floating-point comparison at the boundary, which
        is exactly the kind of one-step ambiguity that makes two
        supposedly identical replays diverge.
        """
        healthy = [1.0, 1.0, 1.0]
        if time_s >= self.at_s:
            healthy[self.axis] = self.health
        return (healthy[0], healthy[1], healthy[2])

    def to_dict(self) -> Dict[str, Any]:
        return {
            "axis": self.axis,
            "axis_name": self.axis_name,
            "at_s": self.at_s,
            "health": self.health,
            "seed": self.seed,
        }


def fault_from_dict(raw: Optional[Dict[str, Any]]) -> Optional[WheelFault]:
    """Rebuild a fault from a run record, for replay.

    This -- not :func:`plan_fault` -- is what re-running a failing seed
    goes through. The record carries the fault that actually ran, so a
    replay reproduces that run even if the sampling rule below has since
    changed. Re-deriving the fault from the seed instead would make
    every historical result quietly depend on the current version of
    this module.
    """
    if not raw:
        return None
    seed = raw.get("seed")
    return WheelFault(
        axis=int(raw["axis"]),
        at_s=float(raw["at_s"]),
        health=float(raw.get("health", DEAD)),
        seed=None if seed is None else int(seed))


def plan_fault(seed: int, window_duration_s: float,
               earliest_s: float = 0.0,
               latest_s: Optional[float] = None,
               health: float = DEAD,
               axes: Sequence[int] = (0, 1, 2)) -> WheelFault:
    """The fault for ``seed``, drawn from a private generator.

    Uses ``random.Random(seed)`` rather than the module-level functions
    on purpose: the global RNG is shared with anything else in the
    process that draws from it, so a fault planned through it would
    depend on how many other draws happened first. That is reproducible
    only by accident, and stops being reproducible the moment an
    unrelated component starts using randomness.

    The epoch is quantised to milliseconds so that the value written
    into the record is the value applied, with no float formatting in
    between. An epoch that survives a JSON round trip differently from
    how it was drawn is a replay that differs from its original by one
    control step, which is the smallest possible way for this experiment
    to be wrong and the hardest to notice.
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
    return WheelFault(axis=axis, at_s=at_s, health=health, seed=seed)


def plan_campaign_faults(seeds: Sequence[int], window_duration_s: float,
                         **kwargs) -> Dict[int, WheelFault]:
    """One fault per seed, as a mapping, for a Monte Carlo campaign.

    Each seed is planned independently, so adding, removing or
    re-ordering seeds cannot change the fault any other seed gets. A
    campaign that drew sequentially from one generator would renumber
    every fault the moment a seed was inserted, and E5's "re-run the
    failing seed" would then reproduce a different fault than the one
    that failed.
    """
    return {seed: plan_fault(seed, window_duration_s, **kwargs)
            for seed in seeds}
