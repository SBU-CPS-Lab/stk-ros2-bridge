"""Seeded imaging-target scenarios (Phase 3a).

``write_scenario(seed, out_dir)`` writes ``acav_seed<seed>.yaml``: the
reference scenario (config/acav_scenario.yaml) with only its target list
replaced and its ground station removed. The vehicle, sensor, pointing,
scheduling and bridge sections are the reference's, byte for byte, so
the twin -- which always reads the reference -- has the same parameters.

Targets are drawn from ``random.Random(seed)``. Each transition's class
is set by construction from the scheduler's own estimate
(``AcavSpec.estimated_slew_time_s``: slew plus settle) on the
dynamics-aware chain, where a declined target does not move the chain
(``scheduling.plan_dynamics_aware``):

    comfortable  margin > 25 s       (attempted)
    moderate     5 s <= margin       (attempted; fills the rest)
    marginal     0 <= margin < 1 s   (attempted)
    infeasible   margin <= -1.5 s    (declined)

with ``margin = (t_image - previous attempted t_image) - required``.
Image epochs are whole seconds, so each target's epoch is rounded in the
direction that keeps its class: up for comfortable, moderate and
marginal, down for infeasible. A draw that cannot meet the window or the
spacing is redrawn; the whole sequence is retried, deterministically,
if it does not fit. The result is checked against plan_dynamics_aware
before it is written.

Same seed and generator version, same bytes.
"""

from __future__ import annotations

import argparse
import math
import os
import random
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

#: 1: 12 targets (>= 6/2/2), ending by about 410 s.
#: 2: 22 targets (>= 11/4/4, the rest moderate), the last image epoch at
#:    650 s or later, so the targets fill the window; everything else as 1.
GENERATOR_VERSION = "2"

COMFORTABLE, MODERATE, MARGINAL, INFEASIBLE = (
    "comfortable", "moderate", "marginal", "infeasible")


@dataclass(frozen=True)
class GeneratorConfig:
    n_targets: int = 22
    earliest_s: int = 40
    latest_s: int = 700
    #: The last image epoch must be at least this late (window coverage).
    last_at_least_s: int = 650
    min_spacing_s: int = 4
    cross_track_min_km: int = 20
    cross_track_max_km: int = 260
    #: At least this many transitions of each class; the rest are
    #: MODERATE.
    quota: Tuple[Tuple[str, int], ...] = (
        (COMFORTABLE, 11), (MARGINAL, 4), (INFEASIBLE, 4))
    #: Margin drawn for each class, seconds (MARGINAL is set by rounding).
    margins: Tuple[Tuple[str, Tuple[float, float]], ...] = (
        (COMFORTABLE, (25.0, 40.0)), (MODERATE, (5.0, 20.0)),
        (INFEASIBLE, (-8.0, -1.5)))
    max_attempts: int = 200


@dataclass(frozen=True)
class GeneratedTarget:
    id: str
    t_image_s: int
    cross_track_km: int
    designed_class: str


def _classes(config: GeneratorConfig, rng: random.Random) -> List[str]:
    classes: List[str] = []
    for name, count in config.quota:
        classes += [name] * count
    classes += [MODERATE] * (config.n_targets - len(classes))
    if len(classes) != config.n_targets:
        raise ValueError("the quota exceeds the number of targets")
    rng.shuffle(classes)
    return classes


def _try_sequence(spec, config: GeneratorConfig,
                  rng: random.Random) -> Optional[List[GeneratedTarget]]:
    margins = dict(config.margins)
    targets: List[GeneratedTarget] = []
    previous_cross, previous_t = 0.0, 0     # the dynamics-aware chain
    last_epoch = None                        # any target, declined or not
    for index, kind in enumerate(_classes(config, rng), start=1):
        placed = None
        for _ in range(50):
            side = rng.choice((-1, 1))
            cross = side * rng.randint(config.cross_track_min_km,
                                       config.cross_track_max_km)
            required = spec.estimated_slew_time_s(previous_cross, cross)
            if kind == MARGINAL:
                t = previous_t + math.ceil(required)
            elif kind == INFEASIBLE:
                low, high = margins[kind]
                t = previous_t + math.floor(
                    required + rng.uniform(low, high))
            else:
                low, high = margins[kind]
                t = previous_t + math.ceil(
                    required + rng.uniform(low, high))
            earliest = (config.earliest_s if last_epoch is None
                        else last_epoch + config.min_spacing_s)
            if earliest <= t <= config.latest_s:
                placed = (t, cross)
                break
        if placed is None:
            return None
        t, cross = placed
        targets.append(GeneratedTarget(f"T{index}", t, cross, kind))
        last_epoch = t
        if kind != INFEASIBLE:
            previous_cross, previous_t = float(cross), t
    if targets[-1].t_image_s < config.last_at_least_s:
        return None
    return targets


def generate_targets(seed: int, spec,
                     config: GeneratorConfig = GeneratorConfig()
                     ) -> List[GeneratedTarget]:
    """The targets for ``seed`` against ``spec``'s vehicle."""
    rng = random.Random(seed)
    for _ in range(config.max_attempts):
        targets = _try_sequence(spec, config, rng)
        if targets is not None:
            return targets
    raise RuntimeError(f"seed {seed}: no target sequence fitted the window "
                       f"in {config.max_attempts} attempts")


def _render(reference_text: str, targets: List[GeneratedTarget],
            seed: int) -> str:
    lines = reference_text.replace("\r\n", "\n").split("\n")
    banner = [i for i, line in enumerate(lines)
              if line.startswith("# ----")]

    # The ground station: its banner (three lines) through the line
    # before the next banner.
    station = lines.index("ground_station:")
    start = max(i for i in banner if i < station) - 1
    end = min(i for i in banner if i > station)
    lines = lines[:start] + lines[end:]

    # The target list: from "  list:" to the next banner.
    first = lines.index("  list:")
    stop = min(i for i, line in enumerate(lines)
               if i > first and line.startswith("# ----"))
    block = ["  list:"]
    for target in targets:
        block += [f"    - id: {target.id}",
                  f"      t_image_s: {float(target.t_image_s):.1f}",
                  f"      cross_track_km: {float(target.cross_track_km):.1f}",
                  "      priority: 1.0",
                  f"      # designed: {target.designed_class}",
                  ""]
    lines = lines[:first] + block + lines[stop:]

    header = [
        f"# GENERATED by acav.scenario_generator, generator version "
        f"{GENERATOR_VERSION}, from seed {seed}. Do not edit by hand.",
        "# The reference scenario (config/acav_scenario.yaml) with its "
        "target list replaced and its",
        "# ground station removed; every other section is the "
        "reference's, unchanged.",
    ]
    return "\n".join(header + lines)


def class_table(spec) -> List[Dict]:
    """Per target: the dynamics-aware and kinematic margins and class."""
    from acav.scheduling import plan_dynamics_aware, plan_kinematic

    dynamic = plan_dynamics_aware(spec)
    margins = {e.target_id: (e.margin_s, True) for e in dynamic.attempted}
    margins.update({d.target_id: (d.margin_s, False)
                    for d in dynamic.declined})
    kinematic = {e.target_id: e.margin_s
                 for e in plan_kinematic(spec).attempted}
    rows = []
    for target in sorted(spec.targets, key=lambda t: t.t_image_s):
        margin, attempted = margins[target.id]
        if not attempted:
            kind = INFEASIBLE if margin <= -1.5 else "declined"
        elif margin > 25.0:
            kind = COMFORTABLE
        elif 0.0 <= margin < 1.0:
            kind = MARGINAL
        else:
            kind = MODERATE
        rows.append({"id": target.id, "t_image_s": target.t_image_s,
                     "cross_track_km": target.cross_track_km,
                     "margin_dynamics_aware_s": margin,
                     "margin_kinematic_s": kinematic[target.id],
                     "class": kind})
    return rows


def write_scenario(seed: int, out_dir: str,
                   reference_path: str = "",
                   config: GeneratorConfig = GeneratorConfig()) -> str:
    """Generate, check and write ``<out_dir>/acav_seed<seed>.yaml``."""
    from acav.spec import AcavSpec

    reference_path = AcavSpec.locate(reference_path or None)
    reference = AcavSpec.load(reference_path)
    with open(reference_path, "rb") as handle:
        reference_text = handle.read().decode("utf-8")
    targets = generate_targets(seed, reference, config)
    text = _render(reference_text, targets, seed)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, f"acav_seed{seed}.yaml")
    temporary = path + ".partial"
    with open(temporary, "wb") as handle:
        handle.write(text.encode("utf-8"))
    try:
        rows = class_table(AcavSpec.load(temporary))
        designed = {t.id: t.designed_class for t in targets}
        wrong = [r["id"] for r in rows
                 if r["class"] != designed[r["id"]]
                 and not (designed[r["id"]] == MODERATE
                          and r["class"] == COMFORTABLE)]
        if wrong:
            raise RuntimeError(f"seed {seed}: targets {wrong} did not land "
                               f"in their designed class")
    except Exception:
        os.remove(temporary)
        raise
    os.replace(temporary, path)
    return path


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seeds", required=True,
                        help="e.g. 1-10 or 3,5,8")
    parser.add_argument("--out-dir", required=True)
    args = parser.parse_args(argv)
    seeds: List[int] = []
    for part in args.seeds.split(","):
        if "-" in part:
            low, high = part.split("-")
            seeds += list(range(int(low), int(high) + 1))
        else:
            seeds.append(int(part))
    from acav.spec import AcavSpec

    for seed in seeds:
        path = write_scenario(seed, args.out_dir)
        print(f"seed {seed}: {path}")
        for row in class_table(AcavSpec.load(path)):
            print(f"  {row['id']:>4} t={row['t_image_s']:6.1f} s "
                  f"x={row['cross_track_km']:7.1f} km  "
                  f"margin {row['margin_dynamics_aware_s']:+7.2f} s "
                  f"(kinematic {row['margin_kinematic_s']:+7.2f} s)  "
                  f"{row['class']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
