"""Seeded target generator (Phase 3a, P10b).

A generated scenario is the reference file with only the target list
replaced and the ground station removed: the vehicle, sensor, pointing
and scheduling sections stay byte-for-byte the reference's. Targets are
drawn from the seed; every transition's class is set by construction
from the scheduler's own slew-plus-settle estimate on the dynamics-aware
chain (a declined target does not move the chain) and checked again
with plan_dynamics_aware:

    comfortable  attempted, margin > 25 s
    marginal     attempted, 0 <= margin < 1 s
    infeasible   declined, margin <= -1.5 s

Defaults (generator version 2): 22 targets, at least 11 / 4 / 4 of
those (the rest moderate); image epochs on whole seconds in [40, 700] s,
at least 4 s apart, the last at 650 s or later so the window is used;
cross-track offsets 20-260 km either side.
"""

import pytest
import yaml

from acav import scenario_generator as sg
from acav.run_record import scenario_digest
from acav.scheduling import plan_dynamics_aware, plan_kinematic
from acav.spec import AcavSpec

SEEDS = range(1, 11)
REFERENCE_DIGEST = (
    "c294ae8024093e02b174817e8dcb9ee38fb45dbb783fc60b2cd8cc610e45b28e")


@pytest.fixture(scope="module")
def generated(tmp_path_factory):
    out = tmp_path_factory.mktemp("generated")
    return {seed: sg.write_scenario(seed, str(out)) for seed in SEEDS}


def test_same_seed_same_bytes(tmp_path):
    first = sg.write_scenario(3, str(tmp_path / "a"))
    second = sg.write_scenario(3, str(tmp_path / "b"))
    assert open(first, "rb").read() == open(second, "rb").read()
    assert first.endswith("acav_seed3.yaml")


def test_different_seeds_differ(generated):
    texts = {open(path, "rb").read() for path in generated.values()}
    assert len(texts) == len(generated)


def test_the_header_names_seed_and_version(generated):
    head = open(generated[7], encoding="utf-8").read().split("\n")[:3]
    assert any("seed 7" in line for line in head)
    assert any(f"version {sg.GENERATOR_VERSION}" in line for line in head)


def test_the_reference_is_untouched(generated):
    assert scenario_digest(AcavSpec.locate()) == REFERENCE_DIGEST


@pytest.mark.parametrize("seed", SEEDS)
def test_every_constraint(generated, seed):
    spec = AcavSpec.load(generated[seed])          # loads and validates
    reference = AcavSpec.load()
    targets = sorted(spec.targets, key=lambda t: t.t_image_s)
    assert len(targets) == 22
    assert [t.id for t in spec.targets] == [f"T{k}" for k in range(1, 23)]
    epochs = [t.t_image_s for t in targets]
    assert all(e == int(e) for e in epochs)
    assert 40 <= epochs[0] and 650 <= epochs[-1] <= 700
    assert all(b - a >= 4 for a, b in zip(epochs, epochs[1:]))
    assert all(20 <= abs(t.cross_track_km) <= 260 for t in targets)
    assert spec.ground_station == {}
    for section in ("scenario", "spacecraft", "sensor", "actuators",
                    "pointing", "scheduling", "bridge"):
        assert spec.raw.get(section) == reference.raw.get(section), section
    assert spec.raw["targets"]["placement"] == "ground_track_relative"


@pytest.mark.parametrize("seed", SEEDS)
def test_the_class_mix_is_guaranteed(generated, seed):
    spec = AcavSpec.load(generated[seed])
    plan = plan_dynamics_aware(spec)
    comfortable = [e for e in plan.attempted if e.margin_s > 25.0]
    marginal = [e for e in plan.attempted if 0.0 <= e.margin_s < 1.0]
    infeasible = [d for d in plan.declined if d.margin_s <= -1.5]
    assert len(comfortable) >= 11
    assert len(marginal) >= 4
    assert len(infeasible) >= 4
    assert len(plan.attempted) + len(plan.declined) == 22
    # Every declined target is a designed infeasible one.
    assert len(plan.declined) == len(infeasible)


@pytest.mark.parametrize("seed", SEEDS)
def test_the_table_matches_the_planners(generated, seed):
    spec = AcavSpec.load(generated[seed])
    table = sg.class_table(spec)
    dynamic = plan_dynamics_aware(spec)
    by_id = {e.target_id: e.margin_s for e in dynamic.attempted}
    by_id.update({d.target_id: d.margin_s for d in dynamic.declined})
    kinematic = {e.target_id: e.margin_s
                 for e in plan_kinematic(spec).attempted}
    for row in table:
        assert row["margin_dynamics_aware_s"] == pytest.approx(
            by_id[row["id"]])
        assert row["margin_kinematic_s"] == pytest.approx(
            kinematic[row["id"]])


def test_the_file_is_the_reference_with_targets_replaced(generated):
    text = open(generated[1], encoding="utf-8").read()
    assert "ground_station:" not in text
    assert "\r\n" not in text
    data = yaml.safe_load(text)
    assert len(data["targets"]["list"]) == 22
