"""Tests for the cross-language fixture contract.

The fixtures are what stop the MATLAB twin and the Python reference from
drifting apart, so the machinery that generates and checks them has to
work. In particular the comparison has to be tolerant of last-bit
platform noise and intolerant of real change -- a check that cries wolf
gets ignored, and an ignored check is the same as no check.
"""

import json
import math

import pytest

from acav.spec import AcavSpec
from acav.twin_fixtures import (
    FIXTURE_VERSION,
    build_fixtures,
    compare_fixtures,
)


@pytest.fixture(scope="module")
def spec():
    return AcavSpec.load()


@pytest.fixture(scope="module")
def fixtures(spec):
    return build_fixtures(spec)


class TestGeneration:
    def test_every_section_has_cases(self, fixtures):
        for section in ("quaternion_algebra", "attitude_construction",
                        "wheels", "dynamics", "profile", "controller",
                        "mode_chart", "closed_loop"):
            assert fixtures[section], f"{section} generated no cases"

    def test_it_survives_a_json_round_trip(self, fixtures):
        """The MATLAB side reads JSON, so anything that cannot be
        serialised is not a fixture however useful it looks here."""
        assert compare_fixtures(json.loads(json.dumps(fixtures)),
                                fixtures) == []

    def test_the_config_block_matches_the_scenario(self, fixtures, spec):
        """The MATLAB test derives its gains from this block, so if it
        drifts from the YAML the cross-check silently compares the twin
        against a vehicle that does not exist."""
        config = fixtures["config"]
        assert config["max_torque_Nm"] == spec.actuators.max_torque_Nm
        assert config["inertia_kg_m2"] == [spec.inertia["ixx"],
                                           spec.inertia["iyy"],
                                           spec.inertia["izz"]]
        assert config["capture_error_rad"] == pytest.approx(
            math.radians(spec.pointing.capture_error_deg))

    def test_version_is_declared(self, fixtures):
        """The MATLAB test refuses to run against a version it does not
        know, rather than checking against the wrong expectations."""
        assert fixtures["version"] == FIXTURE_VERSION

    def test_mode_cases_cover_every_transition_and_the_safe_guards(
            self, fixtures):
        names = {case["name"] for case in fixtures["mode_chart"]}
        assert {"nadir_to_slew", "settle_to_image", "safe_latches",
                "saturation_trips_safe_from_image",
                "rate_trips_safe"} <= names

    def test_wheel_cases_cover_both_saturation_kinds(self, fixtures):
        names = {case["name"] for case in fixtures["wheels"]}
        assert {"torque_saturated", "momentum_saturated",
                "failed_wheel"} <= names


class TestComparison:
    def test_identical_trees_have_no_differences(self, fixtures):
        assert compare_fixtures(fixtures, fixtures) == []

    def test_last_bit_noise_is_tolerated(self, fixtures):
        """sin, cos and atan2 come from the platform's libm and differ in
        the last bit between machines. An exact check reports a file
        nobody touched as out of date, which trains the reader to ignore
        it -- confirmed by the container build doing exactly that."""
        perturbed = json.loads(json.dumps(fixtures))
        case = perturbed["controller"][0]
        case["expected_torque_Nm"][0] += 1e-17
        assert compare_fixtures(perturbed, fixtures) == []

    def test_a_real_change_is_reported_with_its_path(self, fixtures):
        perturbed = json.loads(json.dumps(fixtures))
        perturbed["controller"][0]["expected_torque_Nm"][0] += 1e-3
        differences = compare_fixtures(perturbed, fixtures)
        assert len(differences) == 1
        assert "controller[0].expected_torque_Nm[0]" in differences[0]

    def test_a_changed_mode_verdict_is_reported(self, fixtures):
        perturbed = json.loads(json.dumps(fixtures))
        perturbed["mode_chart"][0]["expected_mode"] = "SAFE"
        differences = compare_fixtures(perturbed, fixtures)
        assert any("expected_mode" in d for d in differences)

    def test_a_missing_case_is_reported_not_ignored(self, fixtures):
        perturbed = json.loads(json.dumps(fixtures))
        perturbed["wheels"].pop()
        differences = compare_fixtures(perturbed, fixtures)
        assert any("wheels" in d for d in differences)

    def test_none_is_not_confused_with_zero(self):
        """`settling_time_s` returns None when a run never settled.
        Treating that as 0.0 would quietly bias any campaign average."""
        assert compare_fixtures({"a": 0.0}, {"a": None})
        assert compare_fixtures({"a": None}, {"a": 0.0})
        assert compare_fixtures({"a": None}, {"a": None}) == []

    def test_booleans_are_not_compared_as_numbers(self):
        """`expected_saturated` is a flag. False == 0 numerically, and a
        tolerant numeric comparison would let a flag flip pass."""
        assert compare_fixtures({"a": False}, {"a": 0.0})
        assert compare_fixtures({"a": True}, {"a": 1.0})
        assert compare_fixtures({"a": True}, {"a": True}) == []
