"""Tests for the scenario spec loader and its cross-checks."""

import copy

import pytest

from acav.spec import AcavSpec


@pytest.fixture
def spec():
    return AcavSpec.load()


class TestLoading:
    def test_default_spec_loads_and_validates(self, spec):
        assert spec.name == "ACAV_SSS1"
        assert len(spec.targets) == 7

    def test_altitude_derived_not_read(self, spec):
        assert spec.altitude_km == pytest.approx(500.0, abs=0.1)

    def test_slew_limits_recomputed_from_inertia_and_actuators(self, spec):
        limits = spec.slew_limits()
        assert limits.alpha_deg_s2 == pytest.approx(0.932, abs=0.01)
        assert limits.omega_max_deg_s == pytest.approx(9.32, abs=0.05)

    def test_summary_renders(self, spec):
        text = spec.summary()
        assert "ACAV_SSS1" in text
        assert "T1" in text and "T7" in text


class TestCrossChecks:
    def _mutate(self, spec, path, value):
        raw = copy.deepcopy(spec.raw)
        node = raw
        for key in path[:-1]:
            node = node[key]
        node[path[-1]] = value
        return raw

    def test_hold_time_mismatch_is_rejected(self, spec):
        """pointing.settle_hold_s and scheduling.hold_before_image_s are
        the same physical quantity; divergence would make the twin and
        the scheduler disagree about what a capture is."""
        raw = self._mutate(spec, ["scheduling", "hold_before_image_s"], 5.0)
        with pytest.raises(ValueError, match="same physical quantity"):
            AcavSpec.from_dict(raw)

    def test_capture_tolerance_wider_than_fov_is_rejected(self, spec):
        raw = self._mutate(spec, ["pointing", "capture_error_deg"], 3.0)
        with pytest.raises(ValueError, match="vacuous"):
            AcavSpec.from_dict(raw)

    def test_settle_entry_tighter_than_capture_is_rejected(self, spec):
        raw = self._mutate(spec, ["pointing", "settle_entry_deg"], 0.1)
        with pytest.raises(ValueError, match="tighter than"):
            AcavSpec.from_dict(raw)

    def test_target_outside_window_is_rejected(self, spec):
        raw = copy.deepcopy(spec.raw)
        raw["targets"]["list"][0]["t_image_s"] = 9999.0
        with pytest.raises(ValueError, match="outside the"):
            AcavSpec.from_dict(raw)

    def test_duplicate_target_ids_rejected(self, spec):
        raw = copy.deepcopy(spec.raw)
        raw["targets"]["list"][1]["id"] = raw["targets"]["list"][0]["id"]
        with pytest.raises(ValueError, match="duplicate target ids"):
            AcavSpec.from_dict(raw)

    def test_sensing_too_slow_for_hold_window_is_rejected(self, spec):
        """A hold window spanning fewer than 5 sensing samples cannot be
        evaluated meaningfully -- capture would become a coin flip."""
        raw = self._mutate(spec, ["bridge", "rate_groups", "fast"], 1.0)
        raw["pointing"]["settle_hold_s"] = 2.0
        raw["scheduling"]["hold_before_image_s"] = 2.0
        with pytest.raises(ValueError, match="samples in the"):
            AcavSpec.from_dict(raw)


class TestFeasibilityTable:
    def test_table_covers_every_target_in_time_order(self, spec):
        rows = spec.feasibility_table()
        assert [r["id"] for r in rows] == ["T1", "T2", "T3", "T4", "T5",
                                           "T6", "T7"]
        times = [r["t_image_s"] for r in rows]
        assert times == sorted(times)

    def test_scenario_spans_feasible_and_marginal_regimes(self, spec):
        """DESIGN INTENT check, not a result: the target set must give
        E2 something to measure. If every target were comfortably
        feasible the study would show nothing; if none were, it would
        show only that the spacecraft is too slow."""
        rows = spec.feasibility_table()
        feasible = [r for r in rows if r["predicted_feasible"]]
        marginal = [r for r in rows if not r["predicted_feasible"]]
        assert len(feasible) >= 3, "scenario is too hard to be informative"
        assert len(marginal) >= 2, "scenario is too easy to be informative"

    def test_opposite_side_slews_are_the_largest(self, spec):
        rows = {r["id"]: r for r in spec.feasibility_table()}
        # T1 (-180 km) -> T2 (+210 km) crosses the ground track.
        assert rows["T2"]["slew_from_previous_deg"] > 40.0
