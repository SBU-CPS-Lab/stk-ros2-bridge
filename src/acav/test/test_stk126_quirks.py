"""Regression tests for STK 12.6 behaviours seen on a live install.

Every test here corresponds to a specific way a live run produced a
scenario that built cleanly and measured the wrong thing.
They run against ``Stk126Double``, which reproduces those behaviours
exactly, so the compensations cannot silently rot.

The bar these set is deliberately end-to-end: not "the builder sends
degrees" but "the scenario that comes out is the one the spec asked
for, on a backend that behaves the way the real install behaved".
"""

import math

import pytest

from acav.ground_track import great_circle_distance_m
from acav.scenario_builder import AcavScenarioBuilder
from acav.scenario_verifier import (
    FAIL,
    PASS,
    SKIP,
    WARN,
    AcavScenarioVerifier,
)
from acav.spec import AcavSpec
from acav.stk_queries import (
    measure_angle_scale,
    osculating_elements,
    read_geodetic,
)
from stk126_double import Stk126Double


@pytest.fixture(scope="module")
def spec():
    return AcavSpec.load()


def build(spec, **quirks):
    backend = Stk126Double(**quirks)
    backend.connect()
    report = AcavScenarioBuilder(backend, spec).build()
    return backend, report


def treat_as_live(monkeypatch):
    """Make the verifier run its geometry checks against the double.

    The double subclasses MockBackend, so the verifier rightly skips
    geometry against it. Two tests below are *about* that geometry
    logic, and letting them pass on a SKIP would repeat the very error
    they exist to guard: a check reporting success without measuring
    anything.
    """
    monkeypatch.setattr(AcavScenarioVerifier, "is_mock",
                        property(lambda self: False))


def check(report, fragment):
    matches = [r for r in report.results if fragment in r.name]
    assert len(matches) == 1, (
        f"{fragment!r} matched {[r.name for r in matches]}")
    return matches[0]


class TestOrbitIsMeasuredNotAssumed:
    """`SetState Classical` ACKed and left i = 1.700 deg instead of 97.4."""

    def test_builder_reaches_the_specified_orbit(self, spec):
        backend, report = build(spec)
        assert report.ok, report.summary()

        position, velocity = _state(backend, report.satellite_path)
        sma_m, _, inclination_rad = osculating_elements(position, velocity)
        assert math.degrees(inclination_rad) == pytest.approx(97.4, abs=0.01)
        assert sma_m == pytest.approx(6_878_137.0, abs=1000.0)

    def test_the_degrees_form_is_the_one_accepted(self, spec):
        _, report = build(spec)
        assert "degrees" in report.orbit_method
        assert "start/stop/step" in report.orbit_method

    def test_a_command_that_acks_without_effect_is_rejected(self, spec):
        """The whole point of the post-condition.

        Here STK accepts the radians form's *syntax* but stores the
        wrong angle. Without the verify predicate the builder would take
        the ACK, stop, and hand back a 1.7 deg orbit -- which is exactly
        what happened live.
        """
        backend, report = build(spec, setstate_unit="radians")
        # With the backend now consuming radians, the degrees candidate
        # ACKs first and produces a nonsense orbit; the builder must
        # keep going rather than accept it.
        position, velocity = _state(backend, report.satellite_path)
        _, _, inclination_rad = osculating_elements(position, velocity)
        assert math.degrees(inclination_rad) == pytest.approx(97.4, abs=0.01)
        assert "radians" in report.orbit_method

    def test_an_unreachable_orbit_is_an_error_not_a_shrug(self, spec):
        backend, report = build(spec, setstate_order="never_matches")
        assert report.orbit_method == "none"
        assert any("500 km SSO" in w for w in report.warnings)
        # And the satellite is still on STK's default, which the
        # verifier must catch rather than the builder papering over.
        verification = AcavScenarioVerifier(backend, spec, report).verify()
        assert check(verification, "Orbit matches").status == FAIL


class TestGeodeticWriteUnitIsMeasured:
    """All seven targets landed on the wrong part of the planet."""

    @pytest.mark.parametrize("unit", ["degrees", "radians"])
    def test_targets_land_where_intended_in_either_unit(self, spec, unit):
        backend, report = build(spec, geodetic_unit=unit)
        assert report.ok, report.summary()
        assert report.geodetic_write_unit == unit

        for placement in report.targets:
            read = read_geodetic(backend, placement.object_path,
                                 backend.get_scenario_epoch())
            assert read.ok, read.diagnostic
            drift = great_circle_distance_m(read.point, placement.point)
            assert drift < 1000.0, (
                f"{placement.target_id} read back {drift:.0f} m from where "
                f"the builder computed it")

    def test_the_probe_is_cleaned_up(self, spec):
        """A throwaway object left behind would show up in the scenario
        the user saves and be mistaken for part of the study."""
        backend, _ = build(spec)
        assert not [p for p in backend._objects if "UnitProbe" in p]

    def test_placement_round_trip_passes_end_to_end(self, spec):
        backend, report = build(spec)
        verification = AcavScenarioVerifier(backend, spec, report).verify()
        assert check(verification, "Targets are placed").status == PASS


class TestVerifierWritesInTheMeasuredUnit:
    """The verifier placed its FOV probes in radians where STK wanted
    degrees, put all seven in the wrong place, and concluded the sensor
    could not see nadir. That was true of the probes, not the sensor."""

    def _verifier(self, spec, backend, report):
        verifier = AcavScenarioVerifier(backend, spec, report)
        verifier._epoch = backend.get_scenario_epoch()
        verifier._satellite = report.satellite_path
        return verifier

    def test_the_write_unit_comes_from_the_build_report(self, spec):
        backend, report = build(spec, geodetic_unit="degrees")
        verifier = self._verifier(spec, backend, report)
        assert verifier.geodetic_write_unit.label == "degrees"
        assert verifier.geodetic_write_unit.factor == pytest.approx(
            180.0 / math.pi)

    def test_it_is_measured_when_verifying_a_scenario_we_did_not_build(
            self, spec):
        """Standalone verification has no build report to inherit from,
        and must not fall back to assuming radians."""
        backend, report = build(spec, geodetic_unit="degrees")
        verifier = AcavScenarioVerifier(backend, spec)
        verifier._epoch = backend.get_scenario_epoch()
        verifier._satellite = report.satellite_path
        assert verifier.geodetic_write_unit.label == "degrees"

    @pytest.mark.parametrize("unit", ["degrees", "radians"])
    def test_probe_targets_land_where_intended(self, spec, unit):
        backend, report = build(spec, geodetic_unit=unit)
        verifier = self._verifier(spec, backend, report)
        epoch_offset = verifier._probe_epoch()
        expected = verifier._sub_satellite(epoch_offset)

        with verifier._probe_targets({"nadir": 0.0}) as probes:
            read = read_geodetic(backend, probes["nadir"],
                                 backend.get_scenario_epoch())
            assert read.ok, read.diagnostic
            drift = great_circle_distance_m(read.point, expected)
        assert drift < 1000.0, (
            f"nadir probe landed {drift / 1000.0:.1f} km from the "
            f"sub-satellite point")

    def test_probe_targets_are_cleaned_up(self, spec):
        backend, report = build(spec)
        verifier = self._verifier(spec, backend, report)
        with verifier._probe_targets({"nadir": 0.0, "edge": 2.0}):
            pass
        assert not [p for p in backend._objects if "Probe" in p]

    def test_the_boresight_flip_is_sent_in_degrees(self, spec):
        """It previously sent -1.5708, which STK reads as -1.57 DEG --
        the horizon. The retry answered a question nobody asked."""
        backend, report = build(spec)
        verifier = self._verifier(spec, backend, report)
        verifier._sensor = report.sensor_path
        backend.commands.clear()
        verifier._retry_with_flipped_boresight(report.targets[0].object_path,
                                               100.0)

        pointing = [c for c in backend.commands if c.startswith("Point ")]
        assert pointing, "the flip issued no Point command at all"
        assert any("-90.0" in c for c in pointing), pointing
        # And it must put the sensor back.
        assert "90.0" in pointing[-1] and "-90.0" not in pointing[-1]


class TestRejectedCommandsAreDiagnosable:
    """A live run reported three commands as 'none' with no reason."""

    def test_the_reason_for_each_rejection_is_recorded(self, spec):
        _, report = build(spec)
        assert report.attempts, "no rejection reasons captured at all"
        joined = report.summary()
        assert "Command has failed." in joined or "NACK" in joined

    def test_a_wide_cone_is_reported_not_assumed_away(self, spec):
        """`Define ... SimpleConic` failing leaves STK's 45 deg default.

        The builder must report the half-angle actually in force, not
        the one the spec asked for -- reporting 2.0 deg here is how the
        paper ends up describing a scenario that never existed.
        """
        _, report = build(spec)
        assert report.sensor_pattern_method == "none"
        assert report.sensor_cone_half_angle_deg == pytest.approx(45.0)
        assert any("default half-angle" in w for w in report.warnings)

    def test_a_working_cone_command_is_reported_as_applied(self, spec):
        _, report = build(spec, reject_define=False)
        assert report.sensor_pattern_method != "none"
        assert report.sensor_cone_half_angle_deg == pytest.approx(2.0)


class TestVerifierDoesNotOverclaim:
    """Conclusions must not outrun the measurement that supports them."""

    def test_terrain_check_skips_when_the_mask_cannot_be_applied(
            self, spec, monkeypatch):
        """A live run compared the unmasked baseline against
        itself three times and reported a verdict about terrain."""
        backend, report = build(spec)
        treat_as_live(monkeypatch)
        verification = AcavScenarioVerifier(backend, spec, report).verify()

        result = check(verification, "Terrain mask")
        assert result.status == SKIP, result.detail
        assert "never ran" in result.detail

    def test_the_elevation_check_still_measures_without_the_mask(
            self, spec, monkeypatch):
        """The constraint is a different mechanism from the mask, so a
        mask that cannot be applied must not block it -- which is
        exactly the live install's situation. It does have to flag that
        it could not control for the terrain confound."""
        backend, report = build(spec)
        treat_as_live(monkeypatch)
        verification = AcavScenarioVerifier(backend, spec, report).verify()

        result = check(verification, "Elevation constraint")
        assert result.status == PASS, result.detail
        assert "could not be disabled" in result.detail

    def test_a_terrain_horizon_above_the_minimum_hides_the_constraint(
            self, spec, monkeypatch):
        """The confound the check controls for. With terrain occluding
        to 15 deg, a 10 deg constraint changes nothing -- and reporting
        "not applied" would be a wrong diagnosis of a correct scenario.
        """
        backend, report = build(spec, reject_azelmask=False)
        treat_as_live(monkeypatch)
        # Pin the mask on so the verifier's attempt to disable it fails.
        backend.reject_azelmask = True
        backend.mask_horizon_deg = 15.0

        verification = AcavScenarioVerifier(backend, spec, report).verify()
        result = check(verification, "Elevation constraint")
        assert result.status == WARN
        assert "already occluding" in result.detail

    def test_mask_checks_measure_when_the_mask_does_work(self, spec,
                                                         monkeypatch):
        """The positive control: with a mask that applies, the same
        checks must produce a verdict rather than skipping forever."""
        backend, report = build(spec, reject_azelmask=False)
        treat_as_live(monkeypatch)
        verification = AcavScenarioVerifier(backend, spec, report).verify()

        elevation = check(verification, "Elevation constraint")
        assert elevation.status == PASS, elevation.detail
        assert "could not be disabled" not in elevation.detail
        terrain = check(verification, "Terrain mask")
        assert terrain.status == PASS, terrain.detail
        assert verification.measurements["mask_mode"] == "terrain"

    def test_a_constant_mask_is_refused_rather_than_attempted(self, spec):
        """STK has no constant-elevation mask: the type enum offers only
        None, File and TerrainData. An earlier verifier tried to use one
        as its experimental control, failed on every run, and then
        reported a terrain verdict from three identical numbers."""
        backend, _ = build(spec, reject_azelmask=False)
        with pytest.raises(Exception, match="AgEAzElMaskType|unknown mask"):
            backend.set_az_el_mask("*/Facility/GS1", "constant", 10.0)

    def test_perturbation_check_skips_when_the_orbit_is_wrong(self, spec):
        # Not gated on treat_as_live: the perturbation check reads state
        # rather than access geometry, so it runs against the mock too.
        """It reported 'unperturbed two-body' from an 82.6 m spread
        measured on a 1.7 deg orbit, where its threshold does not
        apply."""
        backend, report = build(spec, setstate_order="never_matches")
        verification = AcavScenarioVerifier(backend, spec, report).verify()
        assert check(verification, "Orbit matches").status == FAIL
        assert check(verification, "perturbations").status == SKIP


class TestAngleReadUnit:
    def test_radians_are_confirmed_against_cartesian(self, spec):
        backend, report = build(spec)
        scale = measure_angle_scale(backend, report.satellite_path,
                                     _spread(backend, spec))
        assert scale.measured
        assert scale.factor == 1.0
        verification = AcavScenarioVerifier(backend, spec, report).verify()
        assert check(verification, "Angles are reported").status == PASS

    def test_degrees_are_caught_as_a_contract_violation(self, spec):
        """If LLA State ever reports degrees, the bridge's sensing path
        is wrong by 57x and no local compensation saves the loop."""
        backend, report = build(spec, lla_unit="degrees")
        scale = measure_angle_scale(backend, report.satellite_path,
                                     _spread(backend, spec))
        assert scale.measured
        assert scale.factor == pytest.approx(math.pi / 180.0)

        verification = AcavScenarioVerifier(backend, spec, report).verify()
        result = check(verification, "Angles are reported")
        assert result.status == FAIL
        assert "DEGREES" in result.detail

    def test_the_builder_still_places_targets_correctly_in_degrees(self,
                                                                   spec):
        """Compensating locally must actually work -- a warning that the
        contract is broken is not a substitute for a correct scenario,
        because the rest of the run's diagnosis depends on it."""
        backend, report = build(spec, lla_unit="degrees")
        assert report.ok, report.summary()
        assert "DEGREES" in report.angle_read_unit
        for placement in report.targets:
            assert placement.placement_ok


def _spread(backend, spec):
    """Epochs across the window, so the unit measurement is not taken at
    the equator where the two interpretations coincide."""
    epoch = backend.get_scenario_epoch()
    return [epoch + spec.window_duration_s * fraction
            for fraction in (0.25, 0.5, 0.75, 0.0)]


def _state(backend, path):
    from acav.stk_queries import read_state
    return read_state(backend, path, backend.get_scenario_epoch())
