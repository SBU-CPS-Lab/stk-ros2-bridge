"""C2 end-to-end tests: build the ACAV scenario, then verify it.

Run against the mock backend, which models no STK geometry at all. That
limits what these tests can assert -- and the limit is the point. They
check two things:

1. the *plumbing* works end to end (paths, epochs, placement
   arithmetic, report structure, CLI exit codes);
2. the guards are **load-bearing**, by breaking the scenario in each of
   the specific ways C2 is meant to catch and asserting that the
   builder refuses to continue or the verifier reports FAIL.

The second is the part worth having. A verifier that passes a correct
scenario proves very little; a verifier that fails a scenario whose
targets have been moved onto the wrong side of the ground track, or
whose orbit is not the one that was asked for, is the thing that will
save a live-STK debug cycle. Every negative test below corresponds to a
failure that would otherwise produce a complete, self-consistent, wrong
set of results.

What these tests deliberately do NOT establish: anything about STK.
Sensor cones, terrain masking and real access geometry are verified
only by ``verify_acav_scenario.py --backend com`` against the live
install.
"""

import ast
import importlib
import math
import os

import pytest

from stk_bridge.backends.mock_backend import MockBackend

from acav import build_cli, verify_cli
from acav.ground_track import GeoPoint, great_circle_distance_m
from acav.scenario_builder import AcavScenarioBuilder
from acav.scenario_verifier import (
    FAIL,
    PASS,
    SKIP,
    WARN,
    AcavScenarioVerifier,
)
from acav.spec import AcavSpec

SETUP_PY = os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "setup.py")


@pytest.fixture(scope="module")
def spec():
    return AcavSpec.load()


@pytest.fixture
def built(spec):
    """A freshly built ACAV scenario on the mock, plus its report."""
    backend = MockBackend()
    backend.connect()
    report = AcavScenarioBuilder(backend, spec).build()
    return backend, report


def statuses(report):
    return {result.name: result.status for result in report.results}


def check(report, fragment):
    """The one check whose name contains ``fragment``."""
    matches = [r for r in report.results if fragment in r.name]
    assert len(matches) == 1, (
        f"{fragment!r} matched {[r.name for r in matches]}")
    return matches[0]


# ---------------------------------------------------------------------------
# Builder
# ---------------------------------------------------------------------------

class TestBuilder:
    def test_builds_without_errors(self, built):
        _, report = built
        assert report.ok, report.summary()

    def test_creates_every_object_the_study_needs(self, built, spec):
        backend, report = built
        assert backend.object_exists(report.satellite_path)
        assert backend.object_exists(report.sensor_path)
        assert backend.object_exists(report.ground_station_path)
        assert len(report.targets) == len(spec.targets)
        for placement in report.targets:
            assert backend.object_exists(placement.object_path)

    def test_every_target_lands_on_the_requested_side(self, built):
        _, report = built
        for placement in report.targets:
            expected = 1 if placement.requested_cross_track_km >= 0 else -1
            assert placement.side == expected, placement.target_id
            assert placement.actual_cross_track_km == pytest.approx(
                abs(placement.requested_cross_track_km), abs=1.0)

    def test_records_which_command_form_was_accepted(self, built):
        """The report must describe what was *configured*, not what was
        requested -- otherwise the paper's scenario description is a
        restatement of the YAML rather than of the scenario."""
        _, report = built
        for attribute in ("orbit_method", "sensor_pattern_method",
                          "sensor_pointing_method"):
            assert getattr(report, attribute), attribute
            assert getattr(report, attribute) != "none", attribute

    def test_refuses_to_continue_past_a_misplaced_target(self, spec,
                                                         monkeypatch):
        """A target on the wrong side yields a plausible scenario that
        measures something else. The builder must stop, not warn."""
        import acav.scenario_builder as builder_module

        real_offset = builder_module.cross_track_offset

        def mirrored(here, ahead, cross_track_km, *args, **kwargs):
            return real_offset(here, ahead, -cross_track_km, *args, **kwargs)

        monkeypatch.setattr(builder_module, "cross_track_offset", mirrored)

        backend = MockBackend()
        backend.connect()
        report = AcavScenarioBuilder(backend, spec).build()

        assert not report.ok
        assert any("placed incorrectly" in error for error in report.errors)
        # Stopped at the first bad target rather than creating six more.
        assert len(report.targets) == 0

    def test_a_target_at_zero_cross_track_does_not_crash_the_builder(
            self, spec, monkeypatch):
        """The ground station is configured at cross_track_km = 0.0, and
        a target could legitimately be too. An on-track point has no
        bearing from the sub-satellite point, which used to raise out of
        the builder as a bare ValueError -- past the StkBridgeError
        handler, so the build died with a geometry traceback instead of
        a report."""
        zeroed = [
            type(target)(id=target.id, t_image_s=target.t_image_s,
                         cross_track_km=0.0, priority=target.priority)
            for target in spec.targets
        ]
        monkeypatch.setattr(spec, "targets", zeroed)

        backend = MockBackend()
        backend.connect()
        report = AcavScenarioBuilder(backend, spec).build()
        assert report.ok, report.summary()
        assert all(p.side == 0 for p in report.targets)


# ---------------------------------------------------------------------------
# Verifier: the happy path, and what it honestly cannot say
# ---------------------------------------------------------------------------

class TestVerifierOnAGoodScenario:
    @pytest.fixture
    def report(self, built, spec):
        backend, build_report = built
        return AcavScenarioVerifier(backend, spec, build_report).verify()

    def test_no_check_fails(self, report):
        assert report.ok, report.summary()

    def test_placement_round_trip_passes(self, report):
        """Read back through the backend's stored positions, not through
        the builder's own arithmetic -- this is the check that closes
        the loop over SetPosition's argument order and angle unit."""
        assert check(report, "Targets are placed").status == PASS

    def test_geometry_checks_are_skipped_not_passed(self, report):
        """The mock has no sensor cone and no terrain. Reporting PASS
        for those would put an unearned tick next to the two claims the
        paper most needs to be true."""
        for fragment in ("boresight", "half-angle", "Terrain mask",
                         "geometrically reachable", "Elevation constraint"):
            assert check(report, fragment).status == SKIP

    def test_perturbation_check_detects_an_unperturbed_propagator(self,
                                                                  report):
        """The mock propagates pure two-body motion. The check exists to
        notice exactly that, whatever the propagator calls itself -- so
        on the mock it must warn. If it ever passes here, it has stopped
        measuring anything."""
        result = check(report, "perturbations")
        assert result.status == WARN
        assert "two-body" in result.detail

    def test_serialises_for_downstream_phases(self, report):
        payload = report.to_dict()
        assert payload["ok"] is True
        assert payload["counts"][SKIP] > 0
        assert payload["measurements"]["satellite_path"]
        assert len(payload["checks"]) == len(report.results)


# ---------------------------------------------------------------------------
# Verifier: the failures it exists to catch
# ---------------------------------------------------------------------------

class TestVerifierCatchesSilentWrongness:
    """Each test breaks the scenario the way a real STK command would.

    The mutations reach into MockBackend's object store directly. That
    is deliberate: the point is to produce a scenario that is *valid*
    and *wrong*, which is precisely what a command that ACKs without
    taking effect leaves behind, and there is no public API for
    producing one on purpose.
    """

    def test_detects_a_target_moved_to_the_wrong_side(self, built, spec):
        backend, build_report = built
        target = backend._objects["*/Target/T3"]
        latitude, longitude, altitude = target.geodetic
        # Reflect the target's longitude about the sub-satellite point's:
        # roughly the same distance off-track, on the other side. This is
        # what a left/right convention error looks like from outside.
        placement = next(p for p in build_report.targets
                         if p.target_id == "T3")
        mirrored_lon = 2.0 * placement.sub_satellite.longitude_rad - longitude
        target.geodetic = (latitude, mirrored_lon, altitude)

        report = AcavScenarioVerifier(backend, spec, build_report).verify()
        result = check(report, "Targets are placed")
        assert result.status == FAIL
        assert "T3" in result.detail

    def test_detects_a_target_placed_at_the_wrong_distance(self, built, spec):
        backend, build_report = built
        target = backend._objects["*/Target/T5"]
        latitude, longitude, altitude = target.geodetic
        target.geodetic = (latitude + math.radians(1.0), longitude, altitude)

        report = AcavScenarioVerifier(backend, spec, build_report).verify()
        result = check(report, "Targets are placed")
        assert result.status == FAIL
        assert "T5" in result.detail

    def test_detects_an_unreadable_target_position(self, built, spec):
        """A target STK will not report a position for must not be
        silently scored as correctly placed."""
        backend, build_report = built
        backend._objects["*/Target/T1"].geodetic = None

        report = AcavScenarioVerifier(backend, spec, build_report).verify()
        result = check(report, "Targets are placed")
        assert result.status == WARN
        assert "T1" in result.detail

    def test_detects_the_wrong_orbit(self, built, spec):
        """SetState Classical ACKing without taking effect leaves STK's
        default orbit. Everything downstream still works, along a ground
        track that is not the study's."""
        backend, build_report = built
        satellite = backend._objects["*/Satellite/SSS-1"]
        satellite.elements["inclination_rad"] = math.radians(28.5)

        report = AcavScenarioVerifier(backend, spec, build_report).verify()
        result = check(report, "Orbit matches")
        assert result.status == FAIL
        assert "inclination" in result.detail

    def test_detects_the_wrong_altitude(self, built, spec):
        backend, build_report = built
        satellite = backend._objects["*/Satellite/SSS-1"]
        satellite.elements["semi_major_axis_m"] += 300_000.0

        report = AcavScenarioVerifier(backend, spec, build_report).verify()
        result = check(report, "Orbit matches")
        assert result.status == FAIL
        assert "semi-major axis" in result.detail

    def test_detects_a_satellite_with_no_ephemeris(self, built, spec):
        """The most common STK failure of all: an object that exists,
        answers every query, and returns zero rows."""
        backend, build_report = built
        from stk_bridge.backends.base import PropagatorType

        # Switching to the real-time propagator discards analytic
        # ephemeris -- the mock reproduces that, as real STK does it.
        backend.set_propagator(build_report.satellite_path,
                               PropagatorType.REALTIME)

        report = AcavScenarioVerifier(backend, spec, build_report).verify()
        assert check(report, "has ephemeris").status == FAIL

    def test_reports_a_missing_scenario_without_a_wall_of_noise(self, spec):
        """Verifying before building is a normal mistake. It should
        produce one actionable line, not thirteen consequential
        failures."""
        backend = MockBackend()
        backend.connect()
        backend.new_scenario("empty")
        backend.delete_object("*/Satellite/SSS-1")

        report = AcavScenarioVerifier(backend, spec).verify()
        assert not report.ok
        assert check(report, "Satellite object resolves").status == FAIL
        assert check(report, "Remaining checks").status == SKIP
        assert len(report.failures()) == 1


# ---------------------------------------------------------------------------
# Command-line wiring
# ---------------------------------------------------------------------------

class TestCommandLine:
    def test_build_then_verify_succeeds_on_the_mock(self, capsys):
        assert build_cli.main(["--backend", "mock", "--verify"]) == 0
        assert "13 checks" in capsys.readouterr().out

    def test_build_prints_the_mock_disclaimer(self, capsys):
        """Section 8 of the case-study plan commits the paper to
        reporting no mock-derived number. The banner is how a reader of
        the console output knows which run they are looking at."""
        build_cli.main(["--backend", "mock"])
        assert "No number produced here may appear in the paper" \
            in capsys.readouterr().out

    def test_verify_alone_fails_when_nothing_has_been_built(self, capsys):
        """Verifying before building must fail, and must say what to do
        about it rather than reporting a scenario full of absences."""
        assert verify_cli.main(["--backend", "mock"]) == 1
        output = capsys.readouterr().out
        assert "Satellite object resolves" in output
        assert "Build the scenario first" in output

    def test_verify_with_build_succeeds(self, capsys):
        assert verify_cli.main(["--backend", "mock", "--build"]) == 0
        assert "0 failed" in capsys.readouterr().out

    def test_json_report_is_written(self, tmp_path, capsys):
        destination = tmp_path / "report.json"
        assert verify_cli.main(["--backend", "mock", "--build",
                                "--json", str(destination)]) == 0
        capsys.readouterr()
        import json
        payload = json.loads(destination.read_text())
        assert payload["backend"] == "mock"
        assert payload["checks"]

    def test_console_scripts_resolve(self):
        """A typo in setup.py's entry points only shows up after a
        colcon build, on the Windows box, at the worst moment."""
        tree = ast.parse(open(SETUP_PY, encoding="utf-8").read())
        entries = [
            node.value
            for node in ast.walk(tree)
            if isinstance(node, ast.Constant) and isinstance(node.value, str)
            and " = " in node.value and ":" in node.value
        ]
        wanted = {"build_acav_scenario", "verify_acav_scenario"}
        found = set()
        for entry in entries:
            name, target = entry.split(" = ", 1)
            module_name, _, function_name = target.partition(":")
            module = importlib.import_module(module_name)
            assert callable(getattr(module, function_name)), entry
            found.add(name)
        assert wanted <= found, f"missing console_scripts: {wanted - found}"


def test_footprint_assumption_still_holds(spec):
    """Sanity anchor for the tolerances used across C2.

    The 1 km placement tolerance and the probe ladder in the verifier
    are only defensible while the footprint is tens of km. If the sensor
    half-angle or the altitude changes materially, those numbers need
    revisiting rather than silently becoming loose or absurd.
    """
    radius_km = spec.altitude_km * math.tan(
        math.radians(spec.sensor.half_angle_deg))
    assert 10.0 < radius_km < 40.0
    assert great_circle_distance_m(GeoPoint(0.0, 0.0),
                                   GeoPoint(0.0, 0.0)) == 0.0
