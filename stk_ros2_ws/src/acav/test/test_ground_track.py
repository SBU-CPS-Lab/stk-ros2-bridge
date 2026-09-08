"""Tests for the ground-track geodesy used to place ACAV targets.

These are offline tests of pure geometry, but they are not cosmetic.
Target placement is the one part of C2 whose failure mode is a scenario
that builds cleanly, runs cleanly, and measures the wrong thing: a
target on the wrong side of the ground track is still a valid target,
STK still computes access to it, and every downstream number is
self-consistent and meaningless.

So the tests here are weighted towards the *sign and convention* errors
rather than towards arithmetic. In particular they exercise the regime
this study actually flies -- a retrograde, near-polar, southbound
ground track, sometimes across the antimeridian -- rather than the
eastbound-from-the-equator case where a left/right swap is invisible
because both conventions happen to agree.
"""

import math

import pytest

from acav.ground_track import (
    EARTH_MEAN_RADIUS_M,
    GeoPoint,
    cross_track_offset,
    destination_point,
    great_circle_distance_m,
    initial_bearing_rad,
    normalize_longitude,
    side_of_track,
    verify_placement,
)
from acav.spec import AcavSpec

#: Ground speed of a 500 km circular orbit's sub-satellite point, m/s.
#: Used to synthesise a ground track for tests that need one.
GROUND_SPEED_MPS = 7000.0

#: Sensor footprint radius at 500 km for the 2.0 deg half-angle,
#: metres. Several tests are stated as "well inside the footprint",
#: because that -- not an abstract tolerance -- is the scale at which a
#: placement error starts changing capture outcomes.
FOOTPRINT_RADIUS_M = 500_000.0 * math.tan(math.radians(2.0))


def deg(value):
    return math.degrees(value)


def point(latitude_deg, longitude_deg):
    return GeoPoint(math.radians(latitude_deg), math.radians(longitude_deg))


def track(start, bearing_deg, times_s):
    """Sub-satellite points along one great circle at orbital speed."""
    return [
        destination_point(start, math.radians(bearing_deg),
                          GROUND_SPEED_MPS * t)
        for t in times_s
    ]


class TestNormalizeLongitude:
    def test_wraps_into_the_half_open_interval(self):
        for value_deg in (-540, -181, -180, -0.5, 0, 179.9, 180, 181, 540):
            wrapped = normalize_longitude(math.radians(value_deg))
            assert -math.pi < wrapped <= math.pi + 1e-12

    def test_both_ends_of_the_antimeridian_map_to_plus_180(self):
        """+180 and -180 are the same meridian. Mapping them to opposite
        values would make a distance computation across the dateline
        report half the globe."""
        assert normalize_longitude(math.pi) == pytest.approx(math.pi)
        assert normalize_longitude(-math.pi) == pytest.approx(math.pi)

    def test_is_idempotent(self):
        for value_deg in (-359, -95, 12.3, 179.9999, 250):
            once = normalize_longitude(math.radians(value_deg))
            assert normalize_longitude(once) == pytest.approx(once)


class TestInitialBearing:
    @pytest.mark.parametrize("target_deg,expected_deg", [
        ((10.0, 0.0), 0.0),      # due north
        ((0.0, 10.0), 90.0),     # due east
        ((-10.0, 0.0), 180.0),   # due south
        ((0.0, -10.0), 270.0),   # due west
    ])
    def test_cardinal_directions_from_the_equator(self, target_deg,
                                                  expected_deg):
        bearing = initial_bearing_rad(point(0.0, 0.0), point(*target_deg))
        assert deg(bearing) == pytest.approx(expected_deg, abs=1e-6)

    def test_eastward_across_the_antimeridian(self):
        """179.9E to 179.9W is a short hop east, not a long trip west.

        Getting this wrong flips the derived heading by 180 deg for any
        pass crossing the dateline, which silently swaps left and right
        for every target on that pass.
        """
        bearing = initial_bearing_rad(point(0.0, 179.9), point(0.0, -179.9))
        assert deg(bearing) == pytest.approx(90.0, abs=1e-6)

    def test_range_is_zero_to_two_pi(self):
        # Start deliberately off the 0/180 meridian: (5N, 0) and
        # (5S, 180) are exactly antipodal, where the bearing genuinely
        # does not exist and the function is right to refuse.
        for longitude in range(-180, 180, 17):
            bearing = initial_bearing_rad(point(5.0, 3.0),
                                          point(-5.0, longitude))
            assert 0.0 <= bearing < 2.0 * math.pi

    def test_coincident_points_raise_rather_than_returning_zero(self):
        """A zero bearing is a plausible-looking answer that would place
        every target due east of the satellite. The real cause is a
        satellite with no ephemeris, so it must be loud."""
        with pytest.raises(ValueError, match="coincident"):
            initial_bearing_rad(point(12.0, 34.0), point(12.0, 34.0))


class TestDestinationPoint:
    def test_due_north_advances_latitude_by_the_angular_distance(self):
        distance = 500_000.0
        end = destination_point(point(0.0, 0.0), 0.0, distance)
        assert end.latitude_rad == pytest.approx(
            distance / EARTH_MEAN_RADIUS_M, rel=1e-12)
        assert end.longitude_rad == pytest.approx(0.0, abs=1e-12)

    @pytest.mark.parametrize("latitude_deg", [-70.0, -20.0, 0.0, 45.0, 80.0])
    @pytest.mark.parametrize("bearing_deg", [0.0, 37.0, 90.0, 187.0, 300.0])
    def test_round_trips_through_distance_and_bearing(self, latitude_deg,
                                                      bearing_deg):
        start = point(latitude_deg, 10.0)
        distance = 240_000.0
        end = destination_point(start, math.radians(bearing_deg), distance)
        assert great_circle_distance_m(start, end) == pytest.approx(
            distance, rel=1e-9)
        measured = deg(initial_bearing_rad(start, end))
        # Compare on the circle: a due-north bearing lands either side
        # of the 0/360 seam depending on which way the last bit of
        # floating-point noise fell.
        error = (measured - bearing_deg + 180.0) % 360.0 - 180.0
        assert error == pytest.approx(0.0, abs=1e-6)

    def test_zero_distance_returns_the_origin(self):
        start = point(-33.0, 151.0)
        end = destination_point(start, 1.2, 0.0)
        assert end.latitude_rad == pytest.approx(start.latitude_rad)
        assert end.longitude_rad == pytest.approx(start.longitude_rad)

    def test_crossing_the_antimeridian_stays_in_range(self):
        end = destination_point(point(0.0, 179.9), math.radians(90.0),
                                200_000.0)
        assert -math.pi < end.longitude_rad <= math.pi
        assert end.longitude_deg < 0.0
        assert great_circle_distance_m(point(0.0, 179.9), end) == \
            pytest.approx(200_000.0, rel=1e-9)

    def test_non_positive_radius_rejected(self):
        with pytest.raises(ValueError):
            destination_point(point(0.0, 0.0), 0.0, 1000.0, radius_m=0.0)


class TestGreatCircleDistance:
    def test_symmetric_and_zero_on_itself(self):
        a, b = point(12.0, 34.0), point(-56.0, 78.0)
        assert great_circle_distance_m(a, a) == pytest.approx(0.0, abs=1e-9)
        assert great_circle_distance_m(a, b) == pytest.approx(
            great_circle_distance_m(b, a), rel=1e-12)

    def test_quarter_circumference_between_pole_and_equator(self):
        distance = great_circle_distance_m(point(90.0, 0.0), point(0.0, 0.0))
        assert distance == pytest.approx(
            0.5 * math.pi * EARTH_MEAN_RADIUS_M, rel=1e-12)


class TestCrossTrackSignConvention:
    """The convention: positive is RIGHT of the velocity vector.

    A left/right swap is the single most damaging error this module can
    make, because it produces a perfectly valid scenario. These tests
    therefore check the sign across the whole circle of headings and at
    high latitude, not just the eastbound-at-the-equator case where a
    swap can hide.
    """

    @pytest.mark.parametrize("heading_deg", [0.0, 45.0, 90.0, 135.0, 180.0,
                                             187.0, 225.0, 270.0, 315.0])
    @pytest.mark.parametrize("latitude_deg", [-60.0, -20.0, 0.0, 35.0, 75.0])
    @pytest.mark.parametrize("cross_track_km", [-240.0, -40.0, 40.0, 240.0])
    def test_side_matches_the_sign_everywhere(self, heading_deg,
                                              latitude_deg, cross_track_km):
        here, ahead = track(point(latitude_deg, 20.0), heading_deg, (0.0, 1.0))
        placed = cross_track_offset(here, ahead, cross_track_km)
        expected = 1 if cross_track_km > 0 else -1
        assert side_of_track(here, ahead, placed) == expected

    def test_right_of_a_northbound_track_is_east(self):
        here, ahead = track(point(0.0, 0.0), 0.0, (0.0, 1.0))
        placed = cross_track_offset(here, ahead, 100.0)
        assert placed.longitude_deg > 0.0

    def test_right_of_a_southbound_track_is_west(self):
        """The ACAV regime. On a descending pass of a 97.4 deg orbit the
        satellite flies roughly south, so 'right' is westward -- the
        opposite of the northbound case, and the reason the sign cannot
        be checked with a single eastbound example."""
        here, ahead = track(point(0.0, 0.0), 180.0, (0.0, 1.0))
        placed = cross_track_offset(here, ahead, 100.0)
        assert placed.longitude_deg < 0.0

    def test_sign_survives_the_antimeridian(self):
        here, ahead = track(point(20.0, 179.95), 187.0, (0.0, 1.0))
        placed = cross_track_offset(here, ahead, 200.0)
        assert side_of_track(here, ahead, placed) == 1
        assert -math.pi < placed.longitude_rad <= math.pi

    def test_magnitude_is_preserved(self):
        here, ahead = track(point(-15.0, 44.0), 187.0, (0.0, 1.0))
        for cross_track_km in (-240.0, -60.0, 40.0, 220.0):
            placed = cross_track_offset(here, ahead, cross_track_km)
            assert great_circle_distance_m(here, placed) == pytest.approx(
                abs(cross_track_km) * 1000.0, rel=1e-9)

    def test_zero_offset_stays_on_the_track(self):
        here, ahead = track(point(10.0, 10.0), 187.0, (0.0, 1.0))
        placed = cross_track_offset(here, ahead, 0.0)
        assert great_circle_distance_m(here, placed) == pytest.approx(
            0.0, abs=1e-6)
        assert side_of_track(here, ahead, placed) == 0


class TestVerifyPlacement:
    def test_accepts_a_correct_placement(self):
        here, ahead = track(point(5.0, 5.0), 187.0, (0.0, 1.0))
        placed = cross_track_offset(here, ahead, -180.0)
        ok, actual_km = verify_placement(here, ahead, placed, -180.0)
        assert ok
        assert actual_km == pytest.approx(180.0, abs=1e-6)

    def test_rejects_a_doubled_offset(self):
        """The guard has to be load-bearing: a bearing-convention error
        that offsets twice must not pass, or the builder's refusal to
        continue never fires."""
        here, ahead = track(point(5.0, 5.0), 187.0, (0.0, 1.0))
        wrong = cross_track_offset(here, ahead, -360.0)
        ok, actual_km = verify_placement(here, ahead, wrong, -180.0)
        assert not ok
        assert actual_km == pytest.approx(360.0, abs=1e-6)

    def test_distance_alone_cannot_catch_a_side_swap(self):
        """Documents why the builder checks side_of_track as well.

        A target placed on the wrong side is the correct distance from
        the ground track, so verify_placement passes it. If this ever
        starts failing, the builder's separate side check has become
        redundant -- but until then, removing it would open the exact
        hole this test describes.
        """
        here, ahead = track(point(5.0, 5.0), 187.0, (0.0, 1.0))
        mirrored = cross_track_offset(here, ahead, +180.0)
        ok, _ = verify_placement(here, ahead, mirrored, -180.0)
        assert ok
        assert side_of_track(here, ahead, mirrored) == 1


class TestPlacementIsFitForTheStudy:
    """Design-level properties, checked against the real target set."""

    @pytest.fixture(scope="class")
    def spec(self):
        return AcavSpec.load()

    def test_every_acav_offset_round_trips(self, spec):
        here, ahead = track(point(-12.0, 100.0), 187.0, (0.0, 1.0))
        for target in spec.targets:
            placed = cross_track_offset(here, ahead, target.cross_track_km)
            ok, actual_km = verify_placement(here, ahead, placed,
                                             target.cross_track_km)
            assert ok, f"{target.id} placed at {actual_km:.3f} km"
            expected_side = 1 if target.cross_track_km >= 0 else -1
            assert side_of_track(here, ahead, placed) == expected_side

    @pytest.mark.parametrize("baseline_s", [0.5, 2.0, 5.0, 10.0])
    def test_placement_is_insensitive_to_the_heading_baseline(self,
                                                              baseline_s):
        """The builder derives heading from samples 1 s apart.

        If the answer depended materially on that interval, the choice
        would be a hidden tuning parameter and target positions would
        shift whenever it changed. The tolerance is stated against the
        sensor footprint radius, since that is the scale at which a
        shift would change a capture outcome.
        """
        start = point(-12.0, 100.0)
        here, ahead_1s = track(start, 187.0, (0.0, 1.0))
        _, ahead_n = track(start, 187.0, (0.0, baseline_s))
        for cross_track_km in (-240.0, -60.0, 40.0, 220.0):
            reference = cross_track_offset(here, ahead_1s, cross_track_km)
            varied = cross_track_offset(here, ahead_n, cross_track_km)
            drift = great_circle_distance_m(reference, varied)
            assert drift < 0.01 * FOOTPRINT_RADIUS_M, (
                f"{drift:.1f} m of drift from a {baseline_s} s baseline")

    def test_a_left_right_swap_would_be_far_outside_the_footprint(self, spec):
        """Quantifies why the sign matters, so the guard is not mistaken
        for defensive boilerplate: the smallest ACAV offset still puts a
        mirrored target dozens of footprints away from the intended
        one."""
        here, ahead = track(point(-12.0, 100.0), 187.0, (0.0, 1.0))
        smallest = min(abs(t.cross_track_km) for t in spec.targets)
        correct = cross_track_offset(here, ahead, smallest)
        mirrored = cross_track_offset(here, ahead, -smallest)
        separation = great_circle_distance_m(correct, mirrored)
        # Two footprint radii is the threshold at which the intended and
        # mirrored footprints stop overlapping at all -- i.e. a swapped
        # sign cannot be partially right.
        assert separation > 2.0 * FOOTPRINT_RADIUS_M
