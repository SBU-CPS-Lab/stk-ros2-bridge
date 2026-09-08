"""Ground-track geodesy for target placement.

Targets in this study are defined **relative to the ground track**, not
as fixed lat/lon (see docs/C1_case_study_spec.md section 5). Hardcoded
coordinates break the moment the epoch shifts: the satellite is no
longer over them, every access list comes back empty, and the failure
looks like a bridge problem rather than a scenario problem.

So C2 asks STK where the satellite actually is, derives the ground-track
heading, and offsets perpendicular by the requested cross-track
distance. This module is the geometry half of that -- pure functions, no
STK dependency, fully testable offline.

Spherical, not WGS84
--------------------
A spherical Earth is used deliberately. The offsets involved are
<= 250 km, over which the spherical/ellipsoidal difference is a few
hundred metres -- roughly 1-2% of the 17.5 km sensor footprint radius,
and far below anything the study's conclusions turn on. More
importantly, the target *position* does not need to be accurate: it
needs to be **reproducible and roughly where intended**. STK then
computes the true access geometry against wherever the target actually
ended up, so any error here is absorbed rather than propagated.

Using a geodesic library would add a dependency and imply a precision
this placement does not have and does not need.

Sign convention
---------------
``cross_track_km > 0`` places the target to the **right** of the
velocity vector (starboard), ``< 0`` to the left. This matches the
convention documented in ``config/acav_scenario.yaml``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

#: Mean Earth radius, metres. Mean rather than equatorial: the study's
#: orbit is near-polar, so the ground track spends most of its time away
#: from the equator where the equatorial radius would be the worse
#: choice.
EARTH_MEAN_RADIUS_M = 6_371_008.8


@dataclass(frozen=True)
class GeoPoint:
    """A geodetic point. Radians and metres, matching the bridge's SI
    unit contract (see backends/base.py)."""

    latitude_rad: float
    longitude_rad: float
    altitude_m: float = 0.0

    @property
    def latitude_deg(self) -> float:
        return math.degrees(self.latitude_rad)

    @property
    def longitude_deg(self) -> float:
        return math.degrees(self.longitude_rad)

    def __repr__(self) -> str:
        return (f"GeoPoint({self.latitude_deg:.5f} deg, "
                f"{self.longitude_deg:.5f} deg, {self.altitude_m:.1f} m)")


def normalize_longitude(longitude_rad: float) -> float:
    """Wrap to (-pi, pi].

    Applied to every computed longitude. Without it, a ground track
    crossing the antimeridian produces longitudes outside STK's accepted
    range, and STK rejects the target creation with an error that says
    nothing about wrapping.
    """
    wrapped = math.fmod(longitude_rad + math.pi, 2.0 * math.pi)
    if wrapped <= 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


def initial_bearing_rad(start: GeoPoint, end: GeoPoint) -> float:
    """Initial great-circle bearing from ``start`` to ``end``, radians.

    Measured clockwise from true north, in [0, 2*pi).

    Used to derive the ground-track heading from two sub-satellite
    points sampled a second apart. Deriving heading from *positions*
    rather than from the velocity vector avoids having to reconcile
    STK's velocity frame conventions with the local horizontal -- two
    LLA queries are unambiguous, and the bridge already has a validated
    LLA path.
    """
    delta_lon = end.longitude_rad - start.longitude_rad
    cos_end_lat = math.cos(end.latitude_rad)

    y = math.sin(delta_lon) * cos_end_lat
    x = (math.cos(start.latitude_rad) * math.sin(end.latitude_rad)
         - math.sin(start.latitude_rad) * cos_end_lat * math.cos(delta_lon))

    if abs(y) < 1e-15 and abs(x) < 1e-15:
        raise ValueError(
            "cannot determine a bearing between two coincident points; the "
            "ground-track samples are too close together in time, or the "
            "satellite has no ephemeris at the requested epochs"
        )

    return math.atan2(y, x) % (2.0 * math.pi)


def destination_point(origin: GeoPoint, bearing_rad: float,
                      distance_m: float,
                      radius_m: float = EARTH_MEAN_RADIUS_M) -> GeoPoint:
    """Solve the direct geodesic problem on a sphere.

    Given a start point, an initial bearing, and a surface distance,
    return the destination.
    """
    if radius_m <= 0.0:
        raise ValueError(f"radius must be > 0, got {radius_m}")

    angular = distance_m / radius_m
    sin_lat1 = math.sin(origin.latitude_rad)
    cos_lat1 = math.cos(origin.latitude_rad)
    sin_ang = math.sin(angular)
    cos_ang = math.cos(angular)

    sin_lat2 = sin_lat1 * cos_ang + cos_lat1 * sin_ang * math.cos(bearing_rad)
    sin_lat2 = max(-1.0, min(1.0, sin_lat2))
    lat2 = math.asin(sin_lat2)

    y = math.sin(bearing_rad) * sin_ang * cos_lat1
    x = cos_ang - sin_lat1 * sin_lat2
    lon2 = origin.longitude_rad + math.atan2(y, x)

    return GeoPoint(latitude_rad=lat2,
                    longitude_rad=normalize_longitude(lon2),
                    altitude_m=0.0)


def cross_track_offset(sub_satellite: GeoPoint, next_sub_satellite: GeoPoint,
                       cross_track_km: float,
                       radius_m: float = EARTH_MEAN_RADIUS_M) -> GeoPoint:
    """Place a target ``cross_track_km`` perpendicular to the ground track.

    Args:
        sub_satellite: Sub-satellite point at the target's image epoch.
        next_sub_satellite: Sub-satellite point a short time later, used
            only to establish the heading.
        cross_track_km: Signed offset. Positive is to the **right** of
            the velocity vector.

    Returns:
        The target's ground position.
    """
    heading = initial_bearing_rad(sub_satellite, next_sub_satellite)

    # Perpendicular: +90 deg (clockwise from the heading) is starboard.
    quarter_turn = math.pi / 2.0
    if cross_track_km >= 0.0:
        bearing = heading + quarter_turn
    else:
        bearing = heading - quarter_turn

    return destination_point(sub_satellite, bearing % (2.0 * math.pi),
                             abs(cross_track_km) * 1000.0, radius_m)


def great_circle_distance_m(a: GeoPoint, b: GeoPoint,
                            radius_m: float = EARTH_MEAN_RADIUS_M) -> float:
    """Surface distance between two points, metres (haversine).

    Haversine rather than the spherical law of cosines: the latter loses
    precision for the small separations this module produces when
    verifying a placement round-trip, which is exactly where it gets
    used.
    """
    delta_lat = b.latitude_rad - a.latitude_rad
    delta_lon = b.longitude_rad - a.longitude_rad
    h = (math.sin(delta_lat / 2.0) ** 2
         + math.cos(a.latitude_rad) * math.cos(b.latitude_rad)
         * math.sin(delta_lon / 2.0) ** 2)
    return 2.0 * radius_m * math.asin(math.sqrt(min(1.0, h)))


def verify_placement(sub_satellite: GeoPoint, next_sub_satellite: GeoPoint,
                     placed: GeoPoint, expected_cross_track_km: float,
                     tolerance_km: float = 1.0) -> Tuple[bool, float]:
    """Check a placed target really is the intended distance off-track.

    Returns ``(ok, actual_cross_track_km)``.

    This exists because the failure mode of getting the perpendicular
    sign or the bearing convention wrong is a target placed 2x the
    intended distance away, or on the wrong side -- both of which yield
    a *plausible* scenario that silently tests something other than what
    was designed. The builder calls this on every target and refuses to
    continue if it disagrees.
    """
    actual_m = great_circle_distance_m(sub_satellite, placed)
    actual_km = actual_m / 1000.0
    expected_km = abs(expected_cross_track_km)
    ok = abs(actual_km - expected_km) <= tolerance_km
    return ok, actual_km


def side_of_track(sub_satellite: GeoPoint, next_sub_satellite: GeoPoint,
                  point: GeoPoint) -> int:
    """Which side of the ground track a point lies on.

    Returns +1 for right of the velocity vector, -1 for left, 0 if it is
    on the track within numerical noise. Used by
    :func:`verify_placement`'s caller to confirm the *sign* as well as
    the magnitude -- distance alone cannot catch a left/right swap.
    """
    # A point coincident with the sub-satellite point has no bearing
    # from it, and initial_bearing_rad rightly refuses to invent one.
    # Here it is not an error, though: it is the on-track case this
    # function documents a 0 return for, and it is reachable in practice
    # -- the ground station is configured at cross_track_km = 0.0.
    if great_circle_distance_m(sub_satellite, point) < 1e-6:
        return 0

    heading = initial_bearing_rad(sub_satellite, next_sub_satellite)
    bearing_to_point = initial_bearing_rad(sub_satellite, point)
    relative = (bearing_to_point - heading) % (2.0 * math.pi)

    if relative < 1e-9 or abs(relative - math.pi) < 1e-9:
        return 0
    return 1 if relative < math.pi else -1
