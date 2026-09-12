"""Reading positions back out of STK, and measuring the units they use.

Shared by ``scenario_builder`` and ``scenario_verifier``. Both need the
same three things, and both need them to fail informatively rather than
return ``None``.

Why this is not two lines of ``dp.execute``
-------------------------------------------
The first live run against STK 12.6 could not read a position back for
any of the seven targets. ``LLA State`` works on the satellite and
returns nothing useful on a Target, so the placement round-trip -- the
check that catches a ``SetPosition Geodetic`` argument-order or unit
error -- silently degraded to "UNVERIFIED" and the scenario shipped
with all seven targets on the wrong side of the planet.

So position reads go through a fallback chain here, and when the whole
chain fails the caller gets the object's actual data-provider inventory
in the diagnostic. That inventory is the answer to "which provider
should we have used", and getting it costs one round trip on a path
that was going to fail anyway.

Units are measured, not assumed
-------------------------------
``configure_units()`` asks STK for radians and *reports* whether the
request was accepted, but acceptance is not the same as the value in a
column being in that unit -- and the same run showed Connect commands
consuming degrees while the Angle preference was set to radians. The
scale is therefore measured against a quantity whose unit is not in
doubt: Cartesian position in the Fixed frame is metres (verified at
bring-up), and the latitude it implies can be compared against the
latitude ``LLA State`` reports.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from stk_bridge.backends.base import StkBackend, StkTime
from stk_bridge.core import data_providers as dp
from stk_bridge.exceptions import StkBridgeError

from acav.ground_track import GeoPoint

LOGGER = logging.getLogger(__name__)

#: WGS84, used only by the ECEF fallback below.
_WGS84_A = 6_378_137.0
_WGS84_F = 1.0 / 298.257223563
_WGS84_E2 = _WGS84_F * (2.0 - _WGS84_F)

#: Earth gravitational parameter, m^3/s^2 -- the value STK uses.
MU_EARTH = 3.986004418e14

#: Factor converting a reported angle to radians.
RADIANS = 1.0
DEGREES = math.pi / 180.0


@dataclass
class GeodeticRead:
    """The outcome of trying to read one object's position."""

    point: Optional[GeoPoint]
    method: str = ""
    diagnostic: str = ""

    @property
    def ok(self) -> bool:
        return self.point is not None


@dataclass
class AngleScale:
    """How to turn a reported angle into radians."""

    factor: float = RADIANS
    measured: bool = False
    detail: str = ""

    @property
    def name(self) -> str:
        if not self.measured:
            return "radians (assumed)"
        return "radians" if self.factor == RADIANS else "DEGREES"

    def to_radians(self, value: float) -> float:
        return value * self.factor


def read_state(backend: StkBackend, path: str, when: StkTime
               ) -> Tuple[Tuple[float, float, float],
                          Tuple[float, float, float]]:
    """Inertial position and velocity, metres and m/s."""
    position, pmap = dp.execute(backend, dp.CARTESIAN_POSITION, path,
                                frame="ICRF", start=when, stop=when)
    velocity, vmap = dp.execute(backend, dp.CARTESIAN_VELOCITY, path,
                                frame="ICRF", start=when, stop=when)
    return (
        tuple(dp.first_value(position, dp.CARTESIAN_POSITION, pmap, key)
              for key in ("x", "y", "z")),
        tuple(dp.first_value(velocity, dp.CARTESIAN_VELOCITY, vmap, key)
              for key in ("vx", "vy", "vz")),
    )


#: Latitude below which radians and degrees are not distinguishable in
#: practice: at the equator both interpretations read as ~0, and the
#: measurement would "confirm" whichever one it was asked about first.
_MIN_DISCRIMINATING_LATITUDE_RAD = 0.05


def measure_angle_scale(backend: StkBackend, satellite_path: str,
                        times: Sequence[StkTime]) -> AngleScale:
    """Determine whether ``LLA State`` reports radians or degrees.

    Compares the reported latitude against the latitude implied by the
    Fixed-frame Cartesian position, whose unit (metres) was verified at
    bring-up. Both describe the same point, so whichever interpretation
    of the reported number agrees is the unit in use.

    Several epochs are tried because the comparison is degenerate near
    the equator -- 0.01 rad and 0.01 deg are both "about zero", and a
    single sample at the scenario epoch happens to land there for this
    orbit. The first sample far enough from the equator to discriminate
    is the one used; if none is, that is reported rather than papered
    over with a default.
    """
    attempts: List[str] = []
    for when in times:
        try:
            cartesian, cmap = dp.execute(backend, dp.CARTESIAN_POSITION,
                                         satellite_path, frame="Fixed",
                                         start=when, stop=when)
            x, y, z = (dp.first_value(cartesian, dp.CARTESIAN_POSITION,
                                      cmap, key) for key in ("x", "y", "z"))
            lla, lmap = dp.execute(backend, dp.LLA_STATE, satellite_path,
                                   frame="Fixed", start=when, stop=when)
            reported = dp.first_value(lla, dp.LLA_STATE, lmap, "latitude")
        except StkBridgeError as exc:
            attempts.append(str(exc))
            continue

        _, geodetic_lat_rad, _ = ecef_to_geodetic(x, y, z)
        if abs(geodetic_lat_rad) < _MIN_DISCRIMINATING_LATITUDE_RAD:
            attempts.append(
                f"latitude {geodetic_lat_rad:.4f} rad is too close to the "
                f"equator to tell the units apart")
            continue

        as_radians = abs(reported - geodetic_lat_rad)
        as_degrees = abs(math.radians(reported) - geodetic_lat_rad)
        if as_radians <= as_degrees:
            return AngleScale(
                factor=RADIANS, measured=True,
                detail=f"LLA State latitude {reported:.6f} matches the "
                       f"Cartesian-derived {geodetic_lat_rad:.6f} rad: "
                       f"radians")
        return AngleScale(
            factor=DEGREES, measured=True,
            detail=f"LLA State latitude {reported:.6f} matches "
                   f"{math.degrees(geodetic_lat_rad):.6f} DEGREES, not the "
                   f"{geodetic_lat_rad:.6f} rad the SI contract promises")

    return AngleScale(
        factor=RADIANS, measured=False,
        detail="could not measure the angle unit; assuming radians per the "
               "backend's SI contract. Attempts:\n  "
               + "\n  ".join(attempts or ["no epochs supplied"]))


def read_geodetic(backend: StkBackend, path: str, when: StkTime,
                  scale: Optional[AngleScale] = None) -> GeodeticRead:
    """Read any object's ground position, trying several providers.

    Satellites answer ``LLA State``. Targets and facilities did not on
    the first live install, so the Fixed-frame Cartesian position is
    tried next and converted. When nothing works, the diagnostic carries
    the object's provider inventory, which is what a fix needs.
    """
    scale = scale or AngleScale()
    attempts: List[str] = []

    for frame in ("Fixed", "TrueOfDateRotating"):
        try:
            data, mapping = dp.execute(backend, dp.LLA_STATE, path,
                                       frame=frame, start=when, stop=when)
        except StkBridgeError as exc:
            attempts.append(f"LLA State/{frame}: {exc}")
            continue
        if data.row_count == 0:
            attempts.append(f"LLA State/{frame}: no rows")
            continue
        return GeodeticRead(
            point=GeoPoint(
                latitude_rad=scale.to_radians(dp.first_value(
                    data, dp.LLA_STATE, mapping, "latitude")),
                longitude_rad=scale.to_radians(dp.first_value(
                    data, dp.LLA_STATE, mapping, "longitude")),
                altitude_m=0.0),
            method=f"LLA State/{frame}")

    try:
        data, mapping = dp.execute(backend, dp.CARTESIAN_POSITION, path,
                                   frame="Fixed", start=when, stop=when)
        if data.row_count:
            x, y, z = (dp.first_value(data, dp.CARTESIAN_POSITION, mapping,
                                      key) for key in ("x", "y", "z"))
            longitude, latitude, altitude = ecef_to_geodetic(x, y, z)
            return GeodeticRead(
                point=GeoPoint(latitude_rad=latitude,
                               longitude_rad=longitude,
                               altitude_m=altitude),
                method="Cartesian Position/Fixed -> WGS84 geodetic")
        attempts.append("Cartesian Position/Fixed: no rows")
    except StkBridgeError as exc:
        attempts.append(f"Cartesian Position/Fixed: {exc}")

    return GeodeticRead(point=None, diagnostic="\n".join(
        attempts + [f"providers offered by {path}: "
                    f"{_provider_inventory(backend, path)}"]))


def ecef_to_geodetic(x: float, y: float, z: float
                     ) -> Tuple[float, float, float]:
    """Earth-fixed Cartesian metres to WGS84 (lon, lat, alt), radians/m.

    Bowring's method, one iteration, which is good to well under a metre
    at the surface. WGS84 rather than the spherical model
    ``ground_track`` uses on purpose: this converts a position STK
    computed on an ellipsoid, and a spherical conversion here would put
    a systematic ~20 km latitude bias into the comparison -- larger than
    the sensor footprint the tolerances are stated against.
    """
    longitude = math.atan2(y, x)
    p = math.hypot(x, y)
    if p < 1e-9:
        latitude = math.copysign(math.pi / 2.0, z)
        polar_radius = _WGS84_A * math.sqrt(1.0 - _WGS84_E2)
        return longitude, latitude, abs(z) - polar_radius

    b = _WGS84_A * (1.0 - _WGS84_F)
    theta = math.atan2(z * _WGS84_A, p * b)
    latitude = math.atan2(
        z + (_WGS84_E2 / (1.0 - _WGS84_E2)) * b * math.sin(theta) ** 3,
        p - _WGS84_E2 * _WGS84_A * math.cos(theta) ** 3)
    for _ in range(3):
        n = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * math.sin(latitude) ** 2)
        altitude = p / math.cos(latitude) - n
        latitude = math.atan2(z, p * (1.0 - _WGS84_E2 * n / (n + altitude)))
    n = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * math.sin(latitude) ** 2)
    altitude = p / math.cos(latitude) - n
    return longitude, latitude, altitude


def _provider_inventory(backend: StkBackend, path: str) -> str:
    try:
        providers = backend.list_data_providers(path)
    except StkBridgeError as exc:
        return f"(could not enumerate: {exc})"
    if not providers:
        return "(none reported)"
    return ", ".join(
        name + (f"[{'|'.join(members[:4])}]" if members else "")
        for name in sorted(providers)
        for members in [providers[name]])


def osculating_elements(position: Sequence[float],
                        velocity: Sequence[float]
                        ) -> Tuple[float, float, float]:
    """Osculating (a, e, i) from an inertial state. SI units, radians.

    Vis-viva and the angular momentum vector only -- no argument of
    perigee or true anomaly, both of which are singular for the
    near-circular orbit this study uses and neither of which any check
    here needs.
    """
    r = math.sqrt(sum(c * c for c in position))
    v2 = sum(c * c for c in velocity)
    if r <= 0.0:
        raise ValueError("degenerate position vector")

    energy = v2 / 2.0 - MU_EARTH / r
    if energy >= 0.0:
        raise ValueError(
            f"the state is not bound (specific energy {energy:.3e} J/kg); "
            f"|r| = {r:.1f} m, |v| = {math.sqrt(v2):.1f} m/s. If |r| looks "
            f"like kilometres, configure_units() did not take effect.")
    sma = -MU_EARTH / (2.0 * energy)

    hx = position[1] * velocity[2] - position[2] * velocity[1]
    hy = position[2] * velocity[0] - position[0] * velocity[2]
    hz = position[0] * velocity[1] - position[1] * velocity[0]
    h = math.sqrt(hx * hx + hy * hy + hz * hz)
    inclination = math.acos(max(-1.0, min(1.0, hz / h))) if h > 0.0 else 0.0

    rv = sum(p * v for p, v in zip(position, velocity))
    ex = ((v2 - MU_EARTH / r) * position[0] - rv * velocity[0]) / MU_EARTH
    ey = ((v2 - MU_EARTH / r) * position[1] - rv * velocity[1]) / MU_EARTH
    ez = ((v2 - MU_EARTH / r) * position[2] - rv * velocity[2]) / MU_EARTH
    eccentricity = math.sqrt(ex * ex + ey * ey + ez * ez)

    return sma, eccentricity, inclination
