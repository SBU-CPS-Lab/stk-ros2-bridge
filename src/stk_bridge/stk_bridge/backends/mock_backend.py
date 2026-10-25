"""Analytic mock backend -- no STK, no licence, no Windows.

Implements the full :class:`~stk_bridge.backends.base.StkBackend` contract
against a two-body Kepler propagator and an in-memory object store, so
that:

* tests can exercise ``core/`` and ``interfaces/`` end to end without STK;
* the lockstep and time-sync logic can be tested deterministically;
* a problem can be attributed to bridge logic or to STK by swapping the
  backend and re-running.

It reproduces STK's awkward behaviours rather than an idealised
interface, because those are what the rest of the bridge must cope with
(a mock kinder than STK lets tests pass that the real system fails):

* ``add_attitude_quaternion`` rejects non-monotonic samples, as STK 12.6
  does; ``set_position`` drops and counts them, so tests can assert that
  ``core/actuation.py``'s guard caught them first.
* Switching to the RealTime propagator discards the analytic ephemeris.
* Past the newest real-time attitude sample the basic attitude is
  reported, not the last sample.
* ``add_attitude_angular_velocity`` refuses unless an attitude sample
  already exists for the object, as the Connect command does.
* Grouped data providers require a ``group_element``.
* Arbitrary Connect commands are acknowledged without effect.
* Unplaced static objects have no position.

The defaults model a satellite named SSS-1 in a ~500 km sun-synchronous
LEO.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from stk_bridge.backends.base import (
    AccessIntervalData,
    BackendCapabilities,
    BackendInfo,
    BackendType,
    ConnectResponse,
    DataProviderData,
    DataProviderQuery,
    EclipseCondition,
    LinkBudgetData,
    ObjectInfo,
    PropagatorType,
    StkBackend,
    StkTime,
)
from stk_bridge.exceptions import (
    CommandError,
    ObjectNotFoundError,
    QueryError,
)

LOGGER = logging.getLogger(__name__)

#: Earth gravitational parameter, m^3/s^2 (EGM96/WGS84 value STK uses).
MU_EARTH = 3.986004418e14
#: Equatorial radius, m.
R_EARTH = 6378137.0

#: Default orbit: a ~500 km sun-synchronous circular LEO. Placeholder
#: elements for development, not a published orbit.
SSS1_DEFAULTS = {
    "semi_major_axis_m": R_EARTH + 500_000.0,
    "eccentricity": 0.0,
    "inclination_rad": math.radians(97.4),
    "raan_rad": math.radians(0.0),
    "arg_perigee_rad": 0.0,
    "true_anomaly_rad": 0.0,
}


@dataclass
class _MockObject:
    path: str
    object_type: str
    propagator: str = PropagatorType.TWO_BODY.value
    elements: Dict[str, float] = field(default_factory=dict)

    realtime_attitude_enabled: bool = False
    look_ahead_s: float = 3600.0
    look_behind_s: float = 3600.0
    position_samples: List[Tuple[float, Tuple[float, float, float]]] = \
        field(default_factory=list)
    #: (stamp, quaternion_xyzw, was_inertial)
    attitude_samples: List[Tuple[float, Tuple[float, float, float, float],
                                bool]] = field(default_factory=list)
    rate_samples: List[Tuple[float, Tuple[float, float, float]]] = \
        field(default_factory=list)

    #: (latitude_rad, longitude_rad, altitude_m) if the object was
    #: placed with `SetPosition ... Geodetic`. Static objects -- targets
    #: and facilities -- have no propagator, so this is the only thing
    #: that gives them a position at all.
    geodetic: Optional[Tuple[float, float, float]] = None
    #: Conic half-angle in radians once set. None means the sensor is
    #: still at STK's wide default, a state callers must be able to detect.
    cone_half_angle_rad: Optional[float] = None
    #: ("terrain"|"constant", value) once a mask is applied.
    az_el_mask: Optional[Tuple[str, float]] = None

    dropped_positions: int = 0
    dropped_attitudes: int = 0


class MockBackend(StkBackend):
    """A self-contained, deterministic stand-in for STK."""

    def __init__(self, scenario_epoch: float = 1_717_200_000.0,
                 satellite_name: str = "SSS-1"):
        super().__init__(BackendType.MOCK)
        self._epoch = scenario_epoch
        self._now = scenario_epoch
        self._satellite_name = satellite_name
        self._scenario_name = ""
        self._objects: Dict[str, _MockObject] = {}
        self._info = BackendInfo(
            backend_type=BackendType.MOCK,
            stk_version="mock-12.6",
            agi_stk_wheel_version="n/a",
        )

    # -- lifecycle ---------------------------------------------------------

    def connect(self) -> None:
        self._connected = True
        self._capabilities = BackendCapabilities(
            integration=True, astrogator=True, realtime_propagator=True,
            coverage=True, communications=True, graphics=False,
            async_connect=False, numpy_bulk_extraction=False,
        )
        LOGGER.info("mock backend connected (no STK required)")

    def disconnect(self) -> None:
        self._connected = False

    def configure_units(self) -> None:
        """No-op: the mock is SI by construction."""

    # -- scenario ----------------------------------------------------------

    def get_current_scenario_name(self) -> Optional[str]:
        """Return the current mock scenario without resetting its objects."""
        self.require_connected()
        return self._scenario_name or None

    def new_scenario(self, name: str) -> None:
        self._scenario_name = name
        self._objects.clear()
        self._now = self._epoch
        path = f"*/Satellite/{self._satellite_name}"
        self._objects[path] = _MockObject(
            path=path, object_type="Satellite",
            elements=dict(SSS1_DEFAULTS),
        )
        LOGGER.info("mock scenario %r created with %s", name, path)

    def load_scenario(self, path: str) -> None:
        self.new_scenario(path.rsplit("/", 1)[-1].rsplit(".", 1)[0] or "Mock")

    def save_scenario(self, path: Optional[str] = None) -> None:
        LOGGER.debug("mock save_scenario(%s) is a no-op", path)

    def close_scenario(self) -> None:
        self._scenario_name = ""
        self._objects.clear()

    def get_scenario_epoch(self) -> StkTime:
        return StkTime(self._epoch)

    def list_objects(self) -> List[ObjectInfo]:
        return [
            ObjectInfo(object_path=o.path, object_type=o.object_type,
                       propagator_type=o.propagator)
            for o in self._objects.values()
        ]

    def object_exists(self, object_path: str) -> bool:
        return object_path in self._objects

    def create_object(self, object_type: str, name: str,
                      parent_path: str = "") -> str:
        parent = parent_path or "*"
        path = f"{parent}/{object_type}/{name}"
        self._objects[path] = _MockObject(
            path=path, object_type=object_type,
            elements=dict(SSS1_DEFAULTS) if object_type == "Satellite" else {},
        )
        return path

    def delete_object(self, object_path: str) -> None:
        self._objects.pop(object_path, None)

    def _obj(self, object_path: str) -> _MockObject:
        try:
            return self._objects[object_path]
        except KeyError:
            raise ObjectNotFoundError(object_path) from None

    # -- time --------------------------------------------------------------

    def get_current_time(self) -> StkTime:
        return StkTime(self._now)

    def set_current_time(self, when: StkTime) -> None:
        self._now = when.epoch_seconds

    # -- propagation -------------------------------------------------------

    def _has_position_ephemeris_at(self, obj: _MockObject, when: float) -> bool:
        """Does this object have POSITION ephemeris covering ``when``?

        Reproduces STK 12.6: switching a satellite to the **RealTime orbit
        propagator discards its analytic ephemeris**. The object then has
        data only within Look-Behind..Look-Ahead of the injected samples,
        so a query at the scenario epoch, before any sample, returns zero
        rows rather than an extrapolated state.
        """
        if obj.propagator != PropagatorType.REALTIME.value:
            return True
        if not obj.position_samples:
            return False
        earliest = obj.position_samples[0][0] - obj.look_behind_s
        latest = obj.position_samples[-1][0] + obj.look_ahead_s
        return earliest <= when <= latest

    def _has_attitude_ephemeris_at(self, obj: _MockObject, when: float) -> bool:
        """Does this object have ATTITUDE data covering ``when``?

        Independent of :meth:`_has_position_ephemeris_at`: real-time orbit
        (``SetPropagator RealTime``) and real-time attitude (``SetAttitude
        Standard``) are separate STK channels. An object can have
        real-time attitude while its orbit stays on a normal propagator,
        or the reverse, and must still report attitude when it has
        received no position samples.
        """
        if not obj.realtime_attitude_enabled:
            return True
        if not obj.attitude_samples:
            return False
        earliest = obj.attitude_samples[0][0] - obj.look_behind_s
        latest = obj.attitude_samples[-1][0] + obj.look_ahead_s
        return earliest <= when <= latest

    def _propagate(self, obj: _MockObject, when: float
                   ) -> Tuple[Tuple[float, float, float],
                              Tuple[float, float, float]]:
        """Two-body Kepler state in an inertial frame.

        Circular-orbit closed form when e == 0 (which the SSS-1 default
        is); otherwise solves Kepler's equation by Newton iteration. Good
        enough to exercise the bridge; it is not, and is not meant to be,
        an orbit propagator anyone should trust for analysis.
        """
        if obj.position_samples:
            # Real-time samples take precedence, mirroring STK's
            # behaviour where injected data becomes historical truth.
            latest = max(
                (s for s in obj.position_samples if s[0] <= when),
                default=None, key=lambda s: s[0],
            )
            if latest is not None:
                return latest[1], (0.0, 0.0, 0.0)

        a = obj.elements.get("semi_major_axis_m",
                             SSS1_DEFAULTS["semi_major_axis_m"])
        e = obj.elements.get("eccentricity", 0.0)
        inc = obj.elements.get("inclination_rad",
                               SSS1_DEFAULTS["inclination_rad"])
        raan = obj.elements.get("raan_rad", 0.0)
        argp = obj.elements.get("arg_perigee_rad", 0.0)
        nu0 = obj.elements.get("true_anomaly_rad", 0.0)

        n = math.sqrt(MU_EARTH / a ** 3)
        dt = when - self._epoch
        M = nu0 + n * dt

        if e < 1e-9:
            nu = M
            r = a
        else:
            E = M
            for _ in range(12):
                delta = (E - e * math.sin(E) - M) / (1 - e * math.cos(E))
                E -= delta
                if abs(delta) < 1e-12:
                    break
            nu = 2.0 * math.atan2(
                math.sqrt(1 + e) * math.sin(E / 2),
                math.sqrt(1 - e) * math.cos(E / 2),
            )
            r = a * (1 - e * math.cos(E))

        # Perifocal -> inertial.
        x_p, y_p = r * math.cos(nu), r * math.sin(nu)
        h = math.sqrt(MU_EARTH * a * (1 - e ** 2)) if e < 1 else math.sqrt(
            MU_EARTH * a)
        vx_p = -MU_EARTH / h * math.sin(nu)
        vy_p = MU_EARTH / h * (e + math.cos(nu))

        cr, sr = math.cos(raan), math.sin(raan)
        ci, si = math.cos(inc), math.sin(inc)
        ca, sa = math.cos(argp), math.sin(argp)

        r11 = cr * ca - sr * sa * ci
        r12 = -cr * sa - sr * ca * ci
        r21 = sr * ca + cr * sa * ci
        r22 = -sr * sa + cr * ca * ci
        r31 = sa * si
        r32 = ca * si

        position = (r11 * x_p + r12 * y_p,
                    r21 * x_p + r22 * y_p,
                    r31 * x_p + r32 * y_p)
        velocity = (r11 * vx_p + r12 * vy_p,
                    r21 * vx_p + r22 * vy_p,
                    r31 * vx_p + r32 * vy_p)
        return position, velocity

    def _earth_rotation(self, when: float) -> Tuple[float, float, float, float]:
        """ECI->ECF rotation about z at ``when``. Quaternion (x,y,z,w).

        The Earth Rotation Angle (IERS 2010) at ``when`` as UTC -- no
        precession or nutation, which do not matter here. The angle is
        realistic from the first second of a scenario (not zero at its
        epoch), so that pairing an Earth-fixed attitude with an inertial
        read-back is wrong by tens of degrees offline, as it is on STK.
        """
        julian_ut1 = when / 86400.0 + 2440587.5
        turns = 0.7790572732640 + 1.00273781191135448 * (julian_ut1 - 2451545.0)
        angle = (2.0 * math.pi * turns) % (2.0 * math.pi)
        half = angle / 2.0
        return (0.0, 0.0, math.sin(half), math.cos(half))

    def _attitude_at(self, obj: _MockObject, when: float,
                     inertial: bool = True
                     ) -> Tuple[float, float, float, float]:
        """Latest commanded attitude, expressed in the requested frame.

        Rotates between inertial and central-body-fixed as needed, so
        requesting the wrong frame yields a plausible but wrong
        quaternion -- the failure mode this mock must reproduce.
        """
        candidates = [s for s in obj.attitude_samples if s[0] <= when]
        if not candidates:
            return (0.0, 0.0, 0.0, 1.0)
        stamp, quaternion, was_inertial = max(candidates, key=lambda s: s[0])
        if when - obj.attitude_samples[-1][0] > 1e-6:
            # Past the newest real-time sample STK 12.6 mostly reports the
            # object's basic attitude rather than holding the last sample,
            # so a missing command is a large jump, not a harmless hold.
            return (0.0, 0.0, 0.0, 1.0)
        if was_inertial == inertial:
            return quaternion

        from stk_bridge.core.conversions import (
            quaternion_conjugate, quaternion_multiply,
        )

        rotation = self._earth_rotation(stamp)
        if inertial:      # stored fixed, want inertial
            return quaternion_multiply(quaternion_conjugate(rotation),
                                       quaternion)
        return quaternion_multiply(rotation, quaternion)

    def _rate_at(self, obj: _MockObject, when: float
                 ) -> Tuple[float, float, float]:
        candidates = [s for s in obj.rate_samples if s[0] <= when]
        if candidates:
            return max(candidates, key=lambda s: s[0])[1]
        return (0.0, 0.0, 0.0)

    # -- sensing -----------------------------------------------------------

    def query_data_provider(self, query: DataProviderQuery) -> DataProviderData:
        obj = self._obj(query.object_path)
        times = self._sample_times(query)

        name = query.provider_name.strip().lower()
        if name == "cartesian position":
            self._require_group(query, {"ICRF", "J2000", "Fixed"})
            return self._columns(
                query, times,
                self._position_columns(obj, query.group_element),
                coverage=self._has_position_ephemeris_at)
        if name == "cartesian velocity":
            self._require_group(query, {"ICRF", "J2000", "Fixed"})
            return self._columns(
                query, times,
                self._velocity_columns(obj, query.group_element),
                coverage=self._has_position_ephemeris_at)
        if name in ("attitude quaternions", "attitude quaternion"):
            return self._columns(query, times,
                                 self._quaternion_columns(obj, inertial=True),
                                 coverage=self._has_attitude_ephemeris_at)
        if name in ("ecf attitude quaternions",
                    "attitude segment ecf quaternions"):
            return self._columns(query, times,
                                 self._quaternion_columns(obj, inertial=False),
                                 coverage=self._has_attitude_ephemeris_at)
        if name == "attitude segment quaternions":
            return self._columns(query, times, self._segment_columns(obj),
                                 coverage=self._has_attitude_ephemeris_at)
        # No "Body Axes Angular Velocity" provider: STK 12.6 has none,
        # body rates come from "Attitude Segment Quaternions". Serving
        # a provider STK lacks would let provider discovery pass here
        # and fail on STK.
        if name == "lla state":
            self._require_group(query, {"Fixed", "Detic"})
            return self._columns(query, times, self._lla_columns(obj),
                                 coverage=self._has_position_ephemeris_at)
        if name in ("lighting times", "eclipse times"):
            return self._columns(query, times, self._lighting_columns(obj),
                                 coverage=self._has_position_ephemeris_at)

        raise QueryError(
            query.provider_name, query.object_path,
            "mock backend does not implement this data provider; add it to "
            "MockBackend.query_data_provider if a test needs it",
        )

    def _require_group(self, query: DataProviderQuery,
                       members: set) -> None:
        """Reproduce the COM backend's grouped-provider requirement."""
        if not query.group_element:
            raise QueryError(
                query.provider_name, query.object_path,
                "this is a grouped data provider and no group_element was "
                f"given; available members: {sorted(members)}",
            )

    def _sample_times(self, query: DataProviderQuery) -> List[float]:
        if query.is_instantaneous:
            start = query.start.epoch_seconds if query.start else self._now
            return [start]
        start = query.start.epoch_seconds
        stop = query.stop.epoch_seconds
        step = max(query.step_s, 1e-6)
        count = int((stop - start) / step) + 1
        return [start + i * step for i in range(count)]

    def _columns(self, query: DataProviderQuery, times: List[float],
                 builder, coverage=None) -> DataProviderData:
        obj = self._obj(query.object_path)
        # Epochs outside the object's ephemeris coverage produce no rows,
        # exactly as STK does -- not an exception, just nothing.
        # `coverage` is per-provider-KIND (position vs attitude) because
        # those are independent real-time channels in STK -- see
        # _has_position_ephemeris_at / _has_attitude_ephemeris_at.
        check = coverage or self._has_position_ephemeris_at
        times = [t for t in times if check(obj, t)]
        columns: Dict[str, List] = {"Time": []}
        if not times:
            return DataProviderData(
                object_path=query.object_path,
                provider_name=query.provider_name,
                columns={}, row_count=0,
            )
        for when in times:
            columns["Time"].append(when - self._epoch)
            for key, value in builder(when).items():
                columns.setdefault(key, []).append(value)
        if query.elements:
            wanted = set(query.elements) | {"Time"}
            columns = {k: v for k, v in columns.items() if k in wanted}
        return DataProviderData(
            object_path=query.object_path,
            provider_name=query.provider_name,
            columns=columns, row_count=len(times),
        )

    def _position_columns(self, obj: _MockObject,
                          group_element: Optional[str] = None):
        def build(when: float) -> Dict[str, float]:
            self._require_positionable(obj, "Cartesian Position")
            if obj.geodetic is not None:
                fixed = _geodetic_to_ecef(*obj.geodetic)
                position = self._maybe_to_inertial(fixed, when,
                                                   group_element)
            else:
                position, _ = self._propagate(obj, when)
            return {"x": position[0], "y": position[1], "z": position[2]}
        return build

    def _velocity_columns(self, obj: _MockObject,
                          group_element: Optional[str] = None):
        def build(when: float) -> Dict[str, float]:
            self._require_positionable(obj, "Cartesian Velocity")
            if obj.geodetic is not None:
                # A fixed (Earth-rotating) point has non-zero velocity in
                # an inertial frame even though it is motionless on the
                # ground; that rotational component is not modelled here
                # -- callers needing a placed object's inertial velocity
                # should not rely on this mock for it.
                return {"x": 0.0, "y": 0.0, "z": 0.0}
            _, velocity = self._propagate(obj, when)
            return {"x": velocity[0], "y": velocity[1], "z": velocity[2]}
        return build

    def _maybe_to_inertial(self, fixed: Tuple[float, float, float],
                           when: float, group_element: Optional[str]
                           ) -> Tuple[float, float, float]:
        """Rotate a Fixed-frame vector to inertial if that was requested.

        Placed (geodetic) objects such as targets and facilities are
        static only in the Fixed frame; their inertial position changes
        as the Earth rotates. Uses the same Earth-rotation model as
        ``_attitude_at``.
        """
        if not group_element or group_element.strip().lower() == "fixed":
            return fixed
        from stk_bridge.core.conversions import (
            quaternion_conjugate, rotate_vector,
        )
        rotation = self._earth_rotation(when)
        return rotate_vector(quaternion_conjugate(rotation), fixed)

    def _quaternion_columns(self, obj: _MockObject, inertial: bool = True):
        def build(when: float) -> Dict[str, float]:
            qx, qy, qz, qw = self._attitude_at(obj, when, inertial=inertial)
            return {"q1": qx, "q2": qy, "q3": qz, "q4": qw}
        return build

    def _segment_columns(self, obj: _MockObject):
        """Quaternion + body rates together, inertial-referenced."""
        def build(when: float) -> Dict[str, float]:
            qx, qy, qz, qw = self._attitude_at(obj, when, inertial=True)
            wx, wy, wz = self._rate_at(obj, when)
            return {"q1": qx, "q2": qy, "q3": qz, "q4": qw,
                    "wx": wx, "wy": wy, "wz": wz}
        return build

    def _rate_columns(self, obj: _MockObject):
        def build(when: float) -> Dict[str, float]:
            wx, wy, wz = self._rate_at(obj, when)
            return {"wx": wx, "wy": wy, "wz": wz}
        return build

    @staticmethod
    def _require_positionable(obj: _MockObject, provider: str) -> None:
        """Refuse to invent a position for an unplaced static object.

        Without this check an unplaced target would report a propagated
        orbit, a plausible answer for the wrong object. Applies to every
        position provider, so a caller falling back from one provider to
        another gets the same refusal.
        """
        if obj.object_type == "Satellite" or obj.geodetic is not None:
            return
        raise QueryError(
            provider, obj.path,
            f"{obj.object_type} {obj.path} has no position; the mock only "
            f"propagates satellites and objects placed with "
            f"SetPosition ... Geodetic",
        )

    def _lla_columns(self, obj: _MockObject):
        def build(when: float) -> Dict[str, float]:
            if obj.geodetic is not None:
                latitude, longitude, altitude = obj.geodetic
                return {"Lat": latitude, "Lon": longitude, "Alt": altitude}
            self._require_positionable(obj, "LLA State")
            position, _ = self._propagate(obj, when)
            x, y, z = position
            r = math.sqrt(x * x + y * y + z * z)
            return {
                "Lat": math.asin(max(-1.0, min(1.0, z / r))),
                "Lon": math.atan2(y, x),
                "Alt": r - R_EARTH,
            }
        return build

    def _lighting_columns(self, obj: _MockObject):
        def build(when: float) -> Dict[str, Any]:
            position, _ = self._propagate(obj, when)
            # Crude but deterministic: sunlit on the +x hemisphere.
            sunlit = position[0] > 0.0
            return {
                "Condition": (EclipseCondition.SUNLIT.value if sunlit
                              else EclipseCondition.UMBRA.value),
            }
        return build

    def list_data_providers(self, object_path: str) -> Dict[str, List[str]]:
        """Advertise exactly the providers query_data_provider implements.

        Kept in sync with the dispatch table in :meth:`query_data_provider`,
        so that provider discovery behaves as it does on STK.
        """
        self._obj(object_path)
        frames = ["TrueOfDateRotating", "Fixed", "ICRF", "MeanOfDate",
                  "MeanOfEpoch", "TrueOfDate"]
        return {
            "Cartesian Position": list(frames),
            "Cartesian Velocity": list(frames),
            # Mirrors a real STK 12.6 inventory: the attitude providers
            # are UNGROUPED, there is no "Body Axes Angular Velocity",
            # and Lighting Times is grouped by condition.
            "Attitude Quaternions": [],
            "ECF Attitude Quaternions": [],
            "Attitude Segment Quaternions": [],
            "LLA State": ["TrueOfDateRotating", "Fixed"],
            "Lighting Times": ["Sunlight", "Penumbra", "Umbra"],
        }

    def compute_access(self, from_path: str, to_path: str,
                       start: StkTime, stop: StkTime
                       ) -> List[AccessIntervalData]:
        """Synthesise plausible periodic access windows.

        Ten-minute contacts once per orbit. Deterministic so that tests
        asserting on access handling do not flake.
        """
        self._obj(from_path)
        obj = self._objects.get(to_path)
        a = (obj.elements.get("semi_major_axis_m") if obj and obj.elements
             else SSS1_DEFAULTS["semi_major_axis_m"])
        period = 2 * math.pi * math.sqrt(a ** 3 / MU_EARTH)

        intervals: List[AccessIntervalData] = []
        t = start.epoch_seconds
        while t < stop.epoch_seconds:
            begin = t
            end = min(t + 600.0, stop.epoch_seconds)
            if end > begin:
                intervals.append(AccessIntervalData(StkTime(begin),
                                                    StkTime(end)))
            t += period
        return intervals

    def get_link_budget(self, tx_path: str, rx_path: str,
                        when: StkTime) -> LinkBudgetData:
        return LinkBudgetData(
            carrier_to_noise_db=55.0, eb_no_db=12.5, margin_db=6.5,
            received_power_dbw=-120.0, free_space_loss_db=168.0,
            link_closed=True,
        )

    # -- actuation ---------------------------------------------------------

    def set_propagator(self, object_path: str, propagator: PropagatorType,
                       look_ahead_s: float = 3600.0,
                       look_behind_s: float = 3600.0,
                       external_file: str = "") -> None:
        obj = self._obj(object_path)
        if propagator is PropagatorType.EXTERNAL and not external_file:
            raise CommandError("SetPropagator", object_path,
                               "EXTERNAL propagator requires external_file")
        obj.propagator = propagator.value
        obj.look_ahead_s = look_ahead_s
        obj.look_behind_s = look_behind_s
        if propagator is PropagatorType.REALTIME:
            # Real STK discards analytic ephemeris on this switch. Mirror
            # it, so tests see the same empty-data cliff.
            obj.position_samples.clear()

    def set_position(self, object_path: str, when: StkTime, frame: str,
                     position_m: Tuple[float, float, float],
                     velocity_mps: Optional[Tuple[float, float, float]] = None
                     ) -> None:
        obj = self._obj(object_path)
        stamp = when.epoch_seconds
        if obj.position_samples and stamp <= obj.position_samples[-1][0]:
            # Dropped and counted. core/actuation.py should catch this
            # before it reaches a backend; the counter lets tests check.
            obj.dropped_positions += 1
            return
        obj.position_samples.append((stamp, position_m))

    def set_az_el_mask(self, object_path: str, mode: str,
                       value: float = 0.0) -> str:
        if mode not in ("terrain", "none", "file"):
            raise CommandError("SetAzElMask", object_path,
                               f"unknown mask mode {mode!r}; STK has "
                               f"terrain, none and file")
        obj = self._obj(object_path)
        obj.az_el_mask = (mode, value)
        return f"mock (stored {mode})"

    def get_az_el_mask(self, object_path: str) -> Optional[str]:
        mask = self._obj(object_path).az_el_mask
        return None if mask is None else mask[0]

    def set_sensor_cone(self, object_path: str, half_angle_rad: float,
                        angular_resolution_rad: float = 0.017453) -> str:
        obj = self._obj(object_path)
        if obj.object_type != "Sensor":
            raise CommandError("SetSensorCone", object_path,
                               f"{obj.object_type} is not a sensor")
        obj.cone_half_angle_rad = half_angle_rad
        return "mock (stored)"

    def get_sensor_cone_half_angle_rad(self, object_path: str
                                       ) -> Optional[float]:
        return self._obj(object_path).cone_half_angle_rad

    def enable_realtime_attitude(self, object_path: str) -> None:
        self._obj(object_path).realtime_attitude_enabled = True

    def clear_attitude_data(self, object_path: str) -> None:
        obj = self._obj(object_path)
        obj.attitude_samples.clear()
        obj.rate_samples.clear()

    def add_attitude_quaternion(self, object_path: str, when: StkTime,
                                quaternion_xyzw: Tuple[float, float, float, float],
                                inertial: bool = False) -> None:
        obj = self._obj(object_path)
        if not obj.realtime_attitude_enabled:
            raise CommandError(
                "AddAttitude", object_path,
                "real-time attitude not enabled; call "
                "enable_realtime_attitude() first (STK NACKs otherwise)",
            )
        norm = math.sqrt(sum(c * c for c in quaternion_xyzw))
        if abs(norm - 1.0) > 1e-3:
            raise CommandError("AddAttitude", object_path,
                               f"quaternion is not unit-norm (|q|={norm:.6f})")
        stamp = when.epoch_seconds
        if obj.attitude_samples and stamp <= obj.attitude_samples[-1][0]:
            # STK 12.6 rejects (NACKs) a non-monotonic attitude sample.
            obj.dropped_attitudes += 1
            raise CommandError(
                "AddAttitude", object_path,
                f"non-monotonic sample: {stamp} <= "
                f"{obj.attitude_samples[-1][0]}",
            )
        obj.attitude_samples.append((stamp, quaternion_xyzw, inertial))

    def add_attitude_angular_velocity(
        self, object_path: str, when: StkTime,
        angular_velocity_radps: Tuple[float, float, float],
    ) -> None:
        obj = self._obj(object_path)
        if not obj.attitude_samples:
            # Mirrors the documented NACK: AngVels requires at least one
            # prior attitude data point.
            raise CommandError(
                "AddAttitude AngVels", object_path,
                "no prior attitude data point; STK requires at least one "
                "AddAttitude Quat/CBFQuat sample first",
            )
        stamp = when.epoch_seconds
        if obj.rate_samples and stamp <= obj.rate_samples[-1][0]:
            return
        obj.rate_samples.append((stamp, angular_velocity_radps))

    def load_external_ephemeris(self, object_path: str,
                                file_path: str) -> None:
        obj = self._obj(object_path)
        obj.propagator = PropagatorType.EXTERNAL.value

    # -- escape hatch ------------------------------------------------------

    def execute_connect(self, command: str,
                        timeout_s: Optional[float] = None) -> ConnectResponse:
        """Accept anything, echo it back, change almost nothing.

        Acknowledging arbitrary commands reproduces an STK behaviour: an
        ACK means the command parsed, not that it had an effect. Code that
        needs to know whether a Connect command worked must measure the
        result.

        The exception is ``SetPosition ... Geodetic``, which is
        interpreted, because static objects have no other way to acquire
        a position and their placement must be readable offline.
        """
        self._maybe_set_geodetic(command)
        return ConnectResponse(command=command, ack=True,
                               messages=[f"MOCK ACK: {command}"],
                               raw=f"MOCK ACK: {command}")

    def _maybe_set_geodetic(self, command: str) -> None:
        """Interpret `SetPosition <path> Geodetic <lat> <lon> <alt>`.

        The mock takes the values as radians and metres. STK 12.6 consumes
        degrees for this command, so callers measure the angle unit
        instead of assuming it (see ``acav/stk_queries.py``).
        """
        tokens = command.split()
        if len(tokens) < 6 or tokens[0].lower() != "setposition":
            return
        if tokens[2].lower() != "geodetic":
            return
        obj = self._objects.get(tokens[1])
        if obj is None:
            return
        try:
            values = tuple(float(token) for token in tokens[3:6])
        except ValueError:
            return
        obj.geodetic = values

    # -- test introspection ------------------------------------------------

    def drop_counts(self, object_path: str) -> Dict[str, int]:
        obj = self._obj(object_path)
        return {
            "positions": obj.dropped_positions,
            "attitudes": obj.dropped_attitudes,
        }


def _geodetic_to_ecef(latitude_rad: float, longitude_rad: float,
                      altitude_m: float) -> Tuple[float, float, float]:
    """WGS84 geodetic to Earth-fixed Cartesian, metres.

    Lets a caller that reads a placed object's Cartesian position (for
    example as a fallback from LLA State) be tested against the mock.
    """
    f = 1.0 / 298.257223563
    e2 = f * (2.0 - f)
    sin_lat = math.sin(latitude_rad)
    n = R_EARTH / math.sqrt(1.0 - e2 * sin_lat * sin_lat)
    xy = (n + altitude_m) * math.cos(latitude_rad)
    return (xy * math.cos(longitude_rad),
            xy * math.sin(longitude_rad),
            (n * (1.0 - e2) + altitude_m) * sin_lat)
