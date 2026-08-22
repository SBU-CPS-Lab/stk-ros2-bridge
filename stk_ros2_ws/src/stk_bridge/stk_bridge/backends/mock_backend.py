"""Analytic mock backend -- no STK, no licence, no Windows.

Implements the full :class:`~stk_bridge.backends.base.StkBackend` contract
against a two-body Kepler propagator and an in-memory object store, so
that:

* CI can exercise ``core/`` and ``interfaces/`` end to end without STK.
* Phases 3-5 can be developed and demonstrated before an STK install is
  available.
* The lockstep/time-sync logic can be tested deterministically, which is
  effectively impossible against a real STK where timing varies run to run.
* Bugs can be triaged as "bridge logic" versus "STK interaction" by
  swapping the backend and re-running.

It deliberately reproduces STK's awkward behaviours rather than an
idealised interface, because those are what the rest of the bridge must
cope with:

* ``set_position`` and ``add_attitude_*`` are append-only and *silently
  drop* non-monotonic samples, exactly as STK does. The mock counts the
  drops so tests can assert that ``core/actuation.py``'s guard caught
  them first.
* ``add_attitude_angular_velocity`` refuses (raises) unless at least one
  attitude sample already exists for the object, mirroring the documented
  NACK.
* Grouped data providers require a ``group_element``, and omitting it
  raises the same ``QueryError`` the COM backend raises.

Defaults model **SSS-1** in a nominal sun-synchronous LEO so that a fresh
mock scenario is immediately useful for ADCS work.
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

#: SSS-1 nominal orbit. A ~500 km sun-synchronous circular LEO, which is
#: the regime the ADCS twin is being designed for. Values are
#: placeholders for scenario bring-up, not published elements -- replace
#: with the real TLE in config/scenario_defaults.yaml.
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
    position_samples: List[Tuple[float, Tuple[float, float, float]]] = \
        field(default_factory=list)
    attitude_samples: List[Tuple[float, Tuple[float, float, float, float]]] = \
        field(default_factory=list)
    rate_samples: List[Tuple[float, Tuple[float, float, float]]] = \
        field(default_factory=list)

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

    def _attitude_at(self, obj: _MockObject, when: float
                     ) -> Tuple[float, float, float, float]:
        """Latest commanded attitude, or nadir-ish identity if none."""
        candidates = [s for s in obj.attitude_samples if s[0] <= when]
        if candidates:
            return max(candidates, key=lambda s: s[0])[1]
        return (0.0, 0.0, 0.0, 1.0)

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
            return self._columns(query, times, self._position_columns(obj))
        if name == "cartesian velocity":
            self._require_group(query, {"ICRF", "J2000", "Fixed"})
            return self._columns(query, times, self._velocity_columns(obj))
        if name in ("attitude quaternions", "attitude quaternion"):
            return self._columns(query, times, self._quaternion_columns(obj))
        if name in ("body axes angular velocity", "angular velocity"):
            return self._columns(query, times, self._rate_columns(obj))
        if name == "lla state":
            self._require_group(query, {"Fixed", "Detic"})
            return self._columns(query, times, self._lla_columns(obj))
        if name in ("lighting times", "eclipse times"):
            return self._columns(query, times, self._lighting_columns(obj))

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
                 builder) -> DataProviderData:
        columns: Dict[str, List] = {"Time": []}
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

    def _position_columns(self, obj: _MockObject):
        def build(when: float) -> Dict[str, float]:
            position, _ = self._propagate(obj, when)
            return {"x": position[0], "y": position[1], "z": position[2]}
        return build

    def _velocity_columns(self, obj: _MockObject):
        def build(when: float) -> Dict[str, float]:
            _, velocity = self._propagate(obj, when)
            return {"x": velocity[0], "y": velocity[1], "z": velocity[2]}
        return build

    def _quaternion_columns(self, obj: _MockObject):
        def build(when: float) -> Dict[str, float]:
            qx, qy, qz, qw = self._attitude_at(obj, when)
            return {"q1": qx, "q2": qy, "q3": qz, "q4": qw}
        return build

    def _rate_columns(self, obj: _MockObject):
        def build(when: float) -> Dict[str, float]:
            wx, wy, wz = self._rate_at(obj, when)
            return {"wx": wx, "wy": wy, "wz": wz}
        return build

    def _lla_columns(self, obj: _MockObject):
        def build(when: float) -> Dict[str, float]:
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

    def set_position(self, object_path: str, when: StkTime, frame: str,
                     position_m: Tuple[float, float, float],
                     velocity_mps: Optional[Tuple[float, float, float]] = None
                     ) -> None:
        obj = self._obj(object_path)
        stamp = when.epoch_seconds
        if obj.position_samples and stamp <= obj.position_samples[-1][0]:
            # Silent drop, exactly as STK does. core/actuation.py should
            # have caught this before it ever reached a backend; the
            # counter lets tests prove that it did.
            obj.dropped_positions += 1
            return
        obj.position_samples.append((stamp, position_m))

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
            obj.dropped_attitudes += 1
            return
        obj.attitude_samples.append((stamp, quaternion_xyzw))

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
        """Accept anything, echo it back, change nothing.

        Returning ACK for arbitrary commands is intentional: tests that
        exercise the ``ExecuteConnect`` service should verify plumbing,
        not that the mock reimplements the Connect command set.
        """
        return ConnectResponse(command=command, ack=True,
                               messages=[f"MOCK ACK: {command}"],
                               raw=f"MOCK ACK: {command}")

    # -- test introspection ------------------------------------------------

    def drop_counts(self, object_path: str) -> Dict[str, int]:
        obj = self._obj(object_path)
        return {
            "positions": obj.dropped_positions,
            "attitudes": obj.dropped_attitudes,
        }
