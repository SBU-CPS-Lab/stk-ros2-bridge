"""Builds the ACAV STK scenario from ``config/acav_scenario.yaml``.

Creates the satellite (HPOP), the body-fixed conic sensor, the
ground-track-relative imaging targets and, for the reference scenario, a
terrain-masked ground station; ``verify_acav_scenario.py`` then checks
empirically that each piece does what was intended.

Design stance: verify, don't assume
-----------------------------------
Every STK-facing operation here has an uncertain element: enum
spellings, whether a Connect form works, whether a command that ACKs had
an effect (see docs/bridge/stk_notes.md). This module therefore:

* tries multiple candidate forms and reports which worked;
* records what it actually did in a :class:`BuildReport`, so the
  verifier and any report state what was configured rather than what
  was requested;
* refuses to continue past a check that fails in a way that would make
  the scenario wrong (target placement, sensor boresight).

A command returning ACK means STK parsed it, not that it had the
intended effect. Terrain masking is the clearest case: ``SetAzElMask``
is accepted whether or not terrain data covers the region, so the only
real test is to compute access with and without the mask and compare.

The builder does not configure HPOP's force model or physical
properties: HPOP runs with STK's default mass and drag/SRP areas.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

from stk_bridge.backends.base import PropagatorType, StkBackend, StkTime
from stk_bridge.backends import connect_protocol as proto
from stk_bridge.core import conversions as conv
from stk_bridge.core import data_providers as dp
from stk_bridge.exceptions import StkBridgeError

from acav.ground_track import (
    GeoPoint,
    cross_track_offset,
    side_of_track,
    verify_placement,
)
from acav.spec import AcavSpec, TargetSpec
from acav.stk_queries import (
    AngleScale,
    GeodeticWriteUnit,
    measure_angle_scale,
    measure_geodetic_write_unit,
    osculating_elements,
    read_geodetic,
    read_state,
    set_position_geodetic,
)

LOGGER = logging.getLogger(__name__)

#: Separation between the two sub-satellite samples used to derive the
#: ground-track heading. One second at ~7 km/s gives ~7 km of baseline:
#: long enough that the bearing is numerically well-conditioned, short
#: enough that the track's curvature over the interval is negligible.
HEADING_SAMPLE_DT_S = 1.0

#: Tolerances for "the satellite is on the orbit we asked for". Wide
#: enough to absorb the osculating-vs-mean element difference at 500 km,
#: far too tight to be passed by STK's default orbit.
SMA_TOLERANCE_M = 25_000.0
INCLINATION_TOLERANCE_DEG = 1.0


@dataclass
class TargetPlacement:
    """Where a target actually ended up, and how it was checked."""

    target_id: str
    object_path: str
    point: GeoPoint
    sub_satellite: GeoPoint
    requested_cross_track_km: float
    actual_cross_track_km: float
    side: int                       # +1 right, -1 left, 0 on-track
    placement_ok: bool


@dataclass
class BuildReport:
    """What the builder actually configured, as opposed to requested."""

    scenario_name: str = ""
    satellite_path: str = ""
    propagator_applied: str = ""
    orbit_method: str = ""
    sensor_path: str = ""
    sensor_pattern_method: str = ""
    sensor_cone_half_angle_deg: float = float("nan")
    sensor_pointing_method: str = ""
    targets: List[TargetPlacement] = field(default_factory=list)
    ground_station_path: str = ""
    terrain_mask_method: str = ""
    terrain_mask_verified: Optional[bool] = None
    #: How STK reports angles back, and what unit it accepted for
    #: geodetic positions. Both are measured, not assumed: Connect consumes
    #: degrees while the Angle unit preference is radians.
    angle_read_unit: str = "not measured"
    geodetic_write_unit: str = "not measured"
    #: Every Connect form tried, with the exact reason it was rejected, so
    #: a NACK can be diagnosed from the report.
    attempts: Dict[str, List[str]] = field(default_factory=dict)
    warnings: List[str] = field(default_factory=list)
    errors: List[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.errors

    def summary(self) -> str:
        lines = [f"ACAV scenario build report: {self.scenario_name}"]
        lines.append(f"  satellite   {self.satellite_path} "
                     f"({self.propagator_applied}), "
                     f"elements via {self.orbit_method or 'not set'}")
        lines.append(f"  sensor      {self.sensor_path} "
                     f"half-angle {self.sensor_cone_half_angle_deg:.3f} deg "
                     f"via {self.sensor_pattern_method}, "
                     f"pointing via {self.sensor_pointing_method}")
        lines.append(f"  targets     {len(self.targets)}")
        for placement in self.targets:
            flag = "ok" if placement.placement_ok else "MISPLACED"
            side = {1: "right", -1: "left", 0: "on-track"}[placement.side]
            lines.append(
                f"    {placement.target_id:<3} "
                f"requested {placement.requested_cross_track_km:+7.1f} km  "
                f"actual {placement.actual_cross_track_km:7.1f} km "
                f"{side:<8} {flag}")
        terrain = self.terrain_mask_verified
        terrain_text = {True: "VERIFIED (changes access)",
                        False: "NOT EFFECTIVE (access unchanged)",
                        None: "not checked"}[terrain]
        lines.append(f"  ground stn  {self.ground_station_path} "
                     f"mask={self.terrain_mask_method} -> {terrain_text}")
        lines.append(f"  units       angles read as {self.angle_read_unit}, "
                     f"geodetic positions written as "
                     f"{self.geodetic_write_unit}")
        for label, reasons in self.attempts.items():
            lines.append(f"  rejected    {label}")
            for reason in reasons:
                lines.append(f"                {reason}")
        for warning in self.warnings:
            lines.append(f"  WARN  {warning}")
        for error in self.errors:
            lines.append(f"  ERROR {error}")
        return "\n".join(lines)


class AcavScenarioBuilder:
    """Constructs the ACAV scenario in a live (or mock) STK backend."""

    def __init__(self, backend: StkBackend, spec: AcavSpec):
        self._backend = backend
        self._spec = spec
        self._report = BuildReport(scenario_name=spec.name)
        self._epoch: Optional[StkTime] = None
        self._angle_scale = AngleScale()
        self._geodetic_write_unit = GeodeticWriteUnit()

    @property
    def report(self) -> BuildReport:
        return self._report

    # -- entry point -------------------------------------------------------

    def build(self) -> BuildReport:
        """Build the whole scenario. Returns the report; never raises for
        a *recoverable* problem -- those land in ``report.warnings``.
        Unrecoverable ones land in ``report.errors`` and stop the build,
        because continuing would produce a scenario that looks fine and
        measures the wrong thing."""
        try:
            self._close_open_scenario()
            if not self._report.ok:
                return self._report
            self._create_scenario()
            self._create_satellite()
            if not self._report.ok:
                return self._report
            # Units before anything is placed: every SetPosition below
            # depends on knowing which one STK consumes.
            self._measure_units()
            self._create_sensor()
            self._create_targets()
            if not self._report.ok:
                return self._report
            self._create_ground_station()
        except StkBridgeError as exc:
            self._report.errors.append(f"{type(exc).__name__}: {exc}")
        return self._report

    # -- scenario ----------------------------------------------------------

    def _close_open_scenario(self) -> None:
        """Close the scenario open in the attached STK, if any.

        Every scenario, reference and seeded, is named alike, so a new
        one cannot be created next to it. STK itself is never started.
        """
        name = self._backend.get_current_scenario_name()
        if not name:
            return
        self._backend.close_scenario()
        still = self._backend.get_current_scenario_name()
        if still:
            self._report.errors.append(
                f"scenario {still!r} still open after CloseScenario; "
                "close it in STK and build again")
            return
        self._report.warnings.append(f"closed the open scenario {name!r}")
        LOGGER.info("closed the open scenario %r", name)

    def _create_scenario(self) -> None:
        spec = self._spec
        self._backend.new_scenario(spec.name)

        start = spec.window_start_utc
        stop_time = self._parse_start() + spec.window_duration_s
        stop = proto.format_utcg(stop_time)

        # Scenario time period must bracket the window, or every data
        # provider outside it returns zero rows (docs/bridge/stk_notes.md).
        try:
            self._backend.execute_connect(
                f"SetAnalysisTimePeriod * {proto.quote(start)} "
                f"{proto.quote(stop)}")
        except StkBridgeError as exc:
            self._report.warnings.append(
                f"could not set the analysis time period ({exc}); the "
                f"scenario's default period may not cover the study window")

        self._epoch = self._backend.get_scenario_epoch()
        LOGGER.info("scenario %r created, epoch %s", spec.name,
                    proto.format_utcg(self._epoch))

    def _parse_start(self) -> StkTime:
        return proto.parse_utcg(self._spec.window_start_utc)

    def _t(self, offset_s: float) -> StkTime:
        """Absolute time at ``offset_s`` seconds into the window."""
        if self._epoch is None:
            raise RuntimeError("scenario not created yet")
        return self._epoch + offset_s

    # -- satellite ---------------------------------------------------------

    def _create_satellite(self) -> None:
        spec = self._spec
        path = self._backend.create_object("Satellite", spec.spacecraft_name)
        path = conv.canonical_object_path(path)
        self._report.satellite_path = path

        try:
            self._backend.set_propagator(path, PropagatorType.HPOP)
            self._report.propagator_applied = "HPOP"
        except StkBridgeError as exc:
            self._report.warnings.append(
                f"HPOP unavailable ({exc}); falling back to J4Perturbation. "
                f"The paper must then not claim HPOP truth propagation.")
            try:
                self._backend.set_propagator(path,
                                             PropagatorType.J4_PERTURBATION)
                self._report.propagator_applied = "J4Perturbation"
            except StkBridgeError as inner:
                self._report.errors.append(
                    f"could not set any analytic propagator: {inner}")
                return

        self._set_orbital_elements(path)
        self._propagate(path)

        if not self._has_ephemeris(path):
            self._report.errors.append(
                f"{path} has no ephemeris after propagation. Every data "
                f"provider will return zero rows and the whole study is "
                f"unrunnable; see docs/06_troubleshooting.md.")

    def _set_orbital_elements(self, path: str) -> None:
        """Assign the classical elements, then check they took.

        Two STK 12.6 facts shape this:

        * the accepted argument order is ``<start> <stop> <step>``, not
          the ``<step> <start> <stop>`` the Connect reference implies;
        * **the angles are consumed as degrees**, even though
          ``configure_units()`` set the Angle preference to radians:
          sending ``radians(97.4) = 1.70027`` gives a valid 1.7 deg
          inclination orbit, and nothing errors.

        Hence degrees first, and hence ``verify``: a candidate is accepted
        only if the satellite afterwards really is on the specified orbit.
        """
        spec = self._spec
        orbit = spec.orbit
        start = proto.quote(spec.window_start_utc)
        stop = proto.quote(proto.format_utcg(
            self._parse_start() + spec.window_duration_s))
        propagator = self._report.propagator_applied or "HPOP"

        semi_major_m = orbit["semi_major_axis_km"] * 1000.0
        degrees = " ".join(repr(value) for value in (
            semi_major_m, orbit["eccentricity"], orbit["inclination_deg"],
            orbit["arg_perigee_deg"], orbit["raan_deg"],
            orbit["true_anomaly_deg"]))
        radians = " ".join(repr(value) for value in (
            semi_major_m, orbit["eccentricity"],
            math.radians(orbit["inclination_deg"]),
            math.radians(orbit["arg_perigee_deg"]),
            math.radians(orbit["raan_deg"]),
            math.radians(orbit["true_anomaly_deg"])))

        def command(order: str, frame: str, elements: str) -> str:
            return (f"SetState {path} Classical {propagator} {order} "
                    f"{frame} {start} {elements}")

        span_first = f"{start} {stop} 60.0"
        step_first = f"60.0 {start} {stop}"
        candidates = [
            ("SetState Classical (start/stop/step, J2000, degrees)",
             command(span_first, "J2000", degrees)),
            ("SetState Classical (start/stop/step, ICRF, degrees)",
             command(span_first, "ICRF", degrees)),
            ("SetState Classical (start/stop/step, J2000, radians)",
             command(span_first, "J2000", radians)),
            ("SetState Classical (step/start/stop, J2000, degrees)",
             command(step_first, "J2000", degrees)),
            ("SetState Classical (step/start/stop, J2000, radians)",
             command(step_first, "J2000", radians)),
        ]

        self._report.orbit_method = self._try_commands(
            candidates,
            failure_note=(
                "no SetState Classical form produced the specified orbit; "
                "the satellite keeps whatever orbit it had, which is NOT "
                "the 500 km SSO this study specifies. Targets would be "
                "placed along the wrong ground track and the scenario "
                "would look fine while measuring the wrong thing"),
            verify=lambda: self._orbit_matches(path))

    def _orbit_matches(self, path: str) -> bool:
        """Is the satellite actually on the specified orbit now?

        Propagates first: elements assigned but not propagated leave the
        object with no ephemeris, and every data provider on it returns
        zero rows -- which would look identical to "the command did not
        work" and would reject a candidate that was in fact correct.
        """
        self._propagate(path)
        try:
            position, velocity = read_state(self._backend, path, self._t(0.0))
        except StkBridgeError as exc:
            LOGGER.debug("orbit check could not read state: %s", exc)
            return False

        sma_m, _, inclination_rad = osculating_elements(position, velocity)
        inclination_deg = math.degrees(inclination_rad)
        wanted_sma_m = self._spec.orbit["semi_major_axis_km"] * 1000.0
        wanted_inc_deg = self._spec.orbit["inclination_deg"]
        ok = (abs(sma_m - wanted_sma_m) <= SMA_TOLERANCE_M
              and abs(inclination_deg - wanted_inc_deg)
              <= INCLINATION_TOLERANCE_DEG)
        LOGGER.info("orbit after SetState: a=%.3f km i=%.3f deg -> %s",
                    sma_m / 1000.0, inclination_deg, "ok" if ok else "wrong")
        return ok

    def _propagate(self, path: str) -> None:
        propagate = getattr(self._backend, "propagate", None)
        if propagate is None:
            return
        if not propagate(path):
            self._report.warnings.append(
                f"Propagate() reported nothing to do on {path}")

    def _has_ephemeris(self, path: str) -> bool:
        try:
            data, _ = dp.execute(
                self._backend, dp.CARTESIAN_POSITION, path,
                frame="ICRF", start=self._t(0.0), stop=self._t(0.0),
                allow_empty=True)
        except StkBridgeError:
            return False
        return data.row_count > 0

    # -- sensor ------------------------------------------------------------

    def _create_sensor(self) -> None:
        """Create the body-fixed conic sensor.

        Two things must be right and neither is safe to assume:
        the cone half-angle's units, and the boresight direction. Both
        are recorded in the report and re-checked empirically by the
        verifier, which compares sensor access against a target placed
        directly at nadir.
        """
        spec = self._spec
        satellite = self._report.satellite_path
        path = self._backend.create_object("Sensor", spec.sensor.name,
                                           parent_path=satellite)
        path = conv.canonical_object_path(path)
        self._report.sensor_path = path

        half_angle_deg = spec.sensor.half_angle_deg
        self._report.sensor_cone_half_angle_deg = half_angle_deg

        # Through the backend rather than a raw Connect command: STK 12.6
        # rejects the Connect `Define ... SimpleConic` form in both angle
        # units, and only a backend may reach the Object Model to try the
        # typed path or read the cone angle back.
        try:
            self._report.sensor_pattern_method = self._backend.set_sensor_cone(
                path, math.radians(half_angle_deg), math.radians(1.0))
        except StkBridgeError as exc:
            self._report.sensor_pattern_method = "none"
            self._report.attempts["set the sensor cone pattern"] = [str(exc)]
            self._report.warnings.append(
                "could not set the sensor cone pattern; the sensor keeps "
                "STK's default half-angle and the 2.0 deg this study "
                "depends on is NOT in effect. At the default width, "
                "pointing error stops mattering and the effect under test "
                "disappears -- results from this scenario would be "
                "meaningless rather than merely imprecise.")

        applied = self._backend.get_sensor_cone_half_angle_rad(path)
        if applied is not None:
            self._report.sensor_cone_half_angle_deg = math.degrees(applied)

        # Body-fixed pointing along +Z. AzEl(0, 90) in the sensor's own
        # frame is the +Z boresight; Fixed pointing means attitude
        # determines where it looks.
        # Degrees first: STK also accepts the radians form, as an
        # elevation of 1.5708 degrees, pointing the sensor nearly along
        # its frame's horizontal without any error.
        pointing_candidates = [
            ("Point Fixed AzEl (degrees)",
             f"Point {path} Fixed AzEl 0.0 90.0 Rotate"),
            ("Point Fixed AzEl (radians)",
             f"Point {path} Fixed AzEl 0.0 {math.radians(90.0)!r} Rotate"),
            ("Point Fixed AzEl (degrees, no AboutBoresight)",
             f"Point {path} Fixed AzEl 0.0 90.0"),
        ]
        self._report.sensor_pointing_method = self._try_commands(
            pointing_candidates,
            failure_note=(
                "could not set fixed sensor pointing; if the sensor is "
                "left targeting or nadir-locked, it will NOT follow the "
                "commanded attitude and the study measures nothing"))

    # -- units -------------------------------------------------------------

    def _measure_units(self) -> None:
        """Measure how STK reports angles, and what unit it writes them in.

        ``configure_units()`` succeeding does not settle either: Connect
        commands consume degrees, and "LLA State" reports degrees unless
        Latitude and Longitude (their own unit dimensions) are set as well.
        The two directions are measured separately because neither predicts
        the other.
        """
        satellite = self._report.satellite_path
        # A spread of epochs: near the equator the two units are
        # indistinguishable, and this orbit is at latitude ~0 at the
        # scenario epoch.
        self._angle_scale = measure_angle_scale(
            self._backend, satellite,
            [self._t(self._spec.window_duration_s * fraction)
             for fraction in (0.25, 0.5, 0.75, 0.1, 0.9, 0.0)])
        self._report.angle_read_unit = self._angle_scale.name
        LOGGER.info("angle read unit: %s (%s)", self._angle_scale.name,
                    self._angle_scale.detail)
        if self._angle_scale.measured and self._angle_scale.factor != 1.0:
            self._report.warnings.append(
                "STK is reporting angles in DEGREES, not the radians the "
                "backend's SI unit contract promises: "
                f"{self._angle_scale.detail}. This builder compensates "
                "locally, but core/sensing.py does NOT -- the closed loop "
                "would read every attitude and position angle 57x too "
                "large. Fix configure_units() in the bridge before running "
                "any experiment.")

        self._geodetic_write_unit = measure_geodetic_write_unit(
            self._backend, self._t(0.0), self._angle_scale)
        self._report.geodetic_write_unit = self._geodetic_write_unit.label
        if self._geodetic_write_unit.attempts:
            self._report.attempts["SetPosition Geodetic unit probe"] = \
                list(self._geodetic_write_unit.attempts)
        if "UNDETERMINED" in self._geodetic_write_unit.label:
            self._report.warnings.append(
                "could not determine the unit SetPosition Geodetic consumes, "
                "so every target and the ground station may be placed "
                "somewhere else entirely. If they later turn out not to be "
                "within the satellite's horizon, this is why.")

    def _set_position_geodetic(self, path: str, point: GeoPoint) -> None:
        """Place a static object, in the unit STK was measured to want."""
        set_position_geodetic(self._backend, path, point,
                              self._geodetic_write_unit)

    # -- targets -----------------------------------------------------------

    def _create_targets(self) -> None:
        for target in sorted(self._spec.targets, key=lambda t: t.t_image_s):
            placement = self._create_target(target)
            if placement is None:
                return
            self._report.targets.append(placement)

    def _create_target(self, target: TargetSpec) -> Optional[TargetPlacement]:
        satellite = self._report.satellite_path

        try:
            here = self._sub_satellite_point(satellite, target.t_image_s)
            ahead = self._sub_satellite_point(
                satellite, target.t_image_s + HEADING_SAMPLE_DT_S)
        except StkBridgeError as exc:
            self._report.errors.append(
                f"could not read the sub-satellite point for {target.id} "
                f"at t={target.t_image_s}s: {exc}")
            return None

        point = cross_track_offset(here, ahead, target.cross_track_km)

        ok, actual_km = verify_placement(here, ahead, point,
                                         target.cross_track_km)
        side = side_of_track(here, ahead, point)
        # A zero offset is on-track, and side_of_track says so with a 0
        # (the reference ground station is placed this way).
        if target.cross_track_km == 0.0:
            expected_side = 0
        else:
            expected_side = 1 if target.cross_track_km > 0 else -1

        if not ok or side != expected_side:
            # A placement error yields a plausible scenario that tests
            # something other than what was designed. Stop rather than
            # continue.
            self._report.errors.append(
                f"target {target.id} was placed incorrectly: requested "
                f"{target.cross_track_km:+.1f} km, got {actual_km:.1f} km on "
                f"the {self._side_name(side)} side (expected "
                f"{self._side_name(expected_side)}). Check the "
                f"perpendicular bearing convention in "
                f"acav/ground_track.py::cross_track_offset.")
            return None

        try:
            path = self._backend.create_object("Target", target.id)
            path = conv.canonical_object_path(path)
            self._set_position_geodetic(path, point)
        except StkBridgeError as exc:
            self._report.errors.append(
                f"could not create or position target {target.id}: {exc}")
            return None

        return TargetPlacement(
            target_id=target.id, object_path=path, point=point,
            sub_satellite=here,
            requested_cross_track_km=target.cross_track_km,
            actual_cross_track_km=actual_km, side=side, placement_ok=True,
        )

    @staticmethod
    def _side_name(side: int) -> str:
        return {1: "right", -1: "left", 0: "on-track"}.get(side, "unknown")

    def _sub_satellite_point(self, satellite: str,
                             offset_s: float) -> GeoPoint:
        """Where the satellite is over, in radians.

        Goes through the shared reader so the measured angle scale is
        applied and so a failure carries the object's provider
        inventory. ``Fixed`` is the Earth-fixed group member wanted for
        a ground position; a live 12.6 inventory offers that and
        ``TrueOfDateRotating``, notably not ``Detic``.
        """
        read = read_geodetic(self._backend, satellite, self._t(offset_s),
                             self._angle_scale)
        if not read.ok:
            raise StkBridgeError(
                f"no sub-satellite point for {satellite} at "
                f"t={offset_s:.1f} s.\n{read.diagnostic}")
        return read.point

    # -- ground station ----------------------------------------------------

    def _create_ground_station(self) -> None:
        config = self._spec.ground_station
        if not config:
            return

        satellite = self._report.satellite_path
        reference_s = float(config.get("t_reference_s", 600.0))

        try:
            here = self._sub_satellite_point(satellite, reference_s)
            ahead = self._sub_satellite_point(
                satellite, reference_s + HEADING_SAMPLE_DT_S)
        except StkBridgeError as exc:
            self._report.warnings.append(
                f"could not place the ground station: {exc}")
            return

        point = cross_track_offset(
            here, ahead, float(config.get("cross_track_km", 0.0)))

        try:
            path = self._backend.create_object("Facility",
                                               str(config.get("name", "GS1")))
            path = conv.canonical_object_path(path)
            self._set_position_geodetic(path, point)
            self._report.ground_station_path = path
        except StkBridgeError as exc:
            self._report.warnings.append(
                f"could not create the ground station: {exc}")
            return

        min_elevation_deg = float(config.get("min_elevation_deg", 10.0))
        self._apply_elevation_constraint(path, min_elevation_deg)

        if bool(config.get("terrain_masking", False)):
            self._apply_terrain_mask(path)

    def _apply_elevation_constraint(self, path: str,
                                    min_elevation_deg: float) -> None:
        # Degrees first, as every angle-bearing Connect command on STK 12.6
        # consumes degrees.
        candidates = [
            ("SetConstraint ElevationAngle (degrees)",
             f"SetConstraint {path} ElevationAngle Min {min_elevation_deg!r}"),
            ("SetConstraint ElevationAngle (radians)",
             f"SetConstraint {path} ElevationAngle Min "
             f"{math.radians(min_elevation_deg)!r}"),
            ("SetConstraint MinElevationAngle (degrees)",
             f"SetConstraint {path} MinElevationAngle {min_elevation_deg!r}"),
        ]
        method = self._try_commands(
            candidates,
            failure_note=(
                f"could not apply the {min_elevation_deg} deg elevation "
                f"constraint; contact windows will be optimistic"))
        LOGGER.info("elevation constraint applied via %s", method)

    def _apply_terrain_mask(self, path: str) -> None:
        """Enable terrain masking on the ground station.

        Applying it proves only that STK accepted the request, not that
        terrain data covers this region. ``verify_acav_scenario.py``
        therefore computes access with the terrain mask and with no mask:
        identical results mean the terrain mask is not doing anything,
        whatever was accepted here.
        """
        try:
            self._report.terrain_mask_method = self._backend.set_az_el_mask(
                path, "terrain", 0.0)
        except StkBridgeError as exc:
            self._report.terrain_mask_method = "none"
            self._report.attempts["enable terrain masking"] = [str(exc)]
            self._report.warnings.append(
                "could not enable terrain masking. The study still runs; "
                "the paper must then describe the ground-station mask as a "
                "spherical-Earth elevation mask and must NOT claim terrain "
                "masking. Check that terrain data is loaded for the "
                "station's region (Scenario > Basic > Terrain).")

    # -- helpers -----------------------------------------------------------

    def _try_commands(self, candidates: List[Tuple[str, str]],
                      failure_note: str,
                      verify: Optional[Callable[[], bool]] = None) -> str:
        """Try each candidate Connect form; return the label that worked.

        Returns ``"none"`` and records a warning if all fail. Reporting
        which form succeeded is what lets a report state what was
        configured rather than what was requested.

        * **The rejection reason is recorded.** Every attempt's exception
          text lands in ``report.attempts``.
        * **``verify`` closes the loop.** An ACK means STK parsed the
          command, not that it had the intended effect (``SetState
          Classical`` ACKs radians and produces a 1.7 deg inclination).
          When a caller can cheaply measure the effect, it passes a
          predicate here and candidates are tried until one actually
          works.
        """
        reasons: List[str] = []
        for label, command in candidates:
            try:
                response = self._backend.execute_connect(command)
            except StkBridgeError as exc:
                reasons.append(f"{label}: {type(exc).__name__}: {exc}")
                continue
            if not response.ack:
                reasons.append(f"{label}: NACK {' '.join(response.messages)}")
                continue
            if verify is not None and not verify():
                reasons.append(
                    f"{label}: ACKed but did not take effect (the "
                    f"post-condition this command exists to establish is "
                    f"still not satisfied)")
                continue
            LOGGER.info("applied via %s", label)
            if reasons:
                self._report.attempts[label + " (accepted after failures)"] \
                    = reasons
            return label

        self._report.attempts[failure_note.split(";")[0][:60]] = reasons
        self._report.warnings.append(
            failure_note + " (every candidate failed; see the rejected "
            "list above for each one's reason)")
        return "none"
