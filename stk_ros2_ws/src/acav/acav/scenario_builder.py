"""Builds the ACAV STK scenario from ``config/acav_scenario.yaml``.

Creates the satellite (HPOP), the body-fixed conic sensor, seven
ground-track-relative imaging targets, and a terrain-masked ground
station -- then hands off to ``verify_acav_scenario.py``, which checks
empirically that each piece does what was intended.

Design stance: verify, don't assume
-----------------------------------
Every STK-facing operation here has at least one uncertain element --
enum spellings, whether a Connect form works, whether a command that
ACKs actually had an effect. The bridge's own bring-up needed four
rounds of live-install correction for exactly this reason (see
docs/06_troubleshooting.md). So this module:

* tries multiple candidate spellings and reports which worked, rather
  than assuming one;
* records what it actually did in a :class:`BuildReport`, so the
  verifier and the paper can state what was configured rather than what
  was requested;
* refuses to continue past a check that fails in a way that would make
  the scenario silently wrong (target placement, sensor boresight).

The distinction that matters throughout: a command returning ACK means
STK parsed it, not that it had the intended effect. Terrain masking is
the clearest case -- ``SetAzElMask ... TerrainData`` ACKs happily
whether or not terrain data covers the region, so the only honest test
is to compute access twice and compare.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

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

LOGGER = logging.getLogger(__name__)

#: Separation between the two sub-satellite samples used to derive the
#: ground-track heading. One second at ~7 km/s gives ~7 km of baseline:
#: long enough that the bearing is numerically well-conditioned, short
#: enough that the track's curvature over the interval is negligible.
HEADING_SAMPLE_DT_S = 1.0


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
            self._create_scenario()
            self._create_satellite()
            if not self._report.ok:
                return self._report
            self._create_sensor()
            self._create_targets()
            if not self._report.ok:
                return self._report
            self._create_ground_station()
        except StkBridgeError as exc:
            self._report.errors.append(f"{type(exc).__name__}: {exc}")
        return self._report

    # -- scenario ----------------------------------------------------------

    def _create_scenario(self) -> None:
        spec = self._spec
        self._backend.new_scenario(spec.name)

        start = spec.window_start_utc
        stop_time = self._parse_start() + spec.window_duration_s
        stop = proto.format_utcg(stop_time)

        # Scenario time period must bracket the window, or every data
        # provider outside it returns zero rows -- the empty-result
        # failure mode documented in docs/06_troubleshooting.md.
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
        """Assign the classical elements.

        Uses the Connect ``SetState Classical`` form rather than the
        Object Model's ``AssignClassical``, because the Connect argument
        order is stable across STK versions while the Object Model's
        coordinate-system and element-representation enums are among the
        names that drifted during the bridge's bring-up. This runs once
        at setup, so the round-trip cost is irrelevant.

        The argument *order* is the uncertain part, so this issues
        candidates rather than one command, and records which was
        accepted. An ACK still proves only that STK parsed it -- whether
        the satellite ended up on the specified 500 km SSO is measured
        by ``verify_acav_scenario.py``, which reads the state back and
        derives the osculating elements. That separation is deliberate:
        this is the command most able to fail silently, because STK's
        default orbit is a perfectly plausible one to place targets
        along.
        """
        spec = self._spec
        orbit = spec.orbit
        start = proto.quote(spec.window_start_utc)
        stop = proto.quote(proto.format_utcg(
            self._parse_start() + spec.window_duration_s))
        propagator = self._report.propagator_applied or "HPOP"

        # Distances in metres and angles in radians: the backend forced
        # SI unit preferences at connect() time (see backends/base.py's
        # units contract), and Connect honours those preferences.
        semi_major_m = orbit["semi_major_axis_km"] * 1000.0
        radians = " ".join(repr(value) for value in (
            semi_major_m, orbit["eccentricity"],
            math.radians(orbit["inclination_deg"]),
            math.radians(orbit["arg_perigee_deg"]),
            math.radians(orbit["raan_deg"]),
            math.radians(orbit["true_anomaly_deg"])))
        degrees = " ".join(repr(value) for value in (
            semi_major_m, orbit["eccentricity"], orbit["inclination_deg"],
            orbit["arg_perigee_deg"], orbit["raan_deg"],
            orbit["true_anomaly_deg"]))

        def command(order: str, frame: str, elements: str) -> str:
            return (f"SetState {path} Classical {propagator} {order} "
                    f"{frame} {start} {elements}")

        step_first = f"60.0 {start} {stop}"
        span_first = f"{start} {stop} 60.0"
        candidates = [
            ("SetState Classical (step/start/stop, J2000, radians)",
             command(step_first, "J2000", radians)),
            ("SetState Classical (step/start/stop, ICRF, radians)",
             command(step_first, "ICRF", radians)),
            ("SetState Classical (start/stop/step, J2000, radians)",
             command(span_first, "J2000", radians)),
            ("SetState Classical (step/start/stop, J2000, degrees)",
             command(step_first, "J2000", degrees)),
        ]

        self._report.orbit_method = self._try_commands(
            candidates,
            failure_note=(
                "no SetState Classical form was accepted; the satellite "
                "keeps STK's default orbit, which is NOT the 500 km SSO "
                "this study specifies. Target placement will still work -- "
                "against the wrong ground track -- so the scenario will "
                "look fine and measure the wrong thing. Run "
                "verify_acav_scenario.py before trusting any result."))

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

        # Connect's Define command takes the cone angle in the session's
        # current Angle unit, which the backend set to radians.
        candidates = [
            ("Define SimpleConic (radians)",
             f"Define {path} SimpleConic "
             f"{math.radians(half_angle_deg)!r} "
             f"{math.radians(1.0)!r}"),
            ("Define SimpleConic (degrees)",
             f"Define {path} SimpleConic {half_angle_deg!r} 1.0"),
        ]
        self._report.sensor_pattern_method = self._try_commands(
            candidates,
            failure_note=(
                "could not set the sensor cone pattern; the sensor keeps "
                "STK's default pattern and the 2.0 deg half-angle this "
                "study depends on is NOT in effect"))

        # Body-fixed pointing along +Z. AzEl(0, 90) in the sensor's own
        # frame is the +Z boresight; Fixed pointing means attitude
        # determines where it looks, which is the entire point of the
        # study.
        pointing_candidates = [
            ("Point Fixed AzEl (radians)",
             f"Point {path} Fixed AzEl 0.0 {math.radians(90.0)!r} Rotate"),
            ("Point Fixed AzEl (degrees)",
             f"Point {path} Fixed AzEl 0.0 90.0 Rotate"),
        ]
        self._report.sensor_pointing_method = self._try_commands(
            pointing_candidates,
            failure_note=(
                "could not set fixed sensor pointing; if the sensor is "
                "left targeting or nadir-locked, it will NOT follow the "
                "commanded attitude and the study measures nothing"))

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
        # A zero offset is genuinely on-track, and side_of_track says so
        # with a 0. Folding that case into "right" would reject the one
        # placement whose side is unambiguous -- the ground station is
        # configured exactly this way.
        if target.cross_track_km == 0.0:
            expected_side = 0
        else:
            expected_side = 1 if target.cross_track_km > 0 else -1

        if not ok or side != expected_side:
            # A placement error yields a plausible scenario that silently
            # tests something other than what was designed -- the worst
            # possible outcome. Stop rather than continue.
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
            self._backend.execute_connect(
                f"SetPosition {path} Geodetic "
                f"{point.latitude_rad!r} {point.longitude_rad!r} 0.0")
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
        when = self._t(offset_s)
        data, mapping = dp.execute(
            self._backend, dp.LLA_STATE, satellite,
            frame=self._lla_frame(), start=when, stop=when)
        return GeoPoint(
            latitude_rad=dp.first_value(data, dp.LLA_STATE, mapping,
                                        "latitude"),
            longitude_rad=dp.first_value(data, dp.LLA_STATE, mapping,
                                         "longitude"),
            altitude_m=0.0,
        )

    def _lla_frame(self) -> str:
        """LLA State's group member.

        A live STK 12.6 inventory offers ``TrueOfDateRotating`` and
        ``Fixed`` for this provider -- notably NOT ``Detic``, which an
        earlier version of the bridge's spec assumed. ``Fixed`` is the
        Earth-fixed frame we want for a ground position.
        """
        return "Fixed"

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
            self._backend.execute_connect(
                f"SetPosition {path} Geodetic "
                f"{point.latitude_rad!r} {point.longitude_rad!r} 0.0")
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
        candidates = [
            ("SetConstraint ElevationAngle (radians)",
             f"SetConstraint {path} ElevationAngle Min "
             f"{math.radians(min_elevation_deg)!r}"),
            ("SetConstraint ElevationAngle (degrees)",
             f"SetConstraint {path} ElevationAngle Min {min_elevation_deg!r}"),
        ]
        method = self._try_commands(
            candidates,
            failure_note=(
                f"could not apply the {min_elevation_deg} deg elevation "
                f"constraint; contact windows will be optimistic"))
        LOGGER.info("elevation constraint applied via %s", method)

    def _apply_terrain_mask(self, path: str) -> None:
        """Enable terrain masking on the ground station.

        The command ACKing proves only that STK parsed it, not that
        terrain data covers this region. ``verify_acav_scenario.py``
        therefore computes access with the terrain mask and again with a
        constant mask and compares: identical results mean the terrain
        mask is not actually doing anything, whatever the ACK said.
        """
        candidates = [
            ("SetAzElMask TerrainData",
             f"SetAzElMask {path} TerrainData 1 0.0"),
            ("SetAzElMask TerrainData (no range args)",
             f"SetAzElMask {path} TerrainData"),
        ]
        self._report.terrain_mask_method = self._try_commands(
            candidates,
            failure_note=(
                "could not enable terrain masking. The scenario spec says "
                "terrain is licensed, so this is unexpected -- check that "
                "terrain data is loaded for the ground station's region "
                "(Scenario > Basic > Terrain)."))

    # -- helpers -----------------------------------------------------------

    def _try_commands(self, candidates: List[Tuple[str, str]],
                      failure_note: str) -> str:
        """Try each candidate Connect form; return the label that worked.

        Returns ``"none"`` and records a warning if all fail. Reporting
        *which* form succeeded matters: it is the difference between the
        paper describing what was configured and what was requested.
        """
        for label, command in candidates:
            try:
                response = self._backend.execute_connect(command)
            except StkBridgeError:
                continue
            if response.ack:
                LOGGER.info("applied via %s", label)
                return label

        self._report.warnings.append(
            failure_note + f" (tried: {[c[0] for c in candidates]})")
        return "none"
