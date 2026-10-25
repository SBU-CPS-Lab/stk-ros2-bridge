"""Empirical verification of a built ACAV scenario.

``scenario_builder.py`` issues commands. This module checks what those
commands actually *did* -- which is a different question, and the one
that matters. Every STK command in the builder has at least one
uncertain element (enum spelling, argument order, units, whether the
feature is licensed for this region), and STK's failure mode is
overwhelmingly to ACK the command and produce a plausible-looking wrong
answer rather than to error.

The distinction this module is built around
-------------------------------------------
An ACK means STK parsed the command. It does not mean:

* the sensor cone is 2.0 deg (``Define ... SimpleConic`` in the wrong
  angle unit ACKs and leaves a 115 deg cone);
* the boresight is +Z (``Point ... Fixed AzEl`` ACKs whatever it is
  handed, and STK's default satellite attitude decides which body axis
  faces the ground);
* the orbit is the 500 km SSO (``SetState Classical`` with the wrong
  argument order ACKs and leaves STK's default orbit);
* targets are where they were asked to be (``SetPosition Geodetic``
  with degrees-vs-radians confusion puts them tens of thousands of km
  away, on a plausible-looking globe);
* terrain masking is doing anything (``SetAzElMask ... TerrainData``
  ACKs whether or not terrain data covers the region).

So nothing here trusts a return code. Each check *measures* the
quantity it cares about, and where a measurement is impossible it says
so rather than passing.

Reading the report
------------------
``PASS`` measured and correct. ``WARN`` measured, not what was asked
for, but the study still runs in a degraded form the paper must then
describe honestly. ``FAIL`` the scenario would silently measure
something other than what was designed -- do not run experiments
against it. ``SKIP`` could not be measured on this backend.

Mock backend
------------
Most geometric checks are skipped under the mock, because the mock does
not model sensor cones, terrain, or access geometry -- it synthesises
plausible access intervals on a timer. A mock run therefore verifies
the *plumbing* (paths, epochs, target placement arithmetic, the
report's own logic) and nothing about STK. No number produced under
``--backend mock`` may appear in the paper; ``verify_acav_scenario.py``
prints that in a banner rather than leaving it to be remembered.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

from stk_bridge.backends import connect_protocol as proto
from stk_bridge.backends.base import BackendType, StkBackend, StkTime
from stk_bridge.core import conversions as conv
from stk_bridge.core import data_providers as dp
from stk_bridge.exceptions import StkBridgeError

from acav.ground_track import (
    GeoPoint,
    cross_track_offset,
    great_circle_distance_m,
    side_of_track,
)
from acav.scenario_builder import HEADING_SAMPLE_DT_S, BuildReport
from acav.slew_model import off_nadir_angle_deg
from acav.spec import AcavSpec
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

PASS, WARN, FAIL, SKIP = "PASS", "WARN", "FAIL", "SKIP"


#: Semi-major axis tolerance, metres. HPOP reports *osculating*
#: elements, which differ from the mean elements the YAML specifies by a
#: few km at 500 km altitude. Anything inside this is "the orbit we
#: asked for"; anything outside means SetState did not take effect.
SMA_TOLERANCE_M = 25_000.0

#: Inclination tolerance, degrees. Osculating inclination oscillates by
#: a few hundredths of a degree under J2; a wrong orbit is wrong by
#: tens of degrees, so this is a wide gate that still cannot be passed
#: by STK's default (28.5 deg) orbit.
INCLINATION_TOLERANCE_DEG = 1.0

#: Below this spread in osculating semi-major axis across the window,
#: the propagator is not applying perturbations. J2 alone drives several
#: km of short-period oscillation at this altitude, so a spread of tens
#: of metres can only be an unperturbed two-body propagation.
PERTURBATION_SMA_SPREAD_M = 200.0

#: Placement tolerance, km. Wider than the builder's own 1 km check on
#: purpose: the builder compares two points it computed on a sphere,
#: while this compares a spherical placement against the geodetic
#: position STK reports for it. Those disagree by a few hundred metres
#: over these offsets, which is real and expected rather than a fault.
PLACEMENT_TOLERANCE_KM = 2.0

#: Half-width of the access window used to test whether a probe target
#: is inside the sensor cone at its design epoch. The 2.0 deg cone gives
#: a ~17.5 km footprint radius, crossed in ~4.6 s at 7.6 km/s, so +-3 s
#: brackets the crossing without admitting a neighbouring pass.
PROBE_ACCESS_HALF_WINDOW_S = 3.0

#: Off-nadir angles at which FOV probe targets are placed, as multiples
#: of the specified half-angle. The ladder brackets the true half-angle
#: from both sides rather than testing a single point, so the report can
#: state a measured interval instead of a yes/no that hides how wrong a
#: failure is.
PROBE_LADDER = (0.0, 0.5, 0.9, 1.1, 1.5, 2.2)

#: One probe far outside any plausible intended cone. If this one has
#: access the sensor is still at STK's default half-angle (45 deg), i.e.
#: the Define command did nothing at all.
PROBE_WIDE_DEG = 20.0

_PROBE_PREFIX = "ACAV_FOVProbe"


@dataclass
class CheckResult:
    """One verification check. ``detail`` carries the measurement."""

    name: str
    status: str
    detail: str = ""

    @property
    def failed(self) -> bool:
        return self.status == FAIL


@dataclass
class VerificationReport:
    """Everything the verifier measured, in the order it measured it."""

    backend_type: str = ""
    scenario_name: str = ""
    results: List[CheckResult] = field(default_factory=list)
    #: Free-form measurements downstream phases may want (C6 reports the
    #: measured sensor half-angle and terrain mode in the methodology).
    measurements: Dict[str, object] = field(default_factory=dict)

    def add(self, name: str, status: str, detail: str = "") -> CheckResult:
        result = CheckResult(name=name, status=status, detail=detail)
        self.results.append(result)
        return result

    @property
    def ok(self) -> bool:
        return not any(r.failed for r in self.results)

    def counts(self) -> Dict[str, int]:
        counts = {PASS: 0, WARN: 0, FAIL: 0, SKIP: 0}
        for result in self.results:
            counts[result.status] = counts.get(result.status, 0) + 1
        return counts

    def failures(self) -> List[CheckResult]:
        return [r for r in self.results if r.failed]

    def summary(self) -> str:
        marker = {PASS: "  ok  ", WARN: " warn ", FAIL: " FAIL ",
                  SKIP: " skip "}
        lines: List[str] = []
        for result in self.results:
            lines.append(f"[{marker[result.status]}] {result.name}")
            for line in result.detail.splitlines():
                lines.append(f"            {line}")
        counts = self.counts()
        lines.append("-" * 64)
        lines.append(
            f"{len(self.results)} checks: {counts[PASS]} passed, "
            f"{counts[WARN]} warnings, {counts[FAIL]} failed, "
            f"{counts[SKIP]} skipped")
        for result in self.failures():
            lines.append(f"  FAILED: {result.name}")
        return "\n".join(lines)

    def to_dict(self) -> Dict[str, object]:
        return {
            "backend": self.backend_type,
            "scenario": self.scenario_name,
            "ok": self.ok,
            "counts": self.counts(),
            "measurements": dict(self.measurements),
            "checks": [
                {"name": r.name, "status": r.status, "detail": r.detail}
                for r in self.results
            ],
        }


class AcavScenarioVerifier:
    """Measures whether a built ACAV scenario is the one that was designed.

    Constructed with a connected backend and the loaded spec. If the
    scenario was built in this same process, pass the
    :class:`~acav.scenario_builder.BuildReport` so the verifier can
    compare what was *requested* against what it *measures* -- a
    disagreement between the two is more informative than either alone.
    """

    def __init__(self, backend: StkBackend, spec: AcavSpec,
                 build_report: Optional[BuildReport] = None):
        self._backend = backend
        self._spec = spec
        self._build = build_report
        self._report = VerificationReport(
            backend_type=backend.backend_type.value, scenario_name=spec.name)
        self._epoch: Optional[StkTime] = None
        self._angle_scale = AngleScale()
        #: The unit SetPosition Geodetic consumes. Taken from the build
        #: report when the scenario was built in this process, measured
        #: otherwise -- but never assumed. Placing the FOV probes in
        #: radians where STK wanted degrees put every one of them in the
        #: wrong place and made the verifier report that the sensor
        #: could not see nadir, which was true of the probes and not of
        #: the sensor.
        self._geodetic_write_unit: Optional[GeodeticWriteUnit] = None
        self._last_read_diagnostic = ""
        self._mask_failure = ""
        #: Probe-ladder measurement, taken once and read by both the
        #: boresight and the half-angle check. They are two conclusions
        #: from one experiment, and running the probes twice would both
        #: cost a second set of access computations and let the two
        #: conclusions be drawn from different measurements.
        self._fov: Optional[Dict[str, object]] = None
        self._orbit_ok = False
        self._satellite = ""
        self._sensor = ""
        self._ground_station = ""

    @property
    def report(self) -> VerificationReport:
        return self._report

    @property
    def geodetic_write_unit(self) -> GeodeticWriteUnit:
        """The unit to write geodetic positions in, measured once."""
        if self._geodetic_write_unit is None:
            if self._build and self._build.geodetic_write_unit in (
                    "degrees", "radians"):
                factor = (180.0 / math.pi
                          if self._build.geodetic_write_unit == "degrees"
                          else 1.0)
                self._geodetic_write_unit = GeodeticWriteUnit(
                    factor, self._build.geodetic_write_unit)
            else:
                self._geodetic_write_unit = measure_geodetic_write_unit(
                    self._backend, self._t(0.0), self._angle_scale,
                    probe_name="ACAV_VerifyUnitProbe")
            self._report.measurements["geodetic_write_unit"] = \
                self._geodetic_write_unit.label
        return self._geodetic_write_unit

    @property
    def is_mock(self) -> bool:
        return self._backend.backend_type is BackendType.MOCK

    # -- entry point -------------------------------------------------------

    def verify(self) -> VerificationReport:
        """Run every check. Never raises: a check that blows up is a
        FAIL with the traceback text, because the next check may still
        explain why."""
        self._run("Scenario epoch and analysis period",
                  self._check_scenario_epoch)
        self._run("Satellite object resolves", self._check_satellite_exists)

        if not self._satellite:
            self._report.add(
                "Remaining checks", SKIP,
                "no satellite to check against. Build the scenario first "
                "(build_acav_scenario.py, or pass --build).")
            return self._report

        self._run("Satellite has ephemeris", self._check_ephemeris)
        self._run("Angles are reported in radians", self._check_angle_unit)
        self._run("Orbit matches the specified 500 km SSO",
                  self._check_orbit_elements)
        self._run("Propagator applies perturbations",
                  self._check_perturbations)
        self._run("Sensor object resolves", self._check_sensor_exists)
        self._run("Sensor boresight looks at the ground",
                  self._check_boresight)
        self._run("Sensor cone half-angle is 2.0 deg",
                  self._check_half_angle)
        self._run("Targets are placed where they were asked to be",
                  self._check_target_placement)
        self._run("Targets are geometrically reachable",
                  self._check_target_reachable)
        self._run("Ground station object resolves",
                  self._check_ground_station_exists)
        self._run("Elevation constraint is in effect",
                  self._check_elevation_constraint)
        self._run("Terrain mask is effective", self._check_terrain_mask)
        return self._report

    def _run(self, name: str, check: Callable[[], Tuple[str, str]]) -> None:
        try:
            status, detail = check()
        except StkBridgeError as exc:
            status, detail = FAIL, f"{type(exc).__name__}: {exc}"
        except Exception as exc:                      # noqa: BLE001
            status, detail = FAIL, (
                f"the check itself raised {type(exc).__name__}: {exc}")
        self._report.add(name, status, detail)

    def _skip_on_mock(self, what: str) -> Tuple[str, str]:
        return SKIP, (
            f"the mock backend does not model {what}; it synthesises "
            f"access intervals on a timer. Run against --backend com to "
            f"measure this.")

    # -- scenario ----------------------------------------------------------

    def _check_scenario_epoch(self) -> Tuple[str, str]:
        self._epoch = self._backend.get_scenario_epoch()
        actual = proto.format_utcg(self._epoch)
        requested = self._spec.window_start_utc
        self._report.measurements["scenario_epoch_utc"] = actual

        try:
            wanted = proto.parse_utcg(requested)
        except Exception:
            return WARN, (f"scenario epoch is {actual}; could not parse the "
                          f"configured window_start_utc {requested!r} to "
                          f"compare")

        offset = self._epoch.epoch_seconds - wanted.epoch_seconds
        if abs(offset) <= 1.0:
            return PASS, f"epoch {actual} matches window_start_utc"

        # Not fatal: every offset in this study is measured FROM the
        # scenario epoch, so the scenario is still internally consistent
        # -- it is simply not at the epoch the spec names. That matters
        # for reproducibility across machines and for the paper's
        # scenario description, so it must be reported.
        return WARN, (
            f"scenario epoch is {actual}, {offset:+.1f} s from the "
            f"configured window_start_utc {requested}. SetAnalysisTimePeriod "
            f"did not take effect. The scenario is self-consistent (all "
            f"target epochs are offsets from the scenario epoch) but is not "
            f"the epoch the spec names, so runs are not reproducible against "
            f"a differently-initialised STK session.")

    def _check_satellite_exists(self) -> Tuple[str, str]:
        path, how = self._resolve_object(
            "Satellite", self._spec.spacecraft_name,
            self._build.satellite_path if self._build else "")
        if not path:
            return FAIL, (
                f"no Satellite named {self._spec.spacecraft_name!r} in the "
                f"loaded scenario. Objects present: "
                f"{self._object_inventory()}")
        self._satellite = path
        self._report.measurements["satellite_path"] = path
        return PASS, f"{path} (resolved via {how})"

    def _check_ephemeris(self) -> Tuple[str, str]:
        """Both ends of the window, not just one.

        A satellite whose analysis period is shorter than the study
        window returns data at t=0 and nothing at t=720, which would
        surface later as targets that mysteriously stop being visible
        halfway through a run.
        """
        rows = []
        for offset in (0.0, self._spec.window_duration_s):
            data, _ = dp.execute(self._backend, dp.CARTESIAN_POSITION,
                                 self._satellite, frame="ICRF",
                                 start=self._t(offset), stop=self._t(offset),
                                 allow_empty=True)
            rows.append((offset, data.row_count))

        empty = [offset for offset, count in rows if count == 0]
        if empty:
            return FAIL, (
                f"Cartesian Position returned no rows at t="
                f"{', '.join(f'{o:.0f}' for o in empty)} s. The satellite "
                f"has no ephemeris there: either Propagate() was never "
                f"called, or the scenario's analysis period does not cover "
                f"the {self._spec.window_duration_s:.0f} s study window. "
                f"Every data provider on this object will be silently empty.")
        return PASS, ("ephemeris present at both ends of the "
                      f"{self._spec.window_duration_s:.0f} s window")

    def _check_angle_unit(self) -> Tuple[str, str]:
        """Does STK report angles in the radians the SI contract promises?

        Measured against the Fixed-frame Cartesian position, whose unit
        is not in doubt. This is a bridge-level contract, not a
        case-study one: if angles come back in degrees then
        core/sensing.py hands the twin attitudes and positions 57x too
        large, and no amount of correction inside the case study fixes
        the closed loop.
        """
        self._angle_scale = measure_angle_scale(
            self._backend, self._satellite,
            [self._t(self._spec.window_duration_s * fraction)
             for fraction in (0.25, 0.5, 0.75, 0.1, 0.9, 0.0)])
        self._report.measurements["angle_read_unit"] = self._angle_scale.name
        if not self._angle_scale.measured:
            return WARN, self._angle_scale.detail
        if self._angle_scale.factor == 1.0:
            return PASS, self._angle_scale.detail
        return FAIL, (
            self._angle_scale.detail + "\nThe backend's SI unit contract "
            "(backends/base.py) is not holding on this install. The case "
            "study compensates locally for its own reads, but the bridge's "
            "sensing path does not -- fix configure_units() before running "
            "any experiment.")

    def _check_orbit_elements(self) -> Tuple[str, str]:
        """Measure the orbit rather than trusting that SetState worked.

        This is the check that catches a ``SetState Classical`` argument
        order or unit error: the command ACKs, STK keeps its default
        orbit, targets get placed along the *default* ground track, and
        every subsequent result is self-consistent and wrong.
        """
        self._orbit_ok = False
        position, velocity = self._state(0.0)
        sma_m, ecc, inc_rad = osculating_elements(position, velocity)
        inc_deg = math.degrees(inc_rad)

        wanted_sma_m = self._spec.orbit["semi_major_axis_km"] * 1000.0
        wanted_inc_deg = self._spec.orbit["inclination_deg"]
        self._report.measurements["osculating_sma_m"] = sma_m
        self._report.measurements["osculating_inclination_deg"] = inc_deg
        self._report.measurements["osculating_eccentricity"] = ecc

        detail = (
            f"measured a = {sma_m / 1000.0:.3f} km "
            f"(spec {wanted_sma_m / 1000.0:.3f} km), "
            f"i = {inc_deg:.3f} deg (spec {wanted_inc_deg:.3f} deg), "
            f"e = {ecc:.6f}")

        problems = []
        if abs(sma_m - wanted_sma_m) > SMA_TOLERANCE_M:
            problems.append(
                f"semi-major axis is off by "
                f"{(sma_m - wanted_sma_m) / 1000.0:+.1f} km")
        if abs(inc_deg - wanted_inc_deg) > INCLINATION_TOLERANCE_DEG:
            problems.append(
                f"inclination is off by {inc_deg - wanted_inc_deg:+.2f} deg")

        if problems:
            return FAIL, (
                detail + "\n" + "; ".join(problems) + ".\n"
                "SetState Classical did not take effect -- most likely the "
                "argument order or the angle unit. The satellite is on "
                "some other orbit, targets have been placed along ITS "
                "ground track, and the whole scenario is self-consistently "
                "measuring something other than the specified 500 km SSO.")
        self._orbit_ok = True
        return PASS, detail

    def _check_perturbations(self) -> Tuple[str, str]:
        """Is the truth propagator actually perturbed?

        Measured, not read off a propagator name. Osculating semi-major
        axis is constant under two-body motion and oscillates by several
        km under J2 at this altitude, so its spread across the window
        separates HPOP/J4 from TwoBody without needing an independent
        propagator to compare against.
        """
        if not self._orbit_ok:
            return SKIP, (
                "the satellite is not on the specified orbit, and the "
                "threshold this check uses is calibrated for the near-polar "
                "500 km SSO. On the first live run it reported an 82.6 m "
                "spread as 'unperturbed' while the satellite was actually "
                "at 1.7 deg inclination, where J2's short-period signature "
                "is far smaller -- a conclusion the measurement did not "
                "support. Fix the orbit and re-run.")

        samples = []
        count = 13
        for index in range(count):
            offset = self._spec.window_duration_s * index / (count - 1)
            position, velocity = self._state(offset)
            samples.append(osculating_elements(position, velocity)[0])

        spread = max(samples) - min(samples)
        self._report.measurements["osculating_sma_spread_m"] = spread
        named = self._propagator_name()
        detail = (f"osculating a varies by {spread:.1f} m over the "
                  f"{self._spec.window_duration_s:.0f} s window; STK reports "
                  f"the propagator as {named or 'unknown'}")

        if spread < PERTURBATION_SMA_SPREAD_M:
            return WARN, (
                detail + "\nThat is an unperturbed two-body propagation: J2 "
                "alone would drive several km of short-period variation at "
                "this altitude. Whatever the propagator is called, HPOP is "
                "not integrating perturbations. The paper must not claim "
                "HPOP truth propagation on the strength of this scenario.")
        return PASS, detail

    # -- sensor ------------------------------------------------------------

    def _check_sensor_exists(self) -> Tuple[str, str]:
        candidate = (self._build.sensor_path if self._build
                     else f"{self._satellite}/Sensor/{self._spec.sensor.name}")
        candidate = conv.canonical_object_path(candidate)
        if not self._backend.object_exists(candidate):
            return FAIL, (
                f"no sensor at {candidate}. The study has no payload; "
                f"nothing downstream can measure access.")
        self._sensor = candidate
        self._report.measurements["sensor_path"] = candidate
        return PASS, candidate

    def _measure_fov(self) -> Dict[str, object]:
        """Run the probe ladder once; both sensor checks read this.

        Returns a dict with the visibility of each probe angle, the
        bracket they imply, and -- where the backend can reach the
        Object Model -- the cone half-angle STK actually holds for the
        sensor. The direct read is worth having even alongside the
        probes: it separates "the Define command did not apply" from
        "it applied in the wrong unit" without inferring anything from
        access geometry.
        """
        if self._fov is not None:
            return self._fov

        specified = self._spec.sensor.half_angle_deg
        ladder = {f"{multiple:g}h": specified * multiple
                  for multiple in PROBE_LADDER}
        ladder["wide"] = PROBE_WIDE_DEG

        probe_epoch = self._probe_epoch()
        with self._probe_targets(ladder) as probes:
            seen = {name: self._probe_visible(path, probe_epoch)
                    for name, path in probes.items()}
            nadir_visible = seen.get("0h", False)
            flipped_visible = False
            if not nadir_visible:
                flipped_visible = self._retry_with_flipped_boresight(
                    probes["0h"], probe_epoch)

        visible = sorted(ladder[n] for n, ok in seen.items() if ok)
        hidden = sorted(ladder[n] for n, ok in seen.items() if not ok)

        self._fov = {
            "ladder": ladder,
            "seen": seen,
            "lower": visible[-1] if visible else 0.0,
            "upper": hidden[0] if hidden else float("inf"),
            "nadir_visible": nadir_visible,
            "flipped_visible": flipped_visible,
            "probe_epoch": probe_epoch,
            "reported_half_angle_deg": self._reported_half_angle_deg(),
        }
        return self._fov

    def _reported_half_angle_deg(self) -> Optional[float]:
        try:
            value = self._backend.get_sensor_cone_half_angle_rad(self._sensor)
        except StkBridgeError:
            return None
        return None if value is None else math.degrees(value)

    def _check_boresight(self) -> Tuple[str, str]:
        """Does the sensor look at the ground under the default attitude?

        The +Z boresight assumption has two independent halves and
        neither is safe to assume: that ``Point ... Fixed AzEl 0 90``
        aims the sensor along body +Z, and that STK's default satellite
        attitude puts body +Z towards nadir. Only their combination is
        observable, so that is what is measured.

        **The conclusion is gated on the cone being narrow.** On the
        first live run this check reported PASS while the sensor was at
        STK's 45 deg default -- a cone that wide sees a nadir target
        whatever the boresight is doing, so the test proved nothing and
        said it had proved something. A pass is only meaningful when the
        cone is tight enough for the nadir probe to discriminate.
        """
        if not self._sensor:
            return SKIP, "no sensor resolved"
        if self.is_mock:
            return self._skip_on_mock("sensor cones or access geometry")

        fov = self._measure_fov()
        specified = self._spec.sensor.half_angle_deg
        wide = fov["upper"] == float("inf") or fov["lower"] > 3.0 * specified

        if fov["nadir_visible"] and wide:
            return WARN, (
                f"a target at nadir is inside the cone, but the cone is "
                f"wider than {fov['lower']:.1f} deg -- at that width it "
                f"would see nadir whatever the boresight is doing, so this "
                f"says nothing about the boresight. Fix the cone half-angle "
                f"and re-run; this check cannot discriminate until then.")
        if fov["nadir_visible"]:
            return PASS, (
                f"a target directly at nadir is inside the "
                f"{fov['upper']:.2f} deg cone at "
                f"t={fov['probe_epoch']:.0f} s, so the boresight faces the "
                f"ground with the scenario's default attitude")
        if fov["flipped_visible"]:
            return FAIL, (
                "the sensor does NOT see a target at nadir as configured, "
                "but DOES after re-pointing to AzEl(0, -90). The boresight "
                "is inverted: either STK's default satellite attitude puts "
                "body -Z towards nadir, or the Point command's elevation "
                "sign is opposite to what was assumed. Fix "
                "scenario_builder._create_sensor and make C3's body-frame "
                "convention match, since the twin commands attitude in the "
                "same frame.")
        return FAIL, (
            "the sensor sees a target at nadir neither as configured nor "
            "with the boresight flipped. The Point command probably had no "
            "effect at all (the sensor may still be targeting or "
            "nadir-locked), or the cone is degenerate. Nothing downstream "
            "can measure access until this is resolved.")

    def _check_half_angle(self) -> Tuple[str, str]:
        """Bracket the effective half-angle rather than assert it.

        Probe targets sit at a ladder of off-nadir angles either side of
        the specified 2.0 deg. The largest visible angle and the
        smallest hidden one bracket the truth. A single yes/no probe
        would report "wrong" without saying how wrong, and the
        difference between a 2.2 deg cone and STK's untouched 45 deg
        default is the difference between a tolerance to note and a
        command that did nothing.
        """
        if not self._sensor:
            return SKIP, "no sensor resolved"
        if self.is_mock:
            return self._skip_on_mock("sensor cones or access geometry")

        specified = self._spec.sensor.half_angle_deg
        fov = self._measure_fov()
        ladder, seen = fov["ladder"], fov["seen"]
        lower, upper = fov["lower"], fov["upper"]
        self._report.measurements["measured_half_angle_lower_deg"] = lower
        self._report.measurements["measured_half_angle_upper_deg"] = upper

        ladder_text = ", ".join(
            f"{ladder[name]:.2f} deg {'seen' if ok else 'not seen'}"
            for name, ok in sorted(seen.items(), key=lambda kv: ladder[kv[0]]))
        detail = (f"probe ladder: {ladder_text}\n"
                  f"effective half-angle is in ({lower:.2f}, {upper:.2f}] deg")

        reported = fov["reported_half_angle_deg"]
        if reported is not None:
            self._report.measurements["reported_half_angle_deg"] = reported
            detail += f"\nSTK reports the cone angle as {reported:.4f} deg"

        if not fov["nadir_visible"]:
            return FAIL, (
                detail + "\nNothing is visible, not even at nadir -- see the "
                "boresight check; the half-angle cannot be measured until "
                "the sensor looks at the ground.")
        if upper == float("inf"):
            return FAIL, (
                detail + f"\nEven a target {PROBE_WIDE_DEG:.0f} deg "
                f"off-nadir is inside the cone, so the pattern command did "
                f"nothing and the sensor is still at STK's default "
                f"half-angle. The whole study depends on a narrow FOV: at "
                f"this width pointing error stops mattering and the effect "
                f"under test vanishes.")
        if lower <= specified <= upper:
            return PASS, detail
        return FAIL, (
            detail + f"\nThe specified {specified:.2f} deg lies outside the "
            f"measured bracket. The cone angle was set in the wrong unit, or "
            f"STK's SimpleConic 'cone angle' is a full angle rather than a "
            f"half-angle on this version. Fix the pattern command; do not "
            f"adjust the spec to match, since the half-angle is the "
            f"parameter the study's sensitivity rests on.")

    # -- targets -----------------------------------------------------------

    def _check_target_placement(self) -> Tuple[str, str]:
        """Round-trip every target through STK's own position read-back.

        The builder verified its *arithmetic* before issuing SetPosition.
        This verifies the *result*: it re-reads the sub-satellite point
        and the target position from STK and re-derives the cross-track
        offset and side. That closes the loop over SetPosition's
        argument order and angle unit, which the builder cannot check
        by construction -- a lat/lon swap or a degrees-for-radians error
        ACKs cleanly and puts the target somewhere else entirely.
        """
        lines: List[str] = []
        problems: List[str] = []
        unreadable: List[str] = []
        diagnostic = ""

        for target in sorted(self._spec.targets, key=lambda t: t.t_image_s):
            path = self._target_path(target.id)
            if not path:
                problems.append(f"{target.id}: no such object")
                continue

            placed = self._object_lla(path)
            if placed is None:
                unreadable.append(target.id)
                if not diagnostic:
                    diagnostic = self._last_read_diagnostic
                continue

            here = self._sub_satellite(target.t_image_s)
            ahead = self._sub_satellite(target.t_image_s
                                        + HEADING_SAMPLE_DT_S)
            actual_km = great_circle_distance_m(here, placed) / 1000.0
            side = side_of_track(here, ahead, placed)
            wanted_km = abs(target.cross_track_km)
            wanted_side = 1 if target.cross_track_km >= 0 else -1
            distance_ok = abs(actual_km - wanted_km) <= PLACEMENT_TOLERANCE_KM
            side_ok = side == wanted_side

            lines.append(
                f"{target.id:<3} t={target.t_image_s:6.1f}s  "
                f"requested {target.cross_track_km:+7.1f} km  "
                f"measured {actual_km:7.1f} km "
                f"{_side_name(side):<8} "
                f"{'ok' if distance_ok and side_ok else 'WRONG'}")
            if not distance_ok:
                problems.append(
                    f"{target.id} is {actual_km:.1f} km from the ground "
                    f"track, not {wanted_km:.1f} km")
            if not side_ok:
                problems.append(
                    f"{target.id} is on the {_side_name(side)} of the track, "
                    f"not the {_side_name(wanted_side)}")

        detail = "\n".join(lines)
        if unreadable:
            detail += (f"\ncould not read a position back for: "
                       f"{', '.join(unreadable)}")
            if diagnostic:
                detail += "\n" + diagnostic
        if problems:
            return FAIL, (
                detail + "\n" + "\n".join(problems) +
                "\nSetPosition Geodetic did not place these where the "
                "builder computed. Check its argument order (lat before "
                "lon?) and angle unit -- the backend forces radians, and a "
                "value STK reads as degrees lands the target elsewhere "
                "without any error.")
        if unreadable and not lines:
            return WARN, (
                detail + "\nNo target position could be read back, so "
                "placement is UNVERIFIED. The builder's own arithmetic "
                "check passed, but nothing has confirmed STK agrees.")
        if unreadable:
            return WARN, detail
        return PASS, detail

    def _check_target_reachable(self) -> Tuple[str, str]:
        """Is each target within the satellite's horizon at its epoch?

        Deliberately satellite-to-target, not sensor-to-target. With the
        scenario's default attitude the sensor points at nadir, so an
        off-nadir target is correctly invisible to it -- making that the
        test would report six failures for a scenario that is fine. What
        matters here is that the geometry permits access at all; whether
        the sensor achieves it is what the closed loop measures.
        """
        if self.is_mock:
            return self._skip_on_mock("access geometry")

        missing: List[str] = []
        lines: List[str] = []
        for target in sorted(self._spec.targets, key=lambda t: t.t_image_s):
            path = self._target_path(target.id)
            if not path:
                missing.append(target.id)
                continue
            intervals = self._backend.compute_access(
                self._satellite, path,
                self._t(target.t_image_s - 60.0),
                self._t(target.t_image_s + 60.0))
            covered = any(
                i.start.epoch_seconds
                <= self._t(target.t_image_s).epoch_seconds
                <= i.stop.epoch_seconds for i in intervals)
            lines.append(f"{target.id:<3} "
                         f"{'in view' if covered else 'NOT IN VIEW'} at "
                         f"t={target.t_image_s:.0f}s "
                         f"({len(intervals)} interval(s) nearby)")
            if not covered:
                missing.append(target.id)

        detail = "\n".join(lines)
        if missing:
            return FAIL, (
                detail + f"\n{', '.join(missing)} are not within the "
                f"satellite's horizon at their image epochs. Their placement "
                f"is wrong, or the ground track used to place them is not "
                f"the one the satellite now flies -- re-run the builder "
                f"after any change to the orbit or the epoch.")
        return PASS, detail

    # -- ground station ----------------------------------------------------

    def _check_ground_station_exists(self) -> Tuple[str, str]:
        config = self._spec.ground_station
        if not config:
            return SKIP, "no ground station in the spec"
        name = str(config.get("name", "GS1"))
        path, how = self._resolve_object(
            "Facility", name,
            self._build.ground_station_path if self._build else "")
        if not path:
            return WARN, (
                f"no Facility named {name!r}. The ground station supports "
                f"the terrain-masking argument only; the imaging experiments "
                f"(E1-E5) do not need it.")
        self._ground_station = path
        self._report.measurements["ground_station_path"] = path
        return PASS, f"{path} (resolved via {how})"

    def _check_elevation_constraint(self) -> Tuple[str, str]:
        """Does the minimum-elevation constraint actually shorten contact?

        Compares contact as configured against contact with the minimum
        dropped to 0 deg. A constraint that was accepted but not applied
        leaves the two identical.

        This is a *constraint*, not a mask. STK has no constant-elevation
        az-el mask -- the type enum offers only None, File and
        TerrainData -- and an earlier version of this check tried to use
        one as its control, failed on every run, and reported a verdict
        from three identical numbers.
        """
        if not self._ground_station:
            return SKIP, "no ground station"
        if self.is_mock:
            return self._skip_on_mock("elevation constraints")

        minimum = float(self._spec.ground_station.get("min_elevation_deg",
                                                      10.0))
        # Turn the terrain mask off for the duration. A terrain horizon
        # higher than the elevation minimum hides the constraint
        # completely -- the two measurements come out identical and the
        # check reports "not applied" about a constraint that is applied
        # and simply not binding. Controlling for the confound is the
        # difference between a measurement and a coincidence.
        confounded = ""
        with self._temporary_mask("none") as mask_off:
            if not mask_off:
                confounded = (
                    "\nNOTE: the terrain mask could not be disabled for "
                    "this comparison, so a terrain horizon above "
                    f"{minimum:.0f} deg would hide the constraint's "
                    "effect entirely.")
            baseline = self._contact_seconds()
            with self._temporary_elevation(0.0) as applied:
                unmasked = self._contact_seconds() if applied else None

        if unmasked is None:
            return SKIP, (
                f"contact over the window is {baseline:.1f} s, but no "
                f"SetConstraint ElevationAngle form was accepted, so the "
                f"0 deg control could not be applied and nothing was "
                f"measured. This is NOT evidence that the "
                f"{minimum:.0f} deg constraint is missing.")

        detail = (f"contact over the window: {baseline:.1f} s at the "
                  f"configured {minimum:.0f} deg minimum, "
                  f"{unmasked:.1f} s at 0 deg" + confounded)
        self._report.measurements["contact_s_configured"] = baseline
        self._report.measurements["contact_s_zero_elevation"] = unmasked

        if baseline <= 0.0 and unmasked <= 0.0:
            return WARN, (
                detail + "\nNo contact at all in either case, so the "
                "constraint is unverifiable here: the ground station sits "
                "on the ground track at t=600 s and the pass should cover "
                "it. Check the station's position before trusting any "
                "contact-derived result.")
        if abs(baseline - unmasked) < 1e-3:
            return WARN, (
                detail + f"\nIdentical. Either the {minimum:.0f} deg "
                f"constraint is not being applied -- in which case "
                f"contact windows are optimistic by the low-elevation "
                f"tails -- or something else is already occluding "
                f"everything below it.")
        if baseline > unmasked:
            return FAIL, (
                detail + "\nRaising the minimum elevation LENGTHENED "
                "contact, which is geometrically impossible. The "
                "constraint is being applied with the wrong sense or the "
                "wrong unit.")
        return PASS, detail

    def _check_terrain_mask(self) -> Tuple[str, str]:
        """Terrain masking: effective, or merely applied?

        Two access computations, against the only control STK actually
        has: the terrain mask, and no mask at all. If they agree, terrain
        data is not affecting this region -- either it is not loaded, or
        the local horizon never rises above the elevation minimum. The
        study still runs; the paper must then describe the mask as a
        spherical-Earth elevation constraint and must NOT claim terrain.
        """
        config = self._spec.ground_station or {}
        if not config.get("terrain_masking", False):
            return SKIP, "terrain masking not requested in the spec"
        if not self._ground_station:
            return SKIP, "no ground station"
        if self.is_mock:
            return self._skip_on_mock("terrain")

        reported = self._backend.get_az_el_mask(self._ground_station)
        terrain = self._contact_seconds()
        with self._temporary_mask("none") as applied:
            unmasked = self._contact_seconds() if applied else None
        restored = self._restore_configured_mask()

        if unmasked is None:
            self._report.measurements["mask_mode"] = "unmeasurable"
            return SKIP, (
                f"contact over the window is {terrain:.1f} s, but the "
                f"no-mask control could not be applied, so nothing "
                f"distinguishes terrain masking from no masking. "
                f"Reporting a verdict from one number would be an "
                f"inference from an experiment that never ran.")

        detail = (f"contact over the window: {terrain:.1f} s with the "
                  f"terrain mask, {unmasked:.1f} s with no mask")
        if reported is not None:
            detail += f"\nSTK reports the mask type as {reported}"
        if not restored:
            detail += ("\nWARNING: could not restore the terrain mask "
                       "afterwards; rebuild before running experiments.")
        self._report.measurements["contact_s_terrain"] = terrain
        self._report.measurements["contact_s_no_mask"] = unmasked

        if abs(terrain - unmasked) < 1e-3:
            self._report.measurements["mask_mode"] = "elevation_only"
            return WARN, (
                detail + "\nIdentical, so terrain is not affecting this "
                "region -- either terrain data is not loaded for it, or "
                "the local horizon never rises above the elevation "
                "minimum. The study still runs; the paper must describe "
                "the mask as a spherical-Earth elevation constraint and "
                "must NOT claim terrain masking.")
        if terrain > unmasked:
            return FAIL, (
                detail + "\nThe terrain mask LENGTHENED contact, which "
                "is impossible: a horizon can only occlude. Something "
                "other than the mask changed between the two "
                "measurements.")
        self._report.measurements["mask_mode"] = "terrain"
        return PASS, (detail + "\nTerrain shortens the contact windows, "
                      "so the mask is genuinely terrain-derived.")

    # -- measurement helpers ----------------------------------------------

    def _t(self, offset_s: float) -> StkTime:
        if self._epoch is None:
            self._epoch = self._backend.get_scenario_epoch()
        return self._epoch + offset_s

    def _state(self, offset_s: float) -> Tuple[Tuple[float, float, float],
                                               Tuple[float, float, float]]:
        return read_state(self._backend, self._satellite, self._t(offset_s))

    def _sub_satellite(self, offset_s: float) -> GeoPoint:
        point = self._object_lla(self._satellite, offset_s)
        if point is None:
            raise StkBridgeError(
                f"no sub-satellite point at t={offset_s:.1f} s; the "
                f"satellite has no ephemeris there")
        return point

    def _object_lla(self, path: str,
                    offset_s: float = 0.0) -> Optional[GeoPoint]:
        """Geodetic position of any object, or None if unreadable.

        Facilities and targets are static, so the epoch is irrelevant
        for them; it matters only for the satellite. The last failed
        read's diagnostic -- including the object's provider inventory
        -- is kept so the caller can report *why* rather than just that
        nothing came back.
        """
        read = read_geodetic(self._backend, path, self._t(offset_s),
                             self._angle_scale)
        if not read.ok:
            self._last_read_diagnostic = read.diagnostic
            return None
        return read.point

    def _probe_epoch(self) -> float:
        """A window epoch with no scheduled imaging near it.

        Probe targets are created and destroyed inside a check, but they
        are real objects in the scenario while they exist; putting them
        where a real target is imaged would risk a name or geometry
        collision in a scenario a user may then keep.
        """
        duration = self._spec.window_duration_s
        busy = [t.t_image_s for t in self._spec.targets]
        for fraction in (0.75, 0.9, 0.6, 0.45, 0.3):
            candidate = duration * fraction
            if all(abs(candidate - t) > 30.0 for t in busy):
                return candidate
        return duration * 0.5

    def _probe_visible(self, target_path: str, probe_epoch: float) -> bool:
        intervals = self._backend.compute_access(
            self._sensor, target_path,
            self._t(probe_epoch - PROBE_ACCESS_HALF_WINDOW_S),
            self._t(probe_epoch + PROBE_ACCESS_HALF_WINDOW_S))
        return bool(intervals)

    def _retry_with_flipped_boresight(self, target_path: str,
                                      probe_epoch: float) -> bool:
        """Try the opposite elevation, then put the sensor back.

        Degrees, like every other angle-bearing Connect command measured
        on this install. This previously sent radians, so the "flipped"
        retry actually set an elevation of -1.57 DEG -- essentially the
        horizon -- and the answer it produced said nothing about the
        boresight either way.
        """
        for label, elevation_deg in (("flip", -90.0),):
            if not self._connect_ok(
                    f"Point {self._sensor} Fixed AzEl 0.0 "
                    f"{elevation_deg!r} Rotate"):
                LOGGER.warning("could not apply the %s pointing", label)
                return False

        visible = self._probe_visible(target_path, probe_epoch)

        if not self._connect_ok(
                f"Point {self._sensor} Fixed AzEl 0.0 90.0 Rotate"):
            LOGGER.warning("could not restore the sensor pointing after the "
                           "boresight probe; rebuild before running "
                           "experiments")
        return visible

    def _contact_seconds(self) -> float:
        intervals = self._backend.compute_access(
            self._ground_station, self._satellite,
            self._t(0.0), self._t(self._spec.window_duration_s))
        return sum(i.duration_s for i in intervals)

    def _propagator_name(self) -> str:
        try:
            for info in self._backend.list_objects():
                if conv.canonical_object_path(info.object_path) \
                        == self._satellite:
                    return info.propagator_type
        except StkBridgeError:
            pass
        return ""

    # -- scenario manipulation --------------------------------------------

    def _connect_ok(self, command: str) -> bool:
        try:
            return bool(self._backend.execute_connect(command).ack)
        except StkBridgeError as exc:
            LOGGER.debug("connect command failed: %s (%s)", command, exc)
            return False

    class _MaskContext:
        """Applies an az-el mask mode, then restores the configured one.

        ``mode`` is one of STK's three: ``terrain``, ``none``, ``file``.
        There is no constant-elevation mask -- a live 12.6 install
        reports exactly ``eAzElMaskNone``, ``eMaskFile`` and
        ``eTerrainData``. The verifier previously tried to apply a
        constant mask as an experimental control, which failed on every
        run and left the terrain check with nothing to compare against.
        A minimum elevation is an access *constraint*, checked
        separately.
        """

        def __init__(self, verifier: "AcavScenarioVerifier", mode: str):
            self._verifier = verifier
            self._mode = mode
            self.applied = False

        def __enter__(self) -> bool:
            """Returns whether the mask was actually applied.

            Callers must branch on this. A check that concludes from a
            treatment which never ran is worse than no check: it reports
            a verdict with nothing behind it.
            """
            try:
                self._verifier._backend.set_az_el_mask(
                    self._verifier._ground_station, self._mode)
            except StkBridgeError as exc:
                LOGGER.warning("could not apply the %s mask: %s",
                               self._mode, exc)
                self._verifier._mask_failure = str(exc)
                return False
            self.applied = True
            return True

        def __exit__(self, exc_type, exc, tb) -> None:
            self._verifier._restore_configured_mask()

    def _temporary_mask(self, mode: str) -> "_MaskContext":
        return AcavScenarioVerifier._MaskContext(self, mode)

    class _ElevationConstraint:
        """Temporarily changes the minimum-elevation access constraint."""

        def __init__(self, verifier: "AcavScenarioVerifier",
                     minimum_deg: float):
            self._verifier = verifier
            self._minimum_deg = minimum_deg
            self.applied = False

        def __enter__(self) -> bool:
            self.applied = self._verifier._set_elevation_constraint(
                self._minimum_deg)
            return self.applied

        def __exit__(self, exc_type, exc, tb) -> None:
            configured = float((self._verifier._spec.ground_station or {})
                               .get("min_elevation_deg", 0.0))
            self._verifier._set_elevation_constraint(configured)

    def _temporary_elevation(self, minimum_deg: float
                             ) -> "_ElevationConstraint":
        return AcavScenarioVerifier._ElevationConstraint(self, minimum_deg)

    def _set_elevation_constraint(self, minimum_deg: float) -> bool:
        """Degrees first, matching every angle-bearing Connect command."""
        path = self._ground_station
        for command in (
            f"SetConstraint {path} ElevationAngle Min {minimum_deg!r}",
            f"SetConstraint {path} ElevationAngle Min "
            f"{math.radians(minimum_deg)!r}",
        ):
            if self._connect_ok(command):
                return True
        return False

    def _restore_configured_mask(self) -> bool:
        """Put the mask back to what the spec asks for.

        Restoring terrain unconditionally would be wrong for a spec that
        never asked for it: the verifier would leave the scenario in a
        state the builder never produced, and a later run would measure
        that instead.
        """
        path = self._ground_station
        if not path:
            return True
        config = self._spec.ground_station or {}
        mode = "terrain" if config.get("terrain_masking", False) else "none"
        try:
            self._backend.set_az_el_mask(path, mode)
        except StkBridgeError as exc:
            LOGGER.warning("could not restore the configured mask: %s", exc)
            return False
        return True

    class _ProbeTargets:
        """Creates FOV probe targets and always removes them again.

        The probes exist only for the duration of a measurement. Leaving
        them behind would put objects in a scenario the user may save
        and later mistake for part of the study.
        """

        def __init__(self, verifier: "AcavScenarioVerifier",
                     angles_deg: Dict[str, float]):
            self._verifier = verifier
            self._angles = angles_deg
            self._paths: Dict[str, str] = {}

        def __enter__(self) -> Dict[str, str]:
            verifier = self._verifier
            epoch = verifier._probe_epoch()
            here = verifier._sub_satellite(epoch)
            ahead = verifier._sub_satellite(epoch + HEADING_SAMPLE_DT_S)
            for name, angle_deg in self._angles.items():
                cross_km = _cross_track_km_for_off_nadir(
                    angle_deg, verifier._spec.altitude_km)
                point = cross_track_offset(here, ahead, cross_km)
                object_name = f"{_PROBE_PREFIX}_{name}".replace(".", "p")
                path = conv.canonical_object_path(
                    verifier._backend.create_object("Target", object_name))
                try:
                    set_position_geodetic(verifier._backend, path, point,
                                          verifier.geodetic_write_unit)
                except StkBridgeError as exc:
                    LOGGER.warning("could not place probe %s: %s", path, exc)
                self._paths[name] = path
            return self._paths

        def __exit__(self, exc_type, exc, tb) -> None:
            for path in self._paths.values():
                try:
                    self._verifier._backend.delete_object(path)
                except StkBridgeError:
                    LOGGER.warning("could not delete probe target %s", path)

    def _probe_targets(self, angles_deg: Dict[str, float]) -> "_ProbeTargets":
        return AcavScenarioVerifier._ProbeTargets(self, angles_deg)

    # -- object resolution -------------------------------------------------

    def _resolve_object(self, object_class: str, name: str,
                        hint: str = "") -> Tuple[str, str]:
        """Find an object, preferring what the builder reported.

        Returns ``(path, how)``; ``path`` is empty if nothing matched.
        The path *form* matters: STK reports absolute paths but accepts
        the ``*/`` relative form, and mixing them silently breaks
        dictionary lookups. Everything here is canonicalised, and which
        route succeeded is reported so a path-form problem is visible
        rather than inferred.
        """
        wanted = conv.canonical_object_path(f"*/{object_class}/{name}")
        if hint:
            hint = conv.canonical_object_path(hint)
            if self._backend.object_exists(hint):
                return hint, "build report"
        if self._backend.object_exists(wanted):
            return wanted, "canonical path"
        try:
            for info in self._backend.list_objects():
                path = conv.canonical_object_path(info.object_path)
                if path == wanted:
                    return path, "scenario inventory"
        except StkBridgeError:
            pass
        return "", ""

    def _target_path(self, target_id: str) -> str:
        if self._build:
            for placement in self._build.targets:
                if placement.target_id == target_id:
                    return placement.object_path
        path, _ = self._resolve_object("Target", target_id)
        return path

    def _object_inventory(self) -> str:
        try:
            objects = self._backend.list_objects()
        except StkBridgeError as exc:
            return f"(could not enumerate: {exc})"
        if not objects:
            return "(none -- is a scenario loaded?)"
        return ", ".join(sorted(
            conv.canonical_object_path(o.object_path) for o in objects))


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _cross_track_km_for_off_nadir(off_nadir_deg: float,
                                  altitude_km: float) -> float:
    """Inverse of :func:`acav.slew_model.off_nadir_angle_deg`.

    Solved by bisection rather than algebraically: the forward function
    is the study's contract for this geometry, and inverting it
    numerically guarantees the two agree exactly, where a hand-derived
    closed form could drift from it after an edit.
    """
    if off_nadir_deg <= 0.0:
        return 0.0
    low, high = 0.0, 3000.0
    for _ in range(80):
        middle = 0.5 * (low + high)
        if off_nadir_angle_deg(middle, altitude_km) < off_nadir_deg:
            low = middle
        else:
            high = middle
    return 0.5 * (low + high)


def _side_name(side: int) -> str:
    return {1: "right", -1: "left", 0: "on-track"}.get(side, "?")
