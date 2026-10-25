"""Startup convention self-tests.

Runs against the *live* backend before the bridge publishes anything, and
verifies empirically what the rest of the code would otherwise assume.
Every check here exists because getting it wrong produces output that is
wrong but entirely plausible -- the worst failure mode a digital twin can
have, because nothing looks broken.

Checks performed
----------------
1. **Data providers return data.** Observed failing on a live STK 12.6
   install during bring-up: a freshly created satellite returned zero
   columns and zero rows, and because the query itself raised nothing the
   emptiness was invisible. This check is first because everything
   downstream is meaningless without it.
2. **Distance is metres.** Confirms ``configure_units`` took effect by
   checking orbital radius magnitude. A 1000x error here is survivable
   only if detected.
3. **Angular rate scale.** STK 12.6 rejects
   ``SetCurrentUnit("AngleRate", "rad/sec")``, so the reported unit is
   *measured* by commanding a known body rate and reading it back rather
   than assumed either way.
4. **Quaternion element ordering.** Verifies that STK's Q1..Q4 maps to
   ROS ``(x, y, z, w)`` by injecting a known non-symmetric rotation and
   checking it round-trips. A quaternion with the scalar part in the
   wrong slot still normalises, still looks like a valid rotation, and is
   simply wrong.
5. **Monotonicity enforcement.** Confirms STK really does silently drop
   out-of-order samples, so that ``core/actuation.py``'s guard is doing
   necessary work rather than defending against a phantom.

Isolation
---------
The validator creates and owns a **dedicated, disposable satellite**
rather than testing whatever object the caller names. This is not
fastidiousness -- it was a real bug. An earlier version validated a
caller-supplied object that had already been switched to the RealTime
orbit propagator and already had attitude samples injected at the exact
timestamps the checks wanted to use. Both produced failures that looked
like STK defects and were actually test contamination:

* Switching a satellite to the **RealTime orbit propagator discards its
  analytic ephemeris**. The object then has data only inside
  Look-Ahead/Look-Behind of whatever samples have been injected, so a
  query at scenario epoch returns zero rows. Orbit-state checks must run
  on a normally-propagated object.
* Real-time attitude is **append-only with strictly increasing
  timestamps**, and STK 12.6 returns a NACK (not a silent drop) for an
  out-of-order sample. Re-using a timestamp another test already
  consumed therefore fails hard.

So: the validator makes its own satellite, propagates it, leaves the
orbit propagator alone, allocates every attitude timestamp from a
monotonic counter, and deletes the object afterwards.

Policy
------
Failures raise :class:`ConventionValidationError` and are fatal at
startup by default. Individual checks can be downgraded to warnings via
config for a deliberately degraded run (visualisation-only, say), but the
default is to refuse to start rather than to publish unverified data.
"""

from __future__ import annotations

import logging
import math
import uuid
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional

from stk_bridge.backends.base import (
    PropagatorType,
    StkBackend,
    StkTime,
)
from stk_bridge.core import conversions as conv
from stk_bridge.core import data_providers as dp
from stk_bridge.core.startup_phases import maybe_phase
from stk_bridge.exceptions import (
    ConventionValidationError,
    QueryError,
    StkBridgeError,
)

LOGGER = logging.getLogger(__name__)

#: Earth equatorial radius, m -- only used for plausibility bounds.
_R_EARTH = 6378137.0

#: A deliberately asymmetric test rotation: 30 degrees about an axis with
#: three distinct components. Every element of the quaternion is distinct
#: and non-zero, so a transposition or a scalar/vector swap cannot go
#: unnoticed the way it would with a rotation about a single axis.
_TEST_AXIS = (0.26726124, 0.53452248, 0.80178373)   # normalised (1,2,3)
_TEST_ANGLE_RAD = math.radians(30.0)


def _test_quaternion() -> conv.Quaternion:
    half = _TEST_ANGLE_RAD / 2.0
    s = math.sin(half)
    return (_TEST_AXIS[0] * s, _TEST_AXIS[1] * s, _TEST_AXIS[2] * s,
            math.cos(half))


@dataclass
class CheckResult:
    name: str
    passed: bool
    detail: str = ""
    fatal: bool = True
    skipped: bool = False


@dataclass
class ValidationReport:
    results: List[CheckResult] = field(default_factory=list)
    angle_rate_scale: conv.AngleRateScale = field(
        default_factory=conv.AngleRateScale)
    #: True if the backend surfaces out-of-order sample rejection as an
    #: exception (STK 12.6 over COM does). False means the loss is
    #: silent and only core/actuation.py's guard makes it visible.
    monotonicity_is_observable: bool = False
    #: Provider name that round-tripped attitude correctly, and the
    #: command form it pairs with. core/sensing.py (Phase 4) reads these
    #: rather than hardcoding a pairing, so the bridge uses whatever this
    #: install actually supports.
    attitude_provider: str = ""
    attitude_command_inertial: bool = False
    #: Provider that supplied a readable body angular rate, if any.
    angular_rate_provider: str = ""

    @property
    def failures(self) -> List[CheckResult]:
        return [r for r in self.results
                if not r.passed and not r.skipped and r.fatal]

    @property
    def warnings(self) -> List[CheckResult]:
        return [r for r in self.results
                if not r.passed and not r.skipped and not r.fatal]

    @property
    def ok(self) -> bool:
        return not self.failures

    def summary(self) -> str:
        lines = []
        for result in self.results:
            if result.skipped:
                mark = "skip"
            elif result.passed:
                mark = " ok "
            elif result.fatal:
                mark = "FAIL"
            else:
                mark = "warn"
            lines.append(f"  [{mark}] {result.name}")
            if result.detail:
                for line in result.detail.splitlines():
                    lines.append(f"         {line}")
        return "\n".join(lines)

    def raise_if_failed(self) -> None:
        if self.ok:
            return
        raise ConventionValidationError(
            "startup convention validation failed; the bridge will not "
            "publish unverified data.\n" + self.summary()
        )


class ConventionValidator:
    """Runs the startup self-tests against a live backend."""

    #: Name of the disposable satellite the validator creates.
    TEST_OBJECT_NAME = "StkBridgeValidation"

    def __init__(self, backend: StkBackend,
                 test_object_path: Optional[str] = None,
                 fatal_checks: Optional[Dict[str, bool]] = None,
                 cleanup: bool = True,
                 use_dedicated_object: bool = True,
                 prefer_inertial_attitude: bool = False,
                 phases=None):
        self._backend = backend
        #: Optional core.startup_phases.StartupPhases: each check is
        #: timed as "validation/<check>".
        self._phases = phases
        #: The attitude command form actuation is configured to use. When
        #: both pairings round-trip (they do on STK 12.6), the one that
        #: matches it must win: the sensing side reads through whichever
        #: provider validation reports, and the actuation side commands in
        #: the configured form, so a tie resolved the other way leaves the
        #: bridge commanding in one frame and reading back in another.
        self._prefer_inertial = prefer_inertial_attitude
        self._requested_path = (conv.canonical_object_path(test_object_path)
                                if test_object_path else None)
        self._path = self._requested_path or ""
        self._fatal = fatal_checks or {}
        self._cleanup = cleanup
        self._use_dedicated = use_dedicated_object
        self._owns_object = False
        self._report = ValidationReport()

        # Every attitude timestamp comes from here, so no two checks can
        # collide on a stamp and trip STK's monotonicity NACK.
        self._stamp_offset_s = 100.0

    def _next_stamp(self, epoch: StkTime) -> StkTime:
        """Allocate a fresh, strictly-increasing attitude timestamp."""
        self._stamp_offset_s += 5.0
        return epoch + self._stamp_offset_s

    def _setup_object(self) -> str:
        """Create and propagate the validator's own satellite.

        Falls back to the caller-supplied path if creation fails or if
        dedicated objects are disabled, warning clearly that results may
        be affected by that object's existing state.
        """
        if not self._use_dedicated:
            return self._path

        try:
            path = self._backend.create_object(
                "Satellite", f"{self.TEST_OBJECT_NAME}_{uuid.uuid4().hex}"
            )
            path = conv.canonical_object_path(path)
            # Own only the object created by this run. An interrupted prior
            # process can leave its satellite in an otherwise valid scenario.
            self._path = path
            self._owns_object = True
            propagate = getattr(self._backend, "propagate", None)
            if propagate is not None:
                propagate(path)
            LOGGER.info("validation using dedicated object %s", path)
            return path
        except Exception as exc:
            self._teardown_object()
            self._owns_object = False
            if not self._requested_path:
                raise
            LOGGER.warning(
                "could not create a dedicated validation object (%s); "
                "falling back to %s. Results may be affected by that "
                "object's existing propagator and attitude data.",
                exc, self._requested_path,
            )
            return self._requested_path

    def _teardown_object(self) -> None:
        if not (self._owns_object and self._cleanup):
            return
        try:
            self._backend.delete_object(self._path)
            LOGGER.debug("removed validation object %s", self._path)
        except Exception:
            LOGGER.debug("could not remove %s", self._path, exc_info=True)

    def _is_fatal(self, name: str, default: bool = True) -> bool:
        return bool(self._fatal.get(name, default))

    def _record(self, name: str, passed: bool, detail: str = "",
                default_fatal: bool = True, skipped: bool = False) -> None:
        self._report.results.append(CheckResult(
            name=name, passed=passed, detail=detail,
            fatal=self._is_fatal(name, default_fatal), skipped=skipped,
        ))

    def _run(self, name: str, check: Callable[[], str],
             default_fatal: bool = True) -> None:
        with maybe_phase(self._phases, f"validation/{name}"):
            self._run_check(name, check, default_fatal)

    def _run_check(self, name: str, check: Callable[[], str],
                   default_fatal: bool) -> None:
        try:
            detail = check()
            self._record(name, True, detail, default_fatal)
        except StkBridgeError as exc:
            self._record(name, False, str(exc), default_fatal)
        except Exception as exc:  # noqa: BLE001 - report, never abort
            self._record(name, False, f"{type(exc).__name__}: {exc}",
                         default_fatal)

    # -- entry point -------------------------------------------------------

    def validate(self) -> ValidationReport:
        try:
            with maybe_phase(self._phases, "validation/setup_object"):
                self._path = self._setup_object()
        except Exception as exc:
            self._record("setup", False,
                         f"could not create a validation object: {exc}")
            return self._report

        LOGGER.info("running startup convention validation on %s", self._path)
        try:
            self._run("ephemeris_present", self._check_ephemeris)
            self._run("data_provider_returns_data", self._check_data_present)
            self._run("distance_is_metres", self._check_distance_units)
            self._run("geodetic_angles_are_radians",
                      self._check_geodetic_angle_units)
            self._run("realtime_attitude_available",
                      self._check_realtime_attitude)
            self._run("quaternion_ordering", self._check_quaternion_ordering)
            self._run("angular_rate_scale", self._check_angle_rate_scale,
                      default_fatal=False)
            self._run("monotonicity_enforced", self._check_monotonicity,
                      default_fatal=False)
            self._run("attitude_history_clears", self._check_attitude_clear,
                      default_fatal=False)
        finally:
            with maybe_phase(self._phases, "validation/teardown_object"):
                self._teardown_object()

        LOGGER.info("validation complete:\n%s", self._report.summary())
        return self._report

    # -- individual checks -------------------------------------------------

    def _check_attitude_clear(self) -> str:
        """Does ``SetAttitude ClearData`` really drop real-time samples?

        It must, for any run on a reused scenario: STK keeps an object's
        real-time attitude samples across bridge sessions, and NACKs any
        new sample earlier than the newest one it holds. The first live
        run to reuse a scenario (2026-09-27) had every attitude command
        rejected for the first 58 s of simulation time -- exactly as far
        as the previous attempt had got -- while STK replayed that
        attempt's attitude and the twin fought it into SAFE. The
        actuation path now clears the history before an object's first
        command; this check proves the clear works on this install,
        rather than assuming a command that returns ACK did anything.
        """
        epoch = self._backend.get_scenario_epoch()
        later = self._next_stamp(epoch)
        earlier = later - 2.0
        self._backend.enable_realtime_attitude(self._path)
        self._backend.add_attitude_quaternion(
            self._path, later, _test_quaternion(), inertial=False)
        self._backend.clear_attitude_data(self._path)
        try:
            self._backend.add_attitude_quaternion(
                self._path, earlier, _test_quaternion(), inertial=False)
        except StkBridgeError as exc:
            raise ConventionValidationError(
                "after SetAttitude ClearData, a sample earlier than the "
                f"cleared one was still rejected ({exc}). The clear did "
                "not remove the history, so on a REUSED scenario every "
                "attitude command before the previous run's last sample "
                "will be rejected. Rebuild the scenario before each run "
                "until this passes.") from exc
        return ("ClearData removed the real-time attitude history: a "
                "sample 2 s before the cleared one was accepted")

    def _check_ephemeris(self) -> str:
        """Ensure the test object actually has ephemeris.

        A satellite created through ``Children.New`` carries a propagator
        and default orbital elements but generates no ephemeris until
        propagated. Every data provider on it then returns zero rows.
        Rather than merely detect that, this check fixes it where it can,
        because an unpropagated object is a setup omission, not a
        misconfiguration the operator needs to reason about.
        """
        propagate = getattr(self._backend, "propagate", None)
        if propagate is None:
            return ("backend does not expose propagate(); relying on the "
                    "scenario to have been propagated already")
        ran = propagate(self._path)
        if ran:
            return f"Propagate() ran on {self._path}"
        return ("Propagate() unavailable on this object (expected for a "
                "real-time-propagated vehicle, whose ephemeris comes from "
                "injected samples)")

    def _check_data_present(self) -> str:
        epoch = self._backend.get_scenario_epoch()
        data, mapping = dp.execute(
            self._backend, dp.CARTESIAN_POSITION, self._path,
            frame="ICRF", start=epoch, stop=epoch,
        )
        columns = ", ".join(data.columns)
        return (f"Cartesian Position/ICRF returned {data.row_count} row(s), "
                f"columns: {columns}\n"
                f"resolved x/y/z -> "
                f"{mapping['x']}/{mapping['y']}/{mapping['z']}")

    def _check_distance_units(self) -> str:
        epoch = self._backend.get_scenario_epoch()
        data, mapping = dp.execute(
            self._backend, dp.CARTESIAN_POSITION, self._path,
            frame="ICRF", start=epoch, stop=epoch,
        )
        x = dp.first_value(data, dp.CARTESIAN_POSITION, mapping, "x")
        y = dp.first_value(data, dp.CARTESIAN_POSITION, mapping, "y")
        z = dp.first_value(data, dp.CARTESIAN_POSITION, mapping, "z")
        radius = math.sqrt(x * x + y * y + z * z)

        if _R_EARTH * 0.95 < radius < 1.0e9:
            altitude = (radius - _R_EARTH) / 1000.0
            return (f"|r| = {radius:.1f} m (altitude {altitude:.1f} km) -- "
                    f"consistent with metres")

        if _R_EARTH / 1000.0 * 0.95 < radius < 1.0e6:
            raise ConventionValidationError(
                f"|r| = {radius:.3f}, which looks like KILOMETRES, not "
                f"metres. configure_units() did not take effect on this "
                f"install. Every distance and velocity the bridge publishes "
                f"would be wrong by a factor of 1000. Check the STK unit "
                f"preference dialog and the warnings logged by "
                f"configure_units()."
            )

        raise ConventionValidationError(
            f"|r| = {radius:.6g}, which matches neither metres nor "
            f"kilometres for an orbit about this central body. Either the "
            f"object is not where it is expected to be, or the position "
            f"elements were misidentified (resolved to "
            f"{mapping['x']}/{mapping['y']}/{mapping['z']})."
        )

    def _check_geodetic_angle_units(self) -> str:
        """Are LLA latitude and longitude radians, as the contract says?

        Distance is checked above by magnitude; angles cannot be, because
        a latitude is a small number in either unit. So this compares the
        reported latitude against the one implied by the Fixed-frame
        Cartesian position, whose unit the distance check has just
        established.

        This exists because the assumption failed on a live STK 12.6
        install and nothing noticed. ``SetCurrentUnit("Angle", "rad")``
        was accepted, and "LLA State" went on reporting degrees --
        because **Latitude and Longitude are separate unit dimensions
        from Angle**, and only Angle was being set. The bridge published
        LLASample values 57x too large while its own logs said units
        were SI.

        Fatal by default: a digital twin fed degrees-labelled-radians
        produces plausible, wrong, silently-accepted results, which is
        the failure mode this whole validation pass exists to prevent.
        """
        epoch = self._backend.get_scenario_epoch()
        cartesian, cmap = dp.execute(
            self._backend, dp.CARTESIAN_POSITION, self._path,
            frame="Fixed", start=epoch, stop=epoch)
        x = dp.first_value(cartesian, dp.CARTESIAN_POSITION, cmap, "x")
        y = dp.first_value(cartesian, dp.CARTESIAN_POSITION, cmap, "y")
        z = dp.first_value(cartesian, dp.CARTESIAN_POSITION, cmap, "z")

        radius = math.sqrt(x * x + y * y + z * z)
        if radius <= 0.0:
            raise ConventionValidationError(
                "Fixed-frame position is degenerate; cannot check the "
                "geodetic angle unit against it.")
        implied_lat_rad = math.asin(max(-1.0, min(1.0, z / radius)))

        lla, lmap = dp.execute(self._backend, dp.LLA_STATE, self._path,
                               frame="Fixed", start=epoch, stop=epoch)
        reported = dp.first_value(lla, dp.LLA_STATE, lmap, "latitude")

        # Near the equator the two units are indistinguishable, so say so
        # rather than confirm whichever was asked about first.
        if abs(implied_lat_rad) < 0.05:
            return (f"latitude {implied_lat_rad:.4f} rad is too close to the "
                    f"equator to distinguish radians from degrees; unit "
                    f"UNVERIFIED at this epoch")

        # Geocentric vs geodetic latitude differ by at most 0.19 deg,
        # nowhere near the 57x that separates the two units.
        if abs(reported - implied_lat_rad) <= abs(
                math.radians(reported) - implied_lat_rad):
            return (f"LLA latitude {reported:.6f} matches the "
                    f"Cartesian-implied {implied_lat_rad:.6f} rad -- radians")

        raise ConventionValidationError(
            f"LLA State reports latitude {reported:.6f}, which matches "
            f"{math.degrees(implied_lat_rad):.6f} DEGREES rather than the "
            f"{implied_lat_rad:.6f} radians the units contract promises. "
            f"Latitude and Longitude are separate unit dimensions from "
            f"Angle in STK: setting Angle alone leaves these in degrees. "
            f"Check that configure_units() set Latitude and Longitude too, "
            f"and that GetCurrentUnitAbbrv agrees they took effect."
        )

    def _check_realtime_attitude(self) -> str:
        """Confirm the real-time attitude channel accepts a sample.

        This is the primary ADCS actuation path. If it does not work,
        nothing else about the bridge matters, so it is verified before
        the subtler convention checks that depend on it.
        """
        self._backend.enable_realtime_attitude(self._path)
        epoch = self._backend.get_scenario_epoch()
        self._backend.add_attitude_quaternion(
            self._path, self._next_stamp(epoch), _test_quaternion(),
            inertial=False,
        )
        return ("SetAttitude Standard + AddAttitude CBFQuat accepted a "
                "sample")

    def _check_quaternion_ordering(self) -> str:
        """Determine the correct command/read-back pairing empirically.

        STK 12.6 offers two real-time attitude command forms and two
        matching read-back providers, and they must be paired correctly:

        ==========================  ==============================
        Command                     Read-back provider
        ==========================  ==============================
        ``AddAttitude CBFQuat``     ``ECF Attitude Quaternions``
        ``AddAttitude Quat``        ``Attitude Quaternions``
        ==========================  ==============================

        Both read-back providers are **ungrouped** on 12.6 -- they take
        no reference-frame argument -- so a mismatched pairing does not
        error. It silently returns an attitude that differs by the
        Earth-rotation angle at the sample epoch: tens of degrees, and
        indistinguishable from a real attitude. That is exactly what a
        naive version of this check produced (a 99 deg "ordering error"
        that was actually a frame error), so rather than assume a
        pairing, both are tried and the one that round-trips is recorded
        on the report for ``core/sensing.py`` to use.

        The test rotation has four distinct non-zero components, so a
        genuine element-ordering fault still cannot hide behind a
        correct pairing.
        """
        commanded = _test_quaternion()
        epoch = self._backend.get_scenario_epoch()

        pairings = (
            (False, dp.ECF_ATTITUDE_QUATERNION,
             "AddAttitude CBFQuat <-> ECF Attitude Quaternions"),
            (True, dp.ATTITUDE_QUATERNION,
             "AddAttitude Quat <-> Attitude Quaternions"),
        )
        # The configured command form's own pairing is tried first and
        # wins any tie (strict < below), so a round-trip that works for
        # both forms reports the provider that matches actuation.
        if self._prefer_inertial:
            pairings = pairings[::-1]

        attempts: List[str] = []
        best: Optional[tuple] = None

        for inertial, spec, label in pairings:
            sample_time = self._next_stamp(epoch)
            try:
                self._backend.enable_realtime_attitude(self._path)
                self._backend.add_attitude_quaternion(
                    self._path, sample_time, commanded, inertial=inertial
                )
                data, mapping = dp.execute(
                    self._backend, spec, self._path,
                    start=sample_time, stop=sample_time,
                )
            except StkBridgeError as exc:
                attempts.append(f"{label}: {exc}")
                continue

            read = conv.quaternion_from_stk_elements(
                dp.first_value(data, spec, mapping, "q1"),
                dp.first_value(data, spec, mapping, "q2"),
                dp.first_value(data, spec, mapping, "q3"),
                dp.first_value(data, spec, mapping, "q4"),
            )

            if not conv.is_unit_quaternion(read, tolerance=1e-3):
                norm = math.sqrt(sum(c * c for c in read))
                attempts.append(f"{label}: read-back |q| = {norm:.6f}, "
                                f"not a unit quaternion")
                continue

            error_rad = conv.quaternion_angle_between(commanded, read)
            attempts.append(f"{label}: {math.degrees(error_rad):.4f} deg")
            if best is None or error_rad < best[0]:
                best = (error_rad, spec, label, inertial, read, data)

        if best is None:
            inventory = dp.format_available(self._backend, self._path)
            raise ConventionValidationError(
                "no attitude command/read-back pairing worked.\n  " +
                "\n  ".join(attempts) +
                "\nProviders available on this object:\n" + inventory
            )

        error_rad, spec, label, inertial, read, data = best

        if error_rad > math.radians(1.0):
            # Every pairing failed. Distinguish an ordering fault from a
            # frame fault: a permutation preserves the multiset of
            # component magnitudes, a frame rotation does not.
            commanded_mags = sorted(abs(c) for c in commanded)
            read_mags = sorted(abs(c) for c in read)
            same_mags = all(abs(a - b) < 1e-3
                            for a, b in zip(commanded_mags, read_mags))

            if same_mags:
                swapped = (read[3], read[0], read[1], read[2])
                hint = ""
                if conv.quaternion_angle_between(commanded,
                                                 swapped) < math.radians(1.0):
                    hint = ("\n  A scalar-FIRST interpretation (w,x,y,z) "
                            "matches. This install puts the scalar part in "
                            "Q1, not Q4. Fix "
                            "conversions.quaternion_from_stk_elements.")
                diagnosis = ("component magnitudes MATCH, so this is an "
                             "element-ORDERING fault." + hint)
            else:
                diagnosis = (
                    "component magnitudes DIFFER, so this is a FRAME "
                    "fault, not an ordering fault -- the read-back is a "
                    "different rotation, not a reshuffle of the same one. "
                    "Check that the command form and read-back provider "
                    "are paired as documented above."
                )

            raise ConventionValidationError(
                f"best attitude round-trip error is "
                f"{math.degrees(error_rad):.3f} deg ({label}).\n"
                f"  commanded (x,y,z,w): "
                f"{tuple(round(c, 6) for c in commanded)}\n"
                f"  read back  (x,y,z,w): "
                f"{tuple(round(c, 6) for c in read)}\n"
                f"  {diagnosis}\n"
                f"  all attempts:\n    " + "\n    ".join(attempts)
            )

        self._report.attitude_provider = data.provider_name
        self._report.attitude_command_inertial = inertial
        return (f"round-trip error {math.degrees(error_rad):.4f} deg via "
                f"{label}.\nSTK Q1..Q4 maps to ROS (x,y,z,w) as assumed. "
                f"core/sensing.py will read attitude from "
                f"{data.provider_name!r}.\nall attempts: " +
                "; ".join(attempts))

    def _check_angle_rate_scale(self) -> str:
        """Measure whether STK reports body rates in rad/s or deg/s.

        Necessary because STK 12.6 rejects ``rad/sec`` as an AngleRate
        unit preference, so this dimension cannot be forced to SI the way
        distance can. A known rate is commanded and read back; the ratio
        identifies the unit. The result is installed on the report and
        consumed by ``data_providers.column_values`` for every
        angular-rate field.
        """
        commanded_radps = 0.05     # ~2.86 deg/s: distinct in either unit
        epoch = self._backend.get_scenario_epoch()
        sample_time = self._next_stamp(epoch)

        self._backend.enable_realtime_attitude(self._path)
        self._backend.add_attitude_quaternion(
            self._path, sample_time - 1.0, _test_quaternion(), inertial=False
        )
        self._backend.add_attitude_angular_velocity(
            self._path, sample_time, (0.0, 0.0, commanded_radps)
        )

        # The angular-velocity provider is the least consistently named
        # across STK versions, so resolve it by enumeration rather than
        # exhausting a hardcoded alternatives list.
        spec, data, mapping = dp.ANGULAR_VELOCITY, None, None
        resolved = dp.resolve_spec_provider(
            self._backend, dp.ANGULAR_VELOCITY, self._path,
            "angular", "velocity",
        )
        attempts = []
        for candidate, frame in (
            (dp.ANGULAR_VELOCITY, "Body"),
            (dp.ANGULAR_VELOCITY, "ICRF"),
            (dp.ATTITUDE_SEGMENT, "Body"),
        ):
            try:
                data, mapping = dp.execute(
                    self._backend, candidate, self._path, frame=frame,
                    start=sample_time, stop=sample_time, allow_empty=True,
                )
                spec = candidate
                break
            except QueryError as exc:
                attempts.append(f"{candidate.provider_name}/{frame}: {exc}")
                data = None

        if data is None or data.row_count == 0:
            self._report.angle_rate_scale = conv.AngleRateScale()
            hint = (f"enumeration suggests {resolved!r}" if resolved
                    else "enumeration found no angular-velocity provider")
            inventory = dp.format_available(self._backend, self._path)
            raise ConventionValidationError(
                "angular velocity could not be read back, so its unit "
                "could not be measured. Angular-rate output will be "
                "published with an UNVERIFIED scale (assumed rad/s). If "
                "the twin's rate signals look wrong by a factor of ~57, "
                "this is why.\n"
                f"  {hint}\n  attempts:\n    " +
                "\n    ".join(attempts[:3]) +
                "\n  providers available on this object:\n" + inventory
            )

        # data.provider_name is the name that actually resolved, which
        # may be one of the spec's alternatives -- reporting the spec's
        # primary name here would tell the operator to look for a
        # provider this install does not have.
        self._report.angular_rate_provider = data.provider_name
        read = dp.first_value(data, spec, mapping, "wz",
                              angle_rate_scale=conv.AngleRateScale(factor=1.0))
        if abs(read) < 1e-9:
            raise ConventionValidationError(
                "angular velocity read back as zero; cannot infer units"
            )

        ratio = read / commanded_radps
        if abs(ratio - 1.0) < 0.05:
            self._report.angle_rate_scale = conv.AngleRateScale.radians_per_second()
            return (f"STK reports rad/s (ratio {ratio:.4f}) via "
                    f"{data.provider_name!r}")
        if abs(ratio - conv.RAD_TO_DEG) < 0.05 * conv.RAD_TO_DEG:
            self._report.angle_rate_scale = conv.AngleRateScale.degrees_per_second()
            return (f"STK reports deg/s (ratio {ratio:.4f}) via "
                    f"{data.provider_name!r}; a {conv.DEG_TO_RAD:.8f} scale "
                    f"will be applied to every angular-rate field")

        self._report.angle_rate_scale = conv.AngleRateScale()
        raise ConventionValidationError(
            f"angular rate read back as {read:.6g} for a commanded "
            f"{commanded_radps} rad/s (ratio {ratio:.4f}), which matches "
            f"neither rad/s nor deg/s. The rate elements may be "
            f"misidentified or the sample may have been interpolated."
        )

    def _check_monotonicity(self) -> str:
        """Confirm STK silently drops out-of-order samples.

        If this ever stops being true, ``core/actuation.py``'s
        monotonicity guard becomes unnecessary overhead -- but far more
        likely, a failure here means the guard is protecting against
        something subtler than assumed, which is worth knowing.
        """
        epoch = self._backend.get_scenario_epoch()
        newer = self._next_stamp(epoch)
        older = newer - 2.0
        quaternion = _test_quaternion()

        self._backend.enable_realtime_attitude(self._path)
        self._backend.add_attitude_quaternion(self._path, newer,
                                              quaternion, inertial=False)
        try:
            self._backend.add_attitude_quaternion(self._path, older,
                                                  quaternion, inertial=False)
        except StkBridgeError as exc:
            self._report.monotonicity_is_observable = True
            return (f"STK rejected the out-of-order sample explicitly "
                    f"({type(exc).__name__}). Confirmed on STK 12.6: the "
                    f"rejection is a NACK, not a silent drop, so failures "
                    f"are observable. core/actuation.py's guard still runs "
                    f"first so the drop is counted rather than merely "
                    f"logged.")
        self._report.monotonicity_is_observable = False
        return ("out-of-order sample accepted without error -- this backend "
                "drops it silently, so core/actuation.py's guard is the "
                "only thing that makes the loss observable.")


# ---------------------------------------------------------------------------

def validate_backend(backend: StkBackend, test_object_path: str,
                     config: Optional[Dict] = None,
                     phases=None) -> ValidationReport:
    """Convenience entry point used by ``nodes/bridge_node.py``.

    Config shape::

        validation:
          enabled: true
          fatal:
            angular_rate_scale: false
            monotonicity_enforced: false
    """
    settings = (config or {}).get("validation", {}) or {}
    if not settings.get("enabled", True):
        LOGGER.warning(
            "startup validation is DISABLED. The bridge will publish data "
            "whose units, frames, and quaternion ordering have not been "
            "verified against this STK install."
        )
        return ValidationReport()

    validator = ConventionValidator(
        backend=backend,
        test_object_path=test_object_path,
        fatal_checks=settings.get("fatal") or {},
        prefer_inertial_attitude=bool(
            ((config or {}).get("actuation") or {}).get(
                "default_attitude_inertial", False)),
        phases=phases,
    )
    return validator.validate()
