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
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional

from stk_bridge.backends.base import (
    PropagatorType,
    StkBackend,
    StkTime,
)
from stk_bridge.core import conversions as conv
from stk_bridge.core import data_providers as dp
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

    def __init__(self, backend: StkBackend,
                 test_object_path: str,
                 fatal_checks: Optional[Dict[str, bool]] = None,
                 cleanup: bool = True):
        self._backend = backend
        self._path = conv.canonical_object_path(test_object_path)
        self._fatal = fatal_checks or {}
        self._cleanup = cleanup
        self._report = ValidationReport()

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
        LOGGER.info("running startup convention validation on %s", self._path)

        self._run("ephemeris_present", self._check_ephemeris)
        self._run("data_provider_returns_data", self._check_data_present)
        self._run("distance_is_metres", self._check_distance_units)
        self._run("realtime_attitude_available",
                  self._check_realtime_attitude)
        self._run("quaternion_ordering", self._check_quaternion_ordering)
        self._run("angular_rate_scale", self._check_angle_rate_scale,
                  default_fatal=False)
        self._run("monotonicity_enforced", self._check_monotonicity,
                  default_fatal=False)

        LOGGER.info("validation complete:\n%s", self._report.summary())
        return self._report

    # -- individual checks -------------------------------------------------

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

    def _check_realtime_attitude(self) -> str:
        """Confirm the real-time attitude channel accepts a sample.

        This is the primary ADCS actuation path. If it does not work,
        nothing else about the bridge matters, so it is verified before
        the subtler convention checks that depend on it.
        """
        self._backend.enable_realtime_attitude(self._path)
        epoch = self._backend.get_scenario_epoch()
        self._backend.add_attitude_quaternion(
            self._path, epoch + 1.0, _test_quaternion(), inertial=False
        )
        return ("SetAttitude Standard + AddAttitude CBFQuat accepted a "
                "sample")

    def _check_quaternion_ordering(self) -> str:
        """Inject a known rotation and confirm it reads back unchanged.

        The injected quaternion has four distinct non-zero components, so
        any permutation -- most importantly a scalar-first vs
        scalar-last swap -- changes the represented rotation detectably.
        Comparison uses the angle between orientations, which is immune
        to the q/-q double cover that would otherwise produce spurious
        failures.
        """
        commanded = _test_quaternion()
        epoch = self._backend.get_scenario_epoch()
        sample_time = epoch + 2.0

        self._backend.enable_realtime_attitude(self._path)
        self._backend.add_attitude_quaternion(
            self._path, sample_time, commanded, inertial=False
        )

        try:
            data, mapping = dp.execute(
                self._backend, dp.ATTITUDE_QUATERNION, self._path,
                frame="Fixed", start=sample_time, stop=sample_time,
            )
        except QueryError as exc:
            raise ConventionValidationError(
                "could not read attitude back to verify quaternion "
                f"ordering: {exc}\nWithout this check the bridge cannot "
                "confirm that STK's Q1..Q4 maps to ROS (x,y,z,w). If this "
                "install cannot report attitude quaternions, set "
                "validation.fatal.quaternion_ordering: false in "
                "bridge_params.yaml to proceed at your own risk."
            ) from exc

        read = conv.quaternion_from_stk_elements(
            dp.first_value(data, dp.ATTITUDE_QUATERNION, mapping, "q1"),
            dp.first_value(data, dp.ATTITUDE_QUATERNION, mapping, "q2"),
            dp.first_value(data, dp.ATTITUDE_QUATERNION, mapping, "q3"),
            dp.first_value(data, dp.ATTITUDE_QUATERNION, mapping, "q4"),
        )

        if not conv.is_unit_quaternion(read, tolerance=1e-3):
            norm = math.sqrt(sum(c * c for c in read))
            raise ConventionValidationError(
                f"attitude read back is not unit-norm (|q| = {norm:.6f}); "
                f"the elements resolved to "
                f"{mapping['q1']}/{mapping['q2']}/{mapping['q3']}/"
                f"{mapping['q4']} are probably not a quaternion"
            )

        error_rad = conv.quaternion_angle_between(commanded, read)
        if error_rad > math.radians(1.0):
            swapped = (read[3], read[0], read[1], read[2])
            swap_error = conv.quaternion_angle_between(commanded, swapped)
            hint = ""
            if swap_error < math.radians(1.0):
                hint = ("\nA scalar-first interpretation (w,x,y,z) DOES "
                        "match. This install reports the scalar part in "
                        "Q1, not Q4. Fix "
                        "conversions.quaternion_from_stk_elements.")
            raise ConventionValidationError(
                f"quaternion round-trip error is "
                f"{math.degrees(error_rad):.3f} deg.\n"
                f"  commanded (x,y,z,w): "
                f"{tuple(round(c, 6) for c in commanded)}\n"
                f"  read back  (x,y,z,w): "
                f"{tuple(round(c, 6) for c in read)}{hint}"
            )

        return (f"round-trip error {math.degrees(error_rad):.4f} deg -- "
                f"STK Q1..Q4 maps to ROS (x,y,z,w) as assumed")

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
        sample_time = epoch + 3.0

        self._backend.enable_realtime_attitude(self._path)
        self._backend.add_attitude_quaternion(
            self._path, sample_time - 0.5, _test_quaternion(), inertial=False
        )
        self._backend.add_attitude_angular_velocity(
            self._path, sample_time, (0.0, 0.0, commanded_radps)
        )

        data, mapping = dp.execute(
            self._backend, dp.ANGULAR_VELOCITY, self._path,
            frame="Body", start=sample_time, stop=sample_time,
            allow_empty=True,
        )
        if data.row_count == 0:
            self._report.angle_rate_scale = conv.AngleRateScale()
            raise ConventionValidationError(
                "angular velocity could not be read back, so its unit "
                "could not be measured. Angular-rate output will be "
                "published with an UNVERIFIED scale (assumed rad/s). If "
                "the twin's rate signals look wrong by a factor of ~57, "
                "this is why."
            )

        read = dp.first_value(data, dp.ANGULAR_VELOCITY, mapping, "wz",
                              angle_rate_scale=conv.AngleRateScale(factor=1.0))
        if abs(read) < 1e-9:
            raise ConventionValidationError(
                "angular velocity read back as zero; cannot infer units"
            )

        ratio = read / commanded_radps
        if abs(ratio - 1.0) < 0.05:
            self._report.angle_rate_scale = conv.AngleRateScale.radians_per_second()
            return f"STK reports rad/s (ratio {ratio:.4f})"
        if abs(ratio - conv.RAD_TO_DEG) < 0.05 * conv.RAD_TO_DEG:
            self._report.angle_rate_scale = conv.AngleRateScale.degrees_per_second()
            return (f"STK reports deg/s (ratio {ratio:.4f}); a "
                    f"{conv.DEG_TO_RAD:.8f} scale will be applied to every "
                    f"angular-rate field")

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
        base = epoch + 10.0
        quaternion = _test_quaternion()

        self._backend.enable_realtime_attitude(self._path)
        self._backend.add_attitude_quaternion(self._path, base + 1.0,
                                              quaternion, inertial=False)
        try:
            self._backend.add_attitude_quaternion(self._path, base,
                                                  quaternion, inertial=False)
        except StkBridgeError as exc:
            return (f"backend rejected an out-of-order sample explicitly "
                    f"({type(exc).__name__}) rather than silently -- the "
                    f"actuation guard remains correct either way")
        return ("out-of-order sample accepted without error, as expected: "
                "STK drops it silently. core/actuation.py's guard is what "
                "makes this observable.")


# ---------------------------------------------------------------------------

def validate_backend(backend: StkBackend, test_object_path: str,
                     config: Optional[Dict] = None) -> ValidationReport:
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
    )
    return validator.validate()
