"""Scenario specification loader.

Parses ``config/acav_scenario.yaml`` into typed dataclasses, recomputes
every value the YAML marks ``[DERIVED]``, and cross-checks the
duplicated quantities. Downstream phases construct one :class:`AcavSpec`
and read from it; none of them parse the YAML themselves.

Why the cross-checks exist
--------------------------
A few quantities appear in more than one place in the YAML because two
different consumers need them in two different sections
(``pointing.settle_hold_s`` and ``scheduling.hold_before_image_s`` are
the same physical quantity, needed by the twin and the scheduler
respectively). Duplication is the lesser evil compared to a scheduler
reaching into a "pointing" section it otherwise has no business
reading -- but silently diverging duplicates would make E2's comparison
meaningless. So they are checked at load time and a mismatch is a hard
error, not a warning.

The same applies to the derived actuator limits: the YAML comments give
them for the reader's benefit, and this module recomputes them from
first principles. If a reader edits the inertia and forgets the comment,
the code is still right.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import yaml

from acav.slew_model import (
    SlewLimits,
    off_nadir_angle_deg,
    slew_between_targets_rad,
    slew_time_s,
)

EARTH_RADIUS_KM = 6378.137


@dataclass(frozen=True)
class TargetSpec:
    id: str
    t_image_s: float
    cross_track_km: float
    priority: float = 1.0

    def off_nadir_deg(self, altitude_km: float) -> float:
        return off_nadir_angle_deg(self.cross_track_km, altitude_km)


@dataclass(frozen=True)
class SensorSpec:
    name: str
    object_path: str
    half_angle_deg: float
    boresight_body_axis: tuple


@dataclass(frozen=True)
class PointingSpec:
    capture_error_deg: float
    settle_hold_s: float
    settle_entry_deg: float
    max_body_rate_deg_s: float


@dataclass(frozen=True)
class ActuatorSpec:
    max_torque_Nm: float
    momentum_capacity_Nms: float
    configuration: str = "orthogonal_3axis"


@dataclass(frozen=True)
class SchedulingSpec:
    settle_estimate_s: float
    hold_before_image_s: float
    return_to_nadir_between_targets: bool


@dataclass
class AcavSpec:
    """The whole case-study specification, validated."""

    raw: Dict
    name: str
    window_start_utc: str
    window_duration_s: float
    step_s: float

    spacecraft_name: str
    spacecraft_object_path: str
    mass_kg: float
    inertia: Dict[str, float]
    altitude_km: float
    orbit: Dict[str, float]
    propagator: str

    sensor: SensorSpec
    actuators: ActuatorSpec
    pointing: PointingSpec
    scheduling: SchedulingSpec
    targets: List[TargetSpec]
    ground_station: Dict
    bridge: Dict

    # ---------------------------------------------------------------
    # Construction
    # ---------------------------------------------------------------

    @classmethod
    def default_path(cls) -> str:
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(os.path.dirname(here), "config",
                            "acav_scenario.yaml")

    @classmethod
    def locate(cls, path: Optional[str] = None) -> str:
        """Find the scenario YAML from a source tree or an installed share.

        ``default_path`` resolves relative to this module, which is
        correct in a source checkout but not after an ament install,
        where the package lands in ``site-packages`` and its config in
        ``share/acav/config``. Command-line tools call this so that
        ``ros2 run acav verify_acav_scenario`` and
        ``python scripts/verify_acav_scenario.py`` read the same file.
        """
        if path:
            return path
        candidate = cls.default_path()
        if os.path.isfile(candidate):
            return candidate
        try:
            from ament_index_python.packages import (
                get_package_share_directory,
            )
            share = get_package_share_directory("acav")
        except Exception:
            return candidate
        installed = os.path.join(share, "config", "acav_scenario.yaml")
        return installed if os.path.isfile(installed) else candidate

    @classmethod
    def load(cls, path: Optional[str] = None) -> "AcavSpec":
        path = cls.locate(path)
        if not os.path.isfile(path):
            raise FileNotFoundError(
                f"ACAV scenario spec not found at {path}. This file is the "
                f"contract every C2-C6 phase reads; it is not optional."
            )
        with open(path, "r", encoding="utf-8") as handle:
            raw = yaml.safe_load(handle) or {}
        return cls.from_dict(raw)

    @classmethod
    def from_dict(cls, raw: Dict) -> "AcavSpec":
        scenario = _require(raw, "scenario")
        spacecraft = _require(raw, "spacecraft")
        sensor = _require(raw, "sensor")
        actuators = _require(raw, "actuators")
        pointing = _require(raw, "pointing")
        scheduling = _require(raw, "scheduling")
        targets_block = _require(raw, "targets")

        orbit = _require(spacecraft, "orbit")
        altitude_km = float(orbit["semi_major_axis_km"]) - EARTH_RADIUS_KM

        wheels = _require(actuators, "reaction_wheels")

        spec = cls(
            raw=raw,
            name=str(scenario["name"]),
            window_start_utc=str(scenario["window_start_utc"]),
            window_duration_s=float(scenario["window_duration_s"]),
            step_s=float(scenario["step_s"]),
            spacecraft_name=str(spacecraft["name"]),
            spacecraft_object_path=str(spacecraft["object_path"]),
            mass_kg=float(spacecraft["mass_kg"]),
            inertia={k: float(v) for k, v
                     in _require(spacecraft, "inertia_kg_m2").items()},
            altitude_km=altitude_km,
            orbit={k: float(v) for k, v in orbit.items()
                   if k != "propagator"},
            propagator=str(orbit.get("propagator", "HPOP")),
            sensor=SensorSpec(
                name=str(sensor["name"]),
                object_path=str(sensor["object_path"]),
                half_angle_deg=float(sensor["half_angle_deg"]),
                boresight_body_axis=tuple(
                    float(v) for v in sensor["boresight_body_axis"]),
            ),
            actuators=ActuatorSpec(
                max_torque_Nm=float(wheels["max_torque_Nm"]),
                momentum_capacity_Nms=float(wheels["momentum_capacity_Nms"]),
                configuration=str(wheels.get("configuration",
                                             "orthogonal_3axis")),
            ),
            pointing=PointingSpec(
                capture_error_deg=float(pointing["capture_error_deg"]),
                settle_hold_s=float(pointing["settle_hold_s"]),
                settle_entry_deg=float(pointing["settle_entry_deg"]),
                max_body_rate_deg_s=float(pointing["max_body_rate_deg_s"]),
            ),
            scheduling=SchedulingSpec(
                settle_estimate_s=float(scheduling["settle_estimate_s"]),
                hold_before_image_s=float(scheduling["hold_before_image_s"]),
                return_to_nadir_between_targets=bool(
                    scheduling.get("return_to_nadir_between_targets", False)),
            ),
            targets=[
                TargetSpec(
                    id=str(t["id"]),
                    t_image_s=float(t["t_image_s"]),
                    cross_track_km=float(t["cross_track_km"]),
                    priority=float(t.get("priority", 1.0)),
                )
                for t in _require(targets_block, "list")
            ],
            ground_station=dict(raw.get("ground_station") or {}),
            bridge=dict(raw.get("bridge") or {}),
        )
        spec.validate()
        return spec

    # ---------------------------------------------------------------
    # Derived quantities
    # ---------------------------------------------------------------

    def slew_limits(self, axis: str = "ixx") -> SlewLimits:
        """Slew limits about one principal axis.

        Defaults to ``ixx`` because cross-track slews -- the only kind
        this study commands -- are rotations about the body +X axis (see
        the body-frame convention in acav_scenario.yaml). Using the
        largest inertia is also the conservative choice: it gives the
        slowest slew, so the scheduler never promises more agility than
        the vehicle has.
        """
        if axis not in self.inertia:
            raise KeyError(
                f"unknown inertia axis {axis!r}; have {list(self.inertia)}")
        return SlewLimits.from_axis(
            max_torque_Nm=self.actuators.max_torque_Nm,
            momentum_capacity_Nms=self.actuators.momentum_capacity_Nms,
            inertia_kg_m2=self.inertia[axis],
        )

    def slew_between_deg(self, cross_track_a_km: float,
                         cross_track_b_km: float) -> float:
        return math.degrees(slew_between_targets_rad(
            cross_track_a_km, cross_track_b_km, self.altitude_km))

    def estimated_slew_time_s(self, cross_track_a_km: float,
                              cross_track_b_km: float,
                              include_settle: bool = True) -> float:
        """Planning estimate of the time to reposition between targets."""
        delta = slew_between_targets_rad(cross_track_a_km, cross_track_b_km,
                                         self.altitude_km)
        seconds = slew_time_s(delta, self.slew_limits())
        if include_settle:
            seconds += self.scheduling.settle_estimate_s
        return seconds

    def target_by_id(self, target_id: str) -> TargetSpec:
        for target in self.targets:
            if target.id == target_id:
                return target
        raise KeyError(f"no target {target_id!r}; have "
                       f"{[t.id for t in self.targets]}")

    def feasibility_table(self) -> List[Dict]:
        """Per-target planning estimates, in scheduled order.

        Informational: what a dynamics-aware scheduler would compute
        before any simulation runs. C6 reports this alongside measured
        outcomes so the reader can see which targets were *predicted*
        marginal versus which actually failed -- a disagreement between
        the two is itself interesting and must not be hidden.
        """
        ordered = sorted(self.targets, key=lambda t: t.t_image_s)
        rows: List[Dict] = []
        previous_cross_track = 0.0        # start at nadir
        previous_release = 0.0            # window start

        for target in ordered:
            slew_deg = self.slew_between_deg(previous_cross_track,
                                             target.cross_track_km)
            required = self.estimated_slew_time_s(previous_cross_track,
                                                  target.cross_track_km)
            available = target.t_image_s - previous_release
            rows.append({
                "id": target.id,
                "t_image_s": target.t_image_s,
                "cross_track_km": target.cross_track_km,
                "off_nadir_deg": target.off_nadir_deg(self.altitude_km),
                "slew_from_previous_deg": slew_deg,
                "required_s": required,
                "available_s": available,
                "margin_s": available - required,
                "predicted_feasible": available >= required,
            })
            previous_cross_track = target.cross_track_km
            previous_release = target.t_image_s

        return rows

    # ---------------------------------------------------------------
    # Validation
    # ---------------------------------------------------------------

    def validate(self) -> None:
        """Cross-check duplicated and derived quantities. Raises on error."""
        problems: List[str] = []

        if abs(self.pointing.settle_hold_s
               - self.scheduling.hold_before_image_s) > 1e-9:
            problems.append(
                f"pointing.settle_hold_s ({self.pointing.settle_hold_s}) and "
                f"scheduling.hold_before_image_s "
                f"({self.scheduling.hold_before_image_s}) describe the same "
                f"physical quantity and must agree; the twin and the "
                f"scheduler would otherwise disagree about what a capture is"
            )

        if self.pointing.capture_error_deg >= self.sensor.half_angle_deg:
            problems.append(
                f"pointing.capture_error_deg "
                f"({self.pointing.capture_error_deg}) is not smaller than "
                f"sensor.half_angle_deg ({self.sensor.half_angle_deg}); "
                f"pointing error would be permitted to exceed the field of "
                f"view, making the capture criterion vacuous"
            )

        if self.pointing.settle_entry_deg < self.pointing.capture_error_deg:
            problems.append(
                f"pointing.settle_entry_deg ({self.pointing.settle_entry_deg}) "
                f"is tighter than capture_error_deg "
                f"({self.pointing.capture_error_deg}); the SLEW->SETTLE "
                f"transition would be harder to satisfy than capture itself"
            )

        if self.altitude_km <= 0:
            problems.append(
                f"derived altitude is {self.altitude_km:.1f} km; check "
                f"orbit.semi_major_axis_km")

        ids = [t.id for t in self.targets]
        if len(ids) != len(set(ids)):
            problems.append(f"duplicate target ids: {ids}")

        for target in self.targets:
            if not (0.0 <= target.t_image_s <= self.window_duration_s):
                problems.append(
                    f"target {target.id} images at t={target.t_image_s}s, "
                    f"outside the {self.window_duration_s}s window")
            if target.t_image_s < self.pointing.settle_hold_s:
                problems.append(
                    f"target {target.id} images at t={target.t_image_s}s, "
                    f"less than settle_hold_s "
                    f"({self.pointing.settle_hold_s}s) after window start -- "
                    f"its hold window would extend before the run begins")

        # Sensing must be fast enough to resolve the hold window. A hold
        # window shorter than a few sensing periods cannot be evaluated
        # meaningfully, and would silently make capture a coin flip.
        fast_hz = float((self.bridge.get("rate_groups") or {}).get("fast", 10.0))
        samples_in_hold = self.pointing.settle_hold_s * fast_hz
        if samples_in_hold < 5:
            problems.append(
                f"the fast rate group ({fast_hz} Hz) yields only "
                f"{samples_in_hold:.1f} samples in the "
                f"{self.pointing.settle_hold_s}s hold window; at least 5 are "
                f"needed to evaluate the capture criterion meaningfully")

        if problems:
            raise ValueError(
                "ACAV scenario specification is inconsistent:\n  - "
                + "\n  - ".join(problems))

    def summary(self) -> str:
        limits = self.slew_limits()
        lines = [
            f"ACAV spec: {self.name}",
            f"  window        {self.window_duration_s:.0f} s from "
            f"{self.window_start_utc}, step {self.step_s} s "
            f"({int(self.window_duration_s / self.step_s)} steps)",
            f"  spacecraft    {self.spacecraft_name}, {self.mass_kg} kg, "
            f"alt {self.altitude_km:.1f} km, {self.propagator}",
            f"  sensor        {self.sensor.half_angle_deg} deg half-angle "
            f"({self._footprint_radius_km():.1f} km footprint radius)",
            f"  slew limits   alpha={limits.alpha_deg_s2:.3f} deg/s^2, "
            f"omega_max={limits.omega_max_deg_s:.2f} deg/s, "
            f"crossover={limits.crossover_deg:.1f} deg",
            f"  capture       <= {self.pointing.capture_error_deg} deg held "
            f"for {self.pointing.settle_hold_s} s, plus STK access",
            f"  targets       {len(self.targets)}",
        ]
        for row in self.feasibility_table():
            flag = "ok      " if row["predicted_feasible"] else "MARGINAL"
            lines.append(
                f"    {row['id']:<3} t={row['t_image_s']:6.1f}s  "
                f"xt={row['cross_track_km']:+7.1f}km  "
                f"slew={row['slew_from_previous_deg']:5.1f}deg  "
                f"need={row['required_s']:5.1f}s  "
                f"have={row['available_s']:6.1f}s  "
                f"margin={row['margin_s']:+6.1f}s  {flag}")
        return "\n".join(lines)

    def _footprint_radius_km(self) -> float:
        return self.altitude_km * math.tan(
            math.radians(self.sensor.half_angle_deg))


def _require(mapping: Dict, key: str) -> Dict:
    if key not in mapping:
        raise KeyError(
            f"required section {key!r} missing from the ACAV scenario spec")
    return mapping[key]
