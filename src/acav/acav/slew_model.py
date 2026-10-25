"""Rest-to-rest slew time model, shared by the scheduler and the twin.

Why one model
-------------
Experiment E2 compares a *kinematic* schedule (assumes instantaneous
reorientation) against a *dynamics-aware* one (budgets slew and settle
time). The comparison is only meaningful if the dynamics-aware
scheduler's notion of "how long does a slew take" matches what the
twin achieves. With two independently written slew models, E2 would
measure their disagreement rather than the effect under study.

This module is therefore the single source of truth. The scheduler
calls it directly; the twin's reference profile
(``acav/twin_reference.py``, ``+adcs/slew_profile.m``) is built from the
same model and parameters, and its achieved slew times are checked
against this bound (see ``profile_error`` below) rather than assumed to
match.

The model
---------
Rest-to-rest, single-axis, torque- and momentum-limited. Given angular
acceleration limit ``alpha = tau_max / J`` and rate limit
``omega_max = h_max / J``:

* **Triangular** (accelerate to peak, immediately decelerate) when the
  peak rate stays within the limit, i.e. ``dtheta <= omega_max^2 / alpha``::

      t = 2 * sqrt(dtheta / alpha)

* **Trapezoidal** (accelerate, coast at omega_max, decelerate) otherwise::

      t = omega_max / alpha + dtheta / omega_max

The two agree at the crossover ``dtheta = omega_max^2 / alpha``, both
giving ``2 * omega_max / alpha`` (checked in test_slew_model.py), so the
scheduler's feasibility boundary has no step.

For the ACAV parameters (ixx = 0.123 kg m^2, tau = 2 mNm, h = 20 mNms)
the crossover is ~93 deg, so every slew in the study is triangular. The
trapezoidal branch is implemented and tested for other wheel sizings.

What this model does not capture
--------------------------------
Controller settling, flexible modes, wheel friction/stiction, and
cross-axis coupling. It is a *lower bound* on achievable slew time, the
best a perfect controller could do against these limits. The scheduler
therefore adds ``settle_estimate_s`` on top (see
``scheduling.settle_estimate_s`` in acav_scenario.yaml). The gap between
this bound and the twin's achieved time is itself a result, so the model
is not adjusted to fit measurements.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class SlewLimits:
    """Single-axis torque/momentum limits, expressed as rate limits.

    Constructed from the inertia and actuator entries in
    acav_scenario.yaml via :meth:`from_spec`; the derived values in that
    file's comments are recomputed here rather than trusted.
    """

    #: Angular acceleration limit, rad/s^2.
    alpha_rad_s2: float
    #: Angular rate limit, rad/s.
    omega_max_rad_s: float

    def __post_init__(self) -> None:
        if self.alpha_rad_s2 <= 0.0:
            raise ValueError(
                f"alpha must be > 0, got {self.alpha_rad_s2}. A zero or "
                f"negative acceleration limit usually means max_torque_Nm "
                f"or the inertia entry is missing from the scenario YAML."
            )
        if self.omega_max_rad_s <= 0.0:
            raise ValueError(
                f"omega_max must be > 0, got {self.omega_max_rad_s}"
            )

    @classmethod
    def from_axis(cls, max_torque_Nm: float, momentum_capacity_Nms: float,
                  inertia_kg_m2: float) -> "SlewLimits":
        """Build limits for one principal axis."""
        if inertia_kg_m2 <= 0.0:
            raise ValueError(f"inertia must be > 0, got {inertia_kg_m2}")
        return cls(
            alpha_rad_s2=max_torque_Nm / inertia_kg_m2,
            omega_max_rad_s=momentum_capacity_Nms / inertia_kg_m2,
        )

    @property
    def crossover_rad(self) -> float:
        """Slew angle at which the profile changes triangular->trapezoidal."""
        return self.omega_max_rad_s ** 2 / self.alpha_rad_s2

    @property
    def crossover_deg(self) -> float:
        return math.degrees(self.crossover_rad)

    @property
    def alpha_deg_s2(self) -> float:
        return math.degrees(self.alpha_rad_s2)

    @property
    def omega_max_deg_s(self) -> float:
        return math.degrees(self.omega_max_rad_s)


def slew_time_s(delta_theta_rad: float, limits: SlewLimits) -> float:
    """Minimum rest-to-rest slew time for ``delta_theta_rad``.

    Args:
        delta_theta_rad: Slew magnitude, radians. Sign is irrelevant;
            the absolute value is used.
        limits: Axis limits.

    Returns:
        Time in seconds. Zero for a zero-magnitude slew.
    """
    theta = abs(float(delta_theta_rad))
    if theta == 0.0:
        return 0.0

    if theta <= limits.crossover_rad:
        return 2.0 * math.sqrt(theta / limits.alpha_rad_s2)

    return (limits.omega_max_rad_s / limits.alpha_rad_s2
            + theta / limits.omega_max_rad_s)


def slew_time_deg_s(delta_theta_deg: float, limits: SlewLimits) -> float:
    """Convenience wrapper taking degrees."""
    return slew_time_s(math.radians(delta_theta_deg), limits)


def peak_rate_rad_s(delta_theta_rad: float, limits: SlewLimits) -> float:
    """Peak angular rate reached during the slew, rad/s.

    Used to check whether a commanded slew would saturate the wheels,
    the same condition that triggers SAFE mode and that the saturation
    fault injects.
    """
    theta = abs(float(delta_theta_rad))
    if theta <= limits.crossover_rad:
        return math.sqrt(limits.alpha_rad_s2 * theta)
    return limits.omega_max_rad_s


def is_triangular(delta_theta_rad: float, limits: SlewLimits) -> bool:
    return abs(float(delta_theta_rad)) <= limits.crossover_rad


def off_nadir_angle_rad(cross_track_km: float, altitude_km: float,
                        earth_radius_km: float = 6378.137) -> float:
    """Off-nadir angle to a ground point ``cross_track_km`` from nadir.

    Spherical Earth, exact (not the small-angle ``atan(d/h)``
    approximation). Used to turn the scenario's ``cross_track_km``
    target offsets into slew magnitudes for planning.

    This is a *planning estimate only*. STK computes the true geometry
    including oblateness and terrain; where the two disagree, STK is
    correct by definition for this study; the scheduler is allowed to
    be approximate, which is part of what E2 measures.
    """
    if altitude_km <= 0.0:
        raise ValueError(f"altitude must be > 0, got {altitude_km}")
    lam = abs(float(cross_track_km)) / earth_radius_km      # central angle
    numerator = earth_radius_km * math.sin(lam)
    denominator = (earth_radius_km + altitude_km
                   - earth_radius_km * math.cos(lam))
    return math.atan2(numerator, denominator)


def off_nadir_angle_deg(cross_track_km: float, altitude_km: float,
                        earth_radius_km: float = 6378.137) -> float:
    return math.degrees(
        off_nadir_angle_rad(cross_track_km, altitude_km, earth_radius_km))


def slew_between_targets_rad(cross_track_a_km: float, cross_track_b_km: float,
                             altitude_km: float) -> float:
    """Slew magnitude between two cross-track offsets.

    Targets on opposite sides of the ground track require a slew equal
    to the SUM of their off-nadir angles; on the same side, the
    difference. Getting this sign handling wrong would systematically
    under-estimate the hardest slews in the scenario.
    """
    angle_a = off_nadir_angle_rad(cross_track_a_km, altitude_km)
    angle_b = off_nadir_angle_rad(cross_track_b_km, altitude_km)
    same_side = (cross_track_a_km >= 0.0) == (cross_track_b_km >= 0.0)
    return abs(angle_a - angle_b) if same_side else angle_a + angle_b


def profile_error(measured_s: float, delta_theta_rad: float,
                  limits: SlewLimits) -> float:
    """Fractional excess of a measured slew time over the model bound.

    ``(measured - model) / model``. Always >= 0 for a physically valid
    measurement, because the model is a lower bound; a negative value
    means the twin exceeded its stated torque/momentum limits and is a
    bug, not a fast controller.
    """
    model = slew_time_s(delta_theta_rad, limits)
    if model <= 0.0:
        return 0.0
    return (float(measured_s) - model) / model
