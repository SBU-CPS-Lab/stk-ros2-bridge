"""Executable reference for the Simulink ADCS twin (C3).

The twin itself is MATLAB/Simulink and runs on the Windows machine.
This module is the same dynamics, control law, actuator model and mode
logic written once in Python, where it can be tested, and it exists for
three reasons:

1. **The MATLAB cannot be tested here.** Shipping a thousand lines of
   untested attitude code and finding the sign errors during a live
   closed-loop run is the expensive order. The MATLAB is a transcription
   of semantics fixed and tested here.
2. **The two must not drift.** ``scripts/export_twin_fixtures.py``
   writes trajectories from this module; ``stk_matlab/test_acav_twin.m``
   replays them through the MATLAB implementation and asserts agreement.
   A transcription error becomes a failing MATLAB test rather than a
   result.
3. **C5 needs something to validate the twin against.** Experiment E2
   only measures the effect under study if the twin and the scheduler
   share a slew model; C1 made that point about ``slew_model.py`` and it
   applies with more force to the closed-loop response.

What this is not
----------------
Not a substitute for the Simulink model, and no number it produces may
appear in the paper. It has no flexible modes, no wheel friction, no
sensor noise and no Stateflow. It fixes semantics; the twin produces
results.

Design of the control path
--------------------------
A saturated PD controller on a double integrator overshoots badly, and
the overshoot would show up as settling time the scheduler never
budgeted for -- E2 would then be measuring the controller's overshoot
rather than the effect under study. So the twin does what an agile EO
spacecraft actually does: an **eigenaxis profile generator** produces a
feasible reference trajectory using exactly the limits
``acav/slew_model.py`` assumes, and a quaternion-feedback controller
*tracks* it. The achieved slew time is then close to the analytic bound
by construction, and the interesting quantity -- settling -- is what
remains.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Sequence, Tuple

from acav.attitude import (
    Quaternion,
    Vector3,
    axis_angle_from_quat,
    boresight_error_rad,
    cross,
    dot,
    normalize,
    quat_align,
    quat_derivative,
    quat_error,
    quat_from_axis_angle,
    quat_multiply,
    quat_normalize,
    rotate_to_inertial,
    scale,
)
from acav.slew_model import SlewLimits, slew_time_s


class MissionMode(str, Enum):
    """The Stateflow chart's states. Values match the chart's labels."""

    NADIR = "NADIR"
    SLEW = "SLEW"
    SETTLE = "SETTLE"
    IMAGE = "IMAGE"
    SAFE = "SAFE"


@dataclass(frozen=True)
class TwinConfig:
    """Everything the twin needs, all of it from ``acav_scenario.yaml``.

    Nothing here has a default that differs from the YAML: a value that
    could drift between the twin and the scheduler is a value that makes
    E2 measure their disagreement instead of the effect under study.
    """

    inertia_kg_m2: Tuple[float, float, float]
    max_torque_Nm: float
    momentum_capacity_Nms: float
    boresight_body: Vector3 = (0.0, 0.0, 1.0)
    settle_entry_rad: float = math.radians(2.0)
    capture_error_rad: float = math.radians(0.5)
    settle_hold_s: float = 2.0
    max_body_rate_rad_s: float = math.radians(15.0)
    #: Closed-loop natural frequency and damping of the tracking loop.
    #: Measured, not guessed: a sweep over authority and bandwidth put
    #: the excess of achieved settling over the analytic slew bound at
    #: +3.7 to +4.8 s across every slew in the scenario at these values,
    #: against the 8 s the scheduler budgets. At 0.7 rad/s the largest
    #: slew needed +9.8 s and would have missed every tight target for
    #: a reason that has nothing to do with the effect under study.
    #: E4 sweeps these, which is why they are here and not in the model.
    #:
    #: Honest caveat for the paper: this bandwidth is optimistic for a
    #: 6U CubeSat because the twin models no sensor noise, no attitude
    #: estimation lag and no wheel dynamics. It is a fair basis for
    #: comparing schedulers -- both face the same vehicle -- but it is
    #: not a claim about achievable CubeSat pointing performance.
    natural_frequency_rad_s: float = 1.0
    damping_ratio: float = 0.707
    #: Fraction of the wheel limits the reference profile is allowed to
    #: use. At 1.0 the feedforward term alone demands every newton-metre
    #: the wheels have, so the feedback loop has nothing left to correct
    #: tracking error with and the residual only decays once the profile
    #: stops asking. Holding back 20% costs ~12% on the slew itself and
    #: buys back several seconds of settling.
    profile_authority: float = 0.8

    @classmethod
    def from_spec(cls, spec) -> "TwinConfig":
        return cls(
            inertia_kg_m2=(spec.inertia["ixx"], spec.inertia["iyy"],
                           spec.inertia["izz"]),
            max_torque_Nm=spec.actuators.max_torque_Nm,
            momentum_capacity_Nms=spec.actuators.momentum_capacity_Nms,
            boresight_body=tuple(spec.sensor.boresight_body_axis),
            settle_entry_rad=math.radians(spec.pointing.settle_entry_deg),
            capture_error_rad=math.radians(spec.pointing.capture_error_deg),
            settle_hold_s=spec.pointing.settle_hold_s,
            max_body_rate_rad_s=math.radians(
                spec.pointing.max_body_rate_deg_s),
        )

    def gains(self) -> Tuple[Vector3, Vector3]:
        """Per-axis (Kp, Kd) for the tracking loop.

        ``J w_n^2`` and ``2 zeta w_n J`` per axis, so every axis has the
        same closed-loop bandwidth despite the inertia being far from
        isotropic (0.123 / 0.096 / 0.041). Using one scalar gain for all
        three would make the z axis three times faster than the x axis
        and the response direction-dependent for no reason.
        """
        w_n, zeta = self.natural_frequency_rad_s, self.damping_ratio
        kp = tuple(j * w_n * w_n for j in self.inertia_kg_m2)
        kd = tuple(2.0 * zeta * w_n * j for j in self.inertia_kg_m2)
        return kp, kd

    def slew_limits(self, axis: int = 0) -> SlewLimits:
        """The vehicle's actuator limits -- the scheduler's model.

        Defaults to the x axis: the largest inertia, and the axis every
        cross-track slew in this study turns about. Deliberately the
        same call ``AcavSpec.slew_limits`` makes, so the scheduler's
        estimate and the twin's actuators come from one model.
        """
        return SlewLimits.from_axis(self.max_torque_Nm,
                                    self.momentum_capacity_Nms,
                                    self.inertia_kg_m2[axis])

    def profile_limits(self, axis: int = 0) -> SlewLimits:
        """The limits the reference trajectory obeys -- derated.

        Distinct from :meth:`slew_limits` on purpose. The scheduler's
        estimate is a *lower bound* computed against the full limits, as
        C1 says; the twin flies a profile that deliberately leaves
        control margin, so it takes slightly longer than the bound. That
        gap is part of what ``scheduling.settle_estimate_s`` covers, and
        collapsing the two would either starve the controller or make
        the scheduler's bound unattainable in principle.
        """
        full = self.slew_limits(axis)
        return SlewLimits(
            alpha_rad_s2=full.alpha_rad_s2 * self.profile_authority,
            omega_max_rad_s=full.omega_max_rad_s * self.profile_authority)


# ---------------------------------------------------------------------------
# Actuators
# ---------------------------------------------------------------------------

@dataclass
class ReactionWheels:
    """Three orthogonal wheels aligned with the body axes.

    Torque- and momentum-limited, and the momentum limit is the one that
    matters: it is what trips SAFE mode, and it is the fault E5 injects.
    Saturation is modelled as a hard clamp rather than a soft roll-off
    because that is what makes the failure abrupt and therefore
    detectable; a gentle roll-off would degrade pointing gradually and
    look like a tuning problem.
    """

    max_torque_Nm: float
    momentum_capacity_Nms: float
    momentum_Nms: Vector3 = (0.0, 0.0, 0.0)
    #: Per-axis derate, 1.0 = healthy. E5 drives one of these to zero.
    health: Vector3 = (1.0, 1.0, 1.0)

    def command(self, torque_body_Nm: Sequence[float], dt: float
                ) -> Tuple[Vector3, bool]:
        """Apply a body torque request. Returns (delivered, saturated).

        The wheel absorbs the reaction, so its momentum changes by
        ``-torque * dt``. A wheel already at its momentum limit can still
        torque *back* towards zero -- clamping both directions would make
        saturation unrecoverable and turn every E5 run into a permanent
        SAFE, which would not be a fault injection so much as a
        termination.
        """
        delivered = []
        momentum = []
        saturated = False

        for axis in range(3):
            limit = self.max_torque_Nm * self.health[axis]
            request = max(-limit, min(limit, torque_body_Nm[axis]))
            if abs(torque_body_Nm[axis]) > limit + 1e-15:
                saturated = True

            new_momentum = self.momentum_Nms[axis] - request * dt
            if abs(new_momentum) > self.momentum_capacity_Nms:
                clamped = math.copysign(self.momentum_capacity_Nms,
                                        new_momentum)
                # Only the part of the request the wheel could absorb
                # actually reaches the body.
                request = (self.momentum_Nms[axis] - clamped) / dt \
                    if dt > 0.0 else 0.0
                new_momentum = clamped
                saturated = True

            delivered.append(request)
            momentum.append(new_momentum)

        self.momentum_Nms = (momentum[0], momentum[1], momentum[2])
        return (delivered[0], delivered[1], delivered[2]), saturated

    @property
    def momentum_fraction(self) -> float:
        """Largest per-axis momentum as a fraction of capacity."""
        return max(abs(h) for h in self.momentum_Nms) / \
            self.momentum_capacity_Nms


# ---------------------------------------------------------------------------
# Rigid-body dynamics
# ---------------------------------------------------------------------------

@dataclass
class RigidBody:
    """Attitude state and its propagation.

    Euler's equation with the wheel momentum included in the total, so
    gyroscopic coupling through a spinning wheel is present. It is small
    at these momenta, but omitting it would make the twin's dynamics
    differ from a real vehicle's in exactly the regime E5 explores.
    """

    inertia_kg_m2: Tuple[float, float, float]
    quaternion: Quaternion = (0.0, 0.0, 0.0, 1.0)
    rate_rad_s: Vector3 = (0.0, 0.0, 0.0)

    def derivative(self, torque_body_Nm: Sequence[float],
                   wheel_momentum_Nms: Sequence[float]
                   ) -> Tuple[Quaternion, Vector3]:
        j = self.inertia_kg_m2
        w = self.rate_rad_s
        total = (j[0] * w[0] + wheel_momentum_Nms[0],
                 j[1] * w[1] + wheel_momentum_Nms[1],
                 j[2] * w[2] + wheel_momentum_Nms[2])
        gyroscopic = cross(w, total)
        rate_dot = tuple((torque_body_Nm[k] - gyroscopic[k]) / j[k]
                         for k in range(3))
        return quat_derivative(self.quaternion, w), rate_dot

    def step(self, torque_body_Nm: Sequence[float],
             wheel_momentum_Nms: Sequence[float], dt: float) -> None:
        """One RK4 step.

        RK4 rather than Euler: at 0.1 s with a 0.7 rad/s bandwidth,
        forward Euler adds a slow energy drift that reads as a
        controller that never quite settles. That is precisely the
        measurement this study reports, so the integrator must not
        contribute to it.
        """
        q0, w0 = self.quaternion, self.rate_rad_s

        def evaluate(dq, dw, weight):
            self.quaternion = tuple(q0[k] + dq[k] * weight for k in range(4))
            self.rate_rad_s = tuple(w0[k] + dw[k] * weight for k in range(3))
            return self.derivative(torque_body_Nm, wheel_momentum_Nms)

        k1q, k1w = self.derivative(torque_body_Nm, wheel_momentum_Nms)
        k2q, k2w = evaluate(k1q, k1w, dt / 2.0)
        k3q, k3w = evaluate(k2q, k2w, dt / 2.0)
        k4q, k4w = evaluate(k3q, k3w, dt)

        self.quaternion = quat_normalize(tuple(
            q0[k] + dt / 6.0 * (k1q[k] + 2 * k2q[k] + 2 * k3q[k] + k4q[k])
            for k in range(4)))
        self.rate_rad_s = tuple(
            w0[k] + dt / 6.0 * (k1w[k] + 2 * k2w[k] + 2 * k3w[k] + k4w[k])
            for k in range(3))


# ---------------------------------------------------------------------------
# Eigenaxis slew profile
# ---------------------------------------------------------------------------

@dataclass
class SlewProfile:
    """Rest-to-rest eigenaxis reference trajectory.

    Angle, rate and the reference quaternion at any time during the
    slew, following the same triangular/trapezoidal profile
    ``acav/slew_model.py`` predicts the duration of. Sharing the model
    rather than the number is what makes the scheduler's estimate and
    the twin's behaviour comparable at all.
    """

    start: Quaternion
    finish: Quaternion
    axis_body: Vector3
    total_angle_rad: float
    limits: SlewLimits
    duration_s: float

    @classmethod
    def between(cls, start: Sequence[float], finish: Sequence[float],
                limits: SlewLimits) -> "SlewProfile":
        axis, angle = axis_angle_from_quat(quat_error(finish, start))
        return cls(start=quat_normalize(start), finish=quat_normalize(finish),
                   axis_body=axis, total_angle_rad=angle, limits=limits,
                   duration_s=slew_time_s(angle, limits))

    def angle_at(self, elapsed_s: float) -> Tuple[float, float]:
        """(angle travelled, rate) at ``elapsed_s`` into the slew."""
        theta = self.total_angle_rad
        if theta <= 0.0 or elapsed_s <= 0.0:
            return 0.0, 0.0
        if elapsed_s >= self.duration_s:
            return theta, 0.0

        alpha = self.limits.alpha_rad_s2
        omega_max = self.limits.omega_max_rad_s

        if theta <= self.limits.crossover_rad:
            half = self.duration_s / 2.0
            if elapsed_s <= half:
                return 0.5 * alpha * elapsed_s ** 2, alpha * elapsed_s
            remaining = self.duration_s - elapsed_s
            return (theta - 0.5 * alpha * remaining ** 2, alpha * remaining)

        ramp = omega_max / alpha
        if elapsed_s <= ramp:
            return 0.5 * alpha * elapsed_s ** 2, alpha * elapsed_s
        if elapsed_s <= self.duration_s - ramp:
            return (0.5 * omega_max * ramp
                    + omega_max * (elapsed_s - ramp), omega_max)
        remaining = self.duration_s - elapsed_s
        return theta - 0.5 * alpha * remaining ** 2, alpha * remaining

    def reference_at(self, elapsed_s: float) -> Tuple[Quaternion, Vector3]:
        """Reference attitude and body rate at ``elapsed_s``."""
        angle, rate = self.angle_at(elapsed_s)
        q_ref = quat_multiply(self.start,
                              quat_from_axis_angle(self.axis_body, angle))
        return quat_normalize(q_ref), scale(self.axis_body, rate)

    @property
    def complete_at(self) -> float:
        return self.duration_s


# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------

def tracking_torque(config: TwinConfig, q: Sequence[float],
                    omega: Sequence[float], q_reference: Sequence[float],
                    omega_reference: Sequence[float]) -> Vector3:
    """Quaternion feedback with rate feedforward, body-frame torque.

    ``tau = -Kp * sign(qe_w) * qe_v - Kd * (omega - omega_ref)``

    The ``sign(qe_w)`` is not cosmetic: without it the controller happily
    takes the long way round a 350 degree error, arrives, and reports a
    settling time three times longer than the model predicts. The
    feedforward term is what lets the response track the profile rather
    than lag it, which is what keeps achieved slew times near the
    analytic bound.
    """
    kp, kd = config.gains()
    error = quat_error(q, q_reference)
    sign = 1.0 if error[3] >= 0.0 else -1.0
    return tuple(
        -kp[k] * sign * error[k] - kd[k] * (omega[k] - omega_reference[k])
        for k in range(3))


def nadir_attitude(position_inertial: Sequence[float],
                   velocity_inertial: Sequence[float],
                   boresight_body: Sequence[float] = (0.0, 0.0, 1.0)
                   ) -> Quaternion:
    """Boresight at nadir, body +X along the velocity direction."""
    nadir = normalize(tuple(-c for c in position_inertial))
    return quat_align(boresight_body, nadir, (1.0, 0.0, 0.0),
                      velocity_inertial)


def target_attitude(position_inertial: Sequence[float],
                    velocity_inertial: Sequence[float],
                    target_inertial: Sequence[float],
                    boresight_body: Sequence[float] = (0.0, 0.0, 1.0)
                    ) -> Quaternion:
    """Boresight at a ground target, roll pinned by the velocity.

    ``boresight_body`` comes from ``sensor.boresight_body_axis`` in the
    YAML and is the single place the +Z assumption enters the twin. If
    the live install turns out to mount the boresight the other way, one
    value in the scenario file changes and nothing else does.
    """
    line_of_sight = normalize(tuple(t - p for t, p
                                    in zip(target_inertial,
                                           position_inertial)))
    return quat_align(boresight_body, line_of_sight, (1.0, 0.0, 0.0),
                      velocity_inertial)


# ---------------------------------------------------------------------------
# Mission mode logic (the Stateflow chart, as a function)
# ---------------------------------------------------------------------------

@dataclass
class ModeState:
    """Everything the chart remembers between steps."""

    mode: MissionMode = MissionMode.NADIR
    entered_at_s: float = 0.0
    settled_since_s: Optional[float] = None
    profile: Optional[SlewProfile] = None
    slew_started_s: float = 0.0


def update_mode(state: ModeState, config: TwinConfig, now_s: float,
                boresight_error_rad_value: float, body_rate_rad_s: float,
                momentum_fraction: float,
                slew_requested: bool) -> ModeState:
    """One transition evaluation. Mirrors the Stateflow chart exactly.

    Ordering matters and is asserted by tests: the SAFE guards are
    evaluated **first and unconditionally**, so a wheel that saturates
    during IMAGE trips SAFE rather than being masked by an
    already-satisfied hold. Writing the transitions in the other order
    yields a chart that behaves correctly in every nominal run and
    silently fails to protect the vehicle in exactly the campaign (E5)
    that exists to test it.
    """
    if (momentum_fraction >= 1.0
            or body_rate_rad_s > config.max_body_rate_rad_s):
        if state.mode is not MissionMode.SAFE:
            return ModeState(mode=MissionMode.SAFE, entered_at_s=now_s)
        return state

    if state.mode is MissionMode.SAFE:
        # SAFE is latched. Recovery is an operator action, not an
        # automatic transition -- an ADCS that re-armed itself the
        # instant the trip condition cleared would oscillate in and out
        # of SAFE and make E5's replay meaningless.
        return state

    if slew_requested and state.mode in (MissionMode.NADIR,
                                         MissionMode.IMAGE):
        return ModeState(mode=MissionMode.SLEW, entered_at_s=now_s,
                         profile=state.profile, slew_started_s=now_s)

    if state.mode is MissionMode.SLEW:
        profile_done = (state.profile is None
                        or now_s - state.slew_started_s
                        >= state.profile.complete_at)
        if profile_done:
            return ModeState(mode=MissionMode.SETTLE, entered_at_s=now_s,
                             profile=state.profile,
                             slew_started_s=state.slew_started_s)
        return state

    if state.mode is MissionMode.SETTLE:
        if boresight_error_rad_value <= config.settle_entry_rad:
            return ModeState(mode=MissionMode.IMAGE, entered_at_s=now_s,
                             profile=state.profile,
                             slew_started_s=state.slew_started_s)
        return state

    return state


# ---------------------------------------------------------------------------
# A whole closed-loop run, for fixtures and offline checks
# ---------------------------------------------------------------------------

@dataclass
class TwinSample:
    """One logged step. Mirrors what C5 records from the real twin."""

    time_s: float
    quaternion: Quaternion
    rate_rad_s: Vector3
    boresight_error_rad: float
    torque_Nm: Vector3
    wheel_momentum_Nms: Vector3
    mode: str


@dataclass
class TwinRun:
    samples: List[TwinSample] = field(default_factory=list)

    @property
    def settle_time_s(self) -> Optional[float]:
        """First time the boresight error stays inside capture tolerance."""
        return None if not self.samples else self.samples[-1].time_s

    def error_deg_at(self, time_s: float) -> float:
        best = min(self.samples, key=lambda s: abs(s.time_s - time_s))
        return math.degrees(best.boresight_error_rad)


def simulate_slew(config: TwinConfig, q_initial: Sequence[float],
                  q_target: Sequence[float], duration_s: float,
                  dt: float = 0.01) -> TwinRun:
    """Closed-loop rest-to-rest slew, no orbital motion.

    Deliberately inertially fixed targets: this is the *reference*
    implementation, used to pin semantics and generate cross-language
    fixtures. Orbital motion, STK access and the scheduler all live in
    the real loop.
    """
    body = RigidBody(inertia_kg_m2=config.inertia_kg_m2,
                     quaternion=quat_normalize(q_initial))
    wheels = ReactionWheels(max_torque_Nm=config.max_torque_Nm,
                            momentum_capacity_Nms=config.momentum_capacity_Nms)
    profile = SlewProfile.between(q_initial, q_target,
                                  config.profile_limits())
    state = ModeState(mode=MissionMode.SLEW, profile=profile)

    run = TwinRun()
    steps = int(round(duration_s / dt))
    for index in range(steps + 1):
        now = index * dt
        q_ref, omega_ref = profile.reference_at(now)
        error = boresight_error_rad(body.quaternion, q_target,
                                    config.boresight_body)
        rate_magnitude = math.sqrt(dot(body.rate_rad_s, body.rate_rad_s))

        state = update_mode(state, config, now, error, rate_magnitude,
                            wheels.momentum_fraction, slew_requested=False)

        torque = tracking_torque(config, body.quaternion, body.rate_rad_s,
                                 q_ref, omega_ref)
        if state.mode is MissionMode.SAFE:
            torque = (0.0, 0.0, 0.0)
        delivered, _ = wheels.command(torque, dt)

        run.samples.append(TwinSample(
            time_s=now, quaternion=body.quaternion,
            rate_rad_s=body.rate_rad_s, boresight_error_rad=error,
            torque_Nm=delivered, wheel_momentum_Nms=wheels.momentum_Nms,
            mode=state.mode.value))

        body.step(delivered, wheels.momentum_Nms, dt)

    return run


def settling_time_s(run: TwinRun, tolerance_rad: float,
                    hold_s: float) -> Optional[float]:
    """When the boresight error first stays inside tolerance for hold_s.

    Returns the time at the *start* of the sustained interval, which is
    the quantity the scheduler needs: a target imaged at ``t_i`` needs
    the error to have been inside tolerance since ``t_i - hold_s``.
    ``None`` means it never did -- distinct from zero, and the two must
    not be conflated when averaging over a campaign.
    """
    entered: Optional[float] = None
    for sample in run.samples:
        if sample.boresight_error_rad <= tolerance_rad:
            if entered is None:
                entered = sample.time_s
            elif sample.time_s - entered >= hold_s:
                return entered
        else:
            entered = None
    return None
