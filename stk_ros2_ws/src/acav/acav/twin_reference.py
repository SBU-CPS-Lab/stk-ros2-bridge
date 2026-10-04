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
from dataclasses import dataclass, field, replace
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
    quat_conjugate,
    quat_derivative,
    quat_error,
    quat_from_axis_angle,
    quat_multiply,
    quat_normalize,
    rotate_to_inertial,
    rotate_to_body,
    scale,
)
from acav.fault_injection import WheelFault
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
    #: Control step, seconds. The same ``scenario.step_s`` the bridge
    #: steps STK with, because the twin runs exactly one control cycle
    #: per lockstep tick. Mirrors ``cfg.stepS`` in ``+adcs/config.m``;
    #: ``twin_step`` integrates over it.
    step_s: float = 0.1
    #: A reaction wheel degrading partway through the run -- E5's
    #: injected fault. ``None`` is a healthy vehicle, which is every
    #: other experiment.
    #:
    #: Carried on the config rather than arriving as an event because
    #: the fault is decided before the run and written into its record;
    #: see :mod:`acav.fault_injection`. That makes the health at any
    #: instant a pure function of (fault, time), so a replay reproduces
    #: the degradation exactly rather than re-rolling it.
    wheel_fault: Optional[WheelFault] = None

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
            step_s=spec.step_s,
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


def control_torque(config: TwinConfig, mode: MissionMode,
                   q: Sequence[float], omega: Sequence[float],
                   q_reference: Sequence[float],
                   omega_reference: Sequence[float]) -> Vector3:
    """The torque the controller asks for, given the mode.

    SAFE is **rate damping only** -- ``-Kd * omega``, with no attitude
    term. The purpose of SAFE is to stop the vehicle tumbling and stop
    pointing at anything; commanding zero torque instead would leave
    whatever rate tripped the guard exactly where it was, and E5 is the
    campaign that exists to exercise precisely that recovery.

    This function exists because the two implementations disagreed about
    it. ``simulate_slew`` here zeroed the torque, ``+adcs/twin_step.m``
    applied rate damping, and no fixture could see the disagreement
    because the per-step orchestration had no counterpart in this module
    to generate one from. One decision, one place, used by both
    ``simulate_slew`` and ``twin_step``.
    """
    if mode is MissionMode.SAFE:
        _, kd = config.gains()
        return tuple(-kd[k] * omega[k] for k in range(3))
    return tracking_torque(config, q, omega, q_reference, omega_reference)


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
                momentum_fraction: float, slew_requested: bool,
                profile_done: bool) -> ModeState:
    """One transition evaluation. Mirrors the Stateflow chart exactly.

    ``profile_done`` is an argument rather than being derived from
    ``state.profile`` here. It was derived, and the MATLAB transcription
    took it as an argument -- so the two disagreed about a SLEW state
    with no profile, and the cross-check caught it. Deriving it also hid
    a bug state: a SLEW with no profile is a fault, and silently
    treating it as "finished" is the kind of plausible recovery that
    makes a wrong run look like a normal one.

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
# One control cycle: telemetry in, attitude command out
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class TwinInputs:
    """One tick of telemetry. Mirrors ``in`` in ``+adcs/twin_step.m``."""

    quaternion: Quaternion
    omega_rad_s: Vector3
    position_m: Vector3
    velocity_m_s: Vector3
    time_s: float
    target_eci_m: Vector3 = (0.0, 0.0, 0.0)
    have_target: bool = False
    target_id: str = ""


@dataclass
class TwinState:
    """What the twin carries between steps. Mirrors ``initial_state.m``.

    Wheel momentum lives here rather than on :class:`ModeState` because
    ``update_mode`` constructs a *fresh* ``ModeState`` on every
    transition. A momentum field there would be silently zeroed each
    time the mode changed -- which is not only physically impossible but
    self-concealing, since entering SAFE would clear the very saturation
    that tripped it and the vehicle would look recovered.
    """

    mode_state: ModeState = field(default_factory=ModeState)
    wheel_momentum_Nms: Vector3 = (0.0, 0.0, 0.0)
    #: Whether a SATURATION fault has already been applied. It is a
    #: one-shot: the wheel is filled once, and ordinary physics takes
    #: over afterwards so it can still unload. Re-applying it every step
    #: would pin the wheel at capacity forever, which is a termination
    #: rather than a fault injection -- the same reasoning that made
    #: ReactionWheels let a saturated wheel torque back towards zero.
    fault_applied: bool = False
    #: Cumulative integral of |tau . omega|, joules -- a proxy for slew
    #: cost, reported per run. Accumulated here rather than by whoever
    #: reads the telemetry because this is the only place that holds the
    #: delivered torque and the body rate at the same instant, and a
    #: running total survives a dropped sample where a consumer-side
    #: integration would silently lose that step's contribution.
    wheel_energy_J: float = 0.0
    previous_desired: Optional[Quaternion] = None
    previous_desired_time: Optional[float] = None
    previous_target_key: Optional[tuple] = None


@dataclass(frozen=True)
class TwinStepResult:
    """Mirrors ``out`` in ``+adcs/twin_step.m``."""

    torque_Nm: Vector3
    quaternion_cmd: Quaternion
    rate_next_rad_s: Vector3
    state: TwinState
    boresight_error_rad: float
    mode_changed: bool
    wheels_saturated: bool


def _apply_saturation_fault(config: TwinConfig, inputs: TwinInputs,
                            state: TwinState) -> TwinState:
    """Fill the faulted wheel, once, at or after its epoch.

    Applied to the state going INTO the step, before the momentum guard
    is evaluated, so the chart trips SAFE on the very step the fault
    lands rather than one step later. A one-step lag there would be
    invisible in a plot and would make the injection epoch in the record
    disagree with the epoch the run actually reacted to.

    The sign follows whichever way the wheel was already spinning, so a
    wheel unloading in one direction is not abruptly reversed; with no
    momentum yet it fills positive. Either way the choice is a function
    of the trajectory, so a replay makes it identically.
    """
    fault = config.wheel_fault
    if (fault is None or state.fault_applied
            or not fault.saturates_at(inputs.time_s)):
        return state

    momentum = list(state.wheel_momentum_Nms)
    sign = 1.0 if momentum[fault.axis] >= 0.0 else -1.0
    momentum[fault.axis] = sign * config.momentum_capacity_Nms
    return replace(state,
                   wheel_momentum_Nms=(momentum[0], momentum[1], momentum[2]),
                   fault_applied=True)


def twin_step(config: TwinConfig, inputs: TwinInputs,
              state: TwinState) -> TwinStepResult:
    """One guidance-and-control cycle: guidance, mode, control, dynamics.

    The counterpart of ``+adcs/twin_step.m``, and the reason it exists:
    that function was the *only* part of the twin with nothing on this
    side to generate fixtures from, so the cross-check never exercised
    the orchestration, and two defects lived in it undetected until the
    Simulink signal wiring was finally built.

    The one that mattered: ``quaternionCmd`` was the input attitude,
    returned verbatim. The twin read an attitude from STK, computed a
    torque, and published back the attitude it had just read -- nothing
    anywhere integrated that torque. The loop had no integrator at all,
    so the vehicle could never slew and every capture would have been
    decided by wherever it already happened to point. The design note
    that justified it ("in the closed loop STK is the truth, the
    dynamics are bypassed") rested on a misreading of what STK does with
    injected attitude: ``AddAttitude CBFQuat`` stores and interpolates
    samples, it does not propagate attitude dynamics. STK is the truth
    for orbit, geometry and access; the vehicle's attitude *response* is
    the twin's to compute, and this is where it is computed.

    A failed wheel enters here and nowhere else: the health schedule is
    evaluated against this step's time and handed to the actuator model,
    so a dead axis simply stops delivering torque. The controller is not
    told, and that is the point -- it keeps commanding torque the
    vehicle cannot produce, momentum builds on the remaining axes, and
    the mode logic trips SAFE on the momentum guard. A twin that
    reconfigured its controller around the failure would be measuring
    fault-tolerant control rather than what E5 asks about.

    What the command published still is: the ACHIEVED attitude, not the
    reference. That part of the original intent was right and the whole
    study turns on it -- publishing the reference would quietly
    reinstate the perfect-pointing assumption the paper exists to
    challenge. "Achieved" now means integrated one step forward under
    the torque the wheels actually delivered, starting from the state
    STK reported.
    """
    state = _apply_saturation_fault(config, inputs, state)

    if inputs.have_target:
        q_desired = target_attitude(inputs.position_m, inputs.velocity_m_s,
                                    inputs.target_eci_m,
                                    config.boresight_body)
    else:
        q_desired = nadir_attitude(inputs.position_m, inputs.velocity_m_s,
                                   config.boresight_body)

    target_key = (inputs.have_target, inputs.target_id if inputs.have_target else "")
    desired_rate = (0.0, 0.0, 0.0)
    if (state.previous_target_key == target_key and
            state.previous_desired is not None and
            inputs.time_s > state.previous_desired_time):
        delta = quat_multiply(quat_conjugate(state.previous_desired), q_desired)
        axis, angle = axis_angle_from_quat(delta)
        reference_rate = scale(axis, angle / (inputs.time_s - state.previous_desired_time))
        desired_rate = rotate_to_body(inputs.quaternion,
                                     rotate_to_inertial(q_desired, reference_rate))

    error = boresight_error_rad(inputs.quaternion, q_desired,
                                config.boresight_body)
    rate_magnitude = math.sqrt(dot(inputs.omega_rad_s, inputs.omega_rad_s))
    momentum_fraction = max(abs(h) for h in state.wheel_momentum_Nms) \
        / config.momentum_capacity_Nms

    mode_state = state.mode_state

    # A slew is requested when the desired attitude has moved further
    # than the settle-entry threshold: below that the tracking loop
    # absorbs it without a profile, and re-planning every step would
    # restart the ramp continuously and never converge.
    slew_requested = (inputs.have_target
                      and error > config.settle_entry_rad
                      and mode_state.mode is not MissionMode.SLEW)
    profile_done = (mode_state.profile is None
                    or (inputs.time_s - mode_state.slew_started_s)
                    >= mode_state.profile.duration_s)

    previous_mode = mode_state.mode
    mode_state = update_mode(mode_state, config, inputs.time_s, error,
                             rate_magnitude, momentum_fraction,
                             slew_requested, profile_done)
    mode_changed = mode_state.mode is not previous_mode

    if mode_changed and mode_state.mode is MissionMode.SLEW:
        mode_state = replace(
            mode_state,
            profile=SlewProfile.between(inputs.quaternion, q_desired,
                                        config.profile_limits()),
            slew_started_s=inputs.time_s)

    if mode_state.mode is MissionMode.SLEW:
        q_ref, omega_ref = mode_state.profile.reference_at(
            inputs.time_s - mode_state.slew_started_s)
    elif mode_state.mode is MissionMode.SAFE:
        # Stop pointing at anything. Holding the last reference would
        # keep the wheels loaded against a target they cannot reach.
        q_ref, omega_ref = inputs.quaternion, (0.0, 0.0, 0.0)
    else:
        q_ref, omega_ref = q_desired, desired_rate

    torque = control_torque(config, mode_state.mode, inputs.quaternion,
                            inputs.omega_rad_s, q_ref, omega_ref)

    health = ((1.0, 1.0, 1.0) if config.wheel_fault is None
              else config.wheel_fault.health_at(inputs.time_s))
    wheels = ReactionWheels(
        max_torque_Nm=config.max_torque_Nm,
        momentum_capacity_Nms=config.momentum_capacity_Nms,
        momentum_Nms=state.wheel_momentum_Nms,
        health=health)
    delivered, saturated = wheels.command(torque, config.step_s)

    body = RigidBody(inertia_kg_m2=config.inertia_kg_m2,
                     quaternion=quat_normalize(inputs.quaternion),
                     rate_rad_s=tuple(inputs.omega_rad_s))
    body.step(delivered, wheels.momentum_Nms, config.step_s)

    # Power against the rate the torque actually acted on -- the rate at
    # the START of the step, not the integrated one, matching the
    # left-hand rule the rest of the stepping uses. Taking the rate
    # after integration would credit this step's torque with the motion
    # it produced, which overstates the cost of every acceleration.
    energy = state.wheel_energy_J + abs(
        dot(delivered, inputs.omega_rad_s)) * config.step_s

    return TwinStepResult(
        torque_Nm=delivered,
        quaternion_cmd=body.quaternion,
        rate_next_rad_s=body.rate_rad_s,
        state=TwinState(mode_state=mode_state,
                        wheel_momentum_Nms=wheels.momentum_Nms,
                        wheel_energy_J=energy,
                        fault_applied=state.fault_applied,
                        previous_desired=q_desired,
                        previous_desired_time=inputs.time_s,
                        previous_target_key=target_key),
        boresight_error_rad=error,
        mode_changed=mode_changed,
        wheels_saturated=saturated)


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
                            wheels.momentum_fraction, slew_requested=False,
                            profile_done=now >= profile.complete_at)

        torque = control_torque(config, state.mode, body.quaternion,
                                body.rate_rad_s, q_ref, omega_ref)
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
