"""Tests for the ADCS twin's reference implementation.

These are the semantics the MATLAB twin is a transcription of, so a test
that only checks the code against itself is worth nothing here. Each one
below asserts either a physical invariant (momentum conservation, energy
behaviour), agreement with the *independently derived* slew model that
the scheduler uses, or a control-design property the study depends on
(no overshoot past the capture tolerance, SAFE latching).
"""

import math

import pytest

from acav.attitude import (
    boresight_error_rad,
    dot,
    pointing_error_rad,
    quat_from_axis_angle,
    quat_multiply,
    rotate_to_inertial,
)
from acav.fault_injection import FAILURE, WheelFault
from acav.slew_model import slew_time_s
from acav.spec import AcavSpec
from acav.twin_reference import (
    MissionMode,
    ModeState,
    ReactionWheels,
    RigidBody,
    SlewProfile,
    TwinConfig,
    TwinInputs,
    TwinState,
    nadir_attitude,
    settling_time_s,
    simulate_slew,
    target_attitude,
    tracking_torque,
    twin_step,
    update_mode,
)

IDENTITY = (0.0, 0.0, 0.0, 1.0)
X, Y, Z = (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)


@pytest.fixture(scope="module")
def spec():
    return AcavSpec.load()


@pytest.fixture(scope="module")
def config(spec):
    return TwinConfig.from_spec(spec)


class TestConfigComesFromTheScenario:
    def test_nothing_is_hardcoded_away_from_the_yaml(self, spec, config):
        assert config.inertia_kg_m2 == (0.123, 0.096, 0.041)
        assert config.max_torque_Nm == spec.actuators.max_torque_Nm
        assert config.boresight_body == tuple(
            spec.sensor.boresight_body_axis)
        assert config.capture_error_rad == pytest.approx(
            math.radians(spec.pointing.capture_error_deg))

    def test_the_twin_and_the_scheduler_share_one_slew_model(self, spec,
                                                             config):
        """C1's central contract. Two independently-written slew models
        would make E2 measure their disagreement rather than the effect
        under study."""
        assert config.slew_limits().alpha_rad_s2 == pytest.approx(
            spec.slew_limits().alpha_rad_s2)
        assert config.slew_limits().omega_max_rad_s == pytest.approx(
            spec.slew_limits().omega_max_rad_s)

    def test_gains_equalise_bandwidth_across_a_very_anisotropic_body(
            self, config):
        """Inertia spans 0.041 to 0.123 kg m^2. One scalar gain would
        make the z axis three times faster than x and the response
        direction-dependent for no physical reason."""
        kp, kd = config.gains()
        for axis in range(3):
            omega_n = math.sqrt(kp[axis] / config.inertia_kg_m2[axis])
            zeta = kd[axis] / (2.0 * math.sqrt(kp[axis]
                                               * config.inertia_kg_m2[axis]))
            assert omega_n == pytest.approx(config.natural_frequency_rad_s)
            assert zeta == pytest.approx(config.damping_ratio)


class TestReactionWheels:
    def _wheels(self):
        return ReactionWheels(max_torque_Nm=0.002,
                              momentum_capacity_Nms=0.020)

    def test_torque_is_clamped_to_the_wheel_limit(self):
        wheels = self._wheels()
        delivered, saturated = wheels.command((0.010, 0.0, 0.0), 0.1)
        assert delivered[0] == pytest.approx(0.002)
        assert saturated

    def test_momentum_accumulates_opposite_the_body_torque(self):
        """The wheel absorbs the reaction. A sign error here makes
        momentum unload when it should build, and the SAFE trip that E5
        depends on never fires."""
        wheels = self._wheels()
        wheels.command((0.002, 0.0, 0.0), 1.0)
        assert wheels.momentum_Nms[0] == pytest.approx(-0.002)

    def test_momentum_saturation_clamps_and_reports(self):
        wheels = self._wheels()
        for _ in range(20):
            wheels.command((0.002, 0.0, 0.0), 1.0)
        assert abs(wheels.momentum_Nms[0]) == pytest.approx(0.020)
        _, saturated = wheels.command((0.002, 0.0, 0.0), 1.0)
        assert saturated
        assert wheels.momentum_fraction == pytest.approx(1.0)

    def test_a_saturated_wheel_can_still_torque_back(self):
        """Clamping both directions would make saturation permanent, so
        every E5 run would end in an unrecoverable SAFE -- a termination
        rather than a fault injection."""
        wheels = self._wheels()
        for _ in range(20):
            wheels.command((0.002, 0.0, 0.0), 1.0)
        delivered, _ = wheels.command((-0.002, 0.0, 0.0), 1.0)
        assert delivered[0] == pytest.approx(-0.002)
        assert abs(wheels.momentum_Nms[0]) < 0.020

    def test_a_failed_wheel_delivers_nothing(self):
        """E5's injected fault."""
        wheels = ReactionWheels(max_torque_Nm=0.002,
                                momentum_capacity_Nms=0.020,
                                health=(0.0, 1.0, 1.0))
        delivered, saturated = wheels.command((0.002, 0.002, 0.0), 0.1)
        assert delivered[0] == pytest.approx(0.0)
        assert delivered[1] == pytest.approx(0.002)
        assert saturated


class TestRigidBody:
    def test_torque_free_motion_conserves_angular_momentum(self):
        """The strongest available check on the dynamics: with no
        external torque, |J w| in inertial axes must not drift. An error
        in the gyroscopic term shows up here and essentially nowhere
        else in a nominal run."""
        body = RigidBody(inertia_kg_m2=(0.123, 0.096, 0.041),
                         rate_rad_s=(0.05, -0.03, 0.08))

        def inertial_momentum():
            j = body.inertia_kg_m2
            body_momentum = tuple(j[k] * body.rate_rad_s[k]
                                  for k in range(3))
            return rotate_to_inertial(body.quaternion, body_momentum)

        initial = inertial_momentum()
        for _ in range(20000):
            body.step((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.005)
        final = inertial_momentum()

        for k in range(3):
            assert final[k] == pytest.approx(initial[k], abs=1e-9)

    def test_torque_free_motion_conserves_kinetic_energy(self):
        body = RigidBody(inertia_kg_m2=(0.123, 0.096, 0.041),
                         rate_rad_s=(0.05, -0.03, 0.08))

        def energy():
            j = body.inertia_kg_m2
            return 0.5 * sum(j[k] * body.rate_rad_s[k] ** 2
                             for k in range(3))

        initial = energy()
        for _ in range(20000):
            body.step((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.005)
        assert energy() == pytest.approx(initial, rel=1e-9)

    def test_constant_torque_about_a_principal_axis_matches_theory(self):
        body = RigidBody(inertia_kg_m2=(0.123, 0.096, 0.041))
        torque, duration, dt = 0.001, 2.0, 0.001
        for _ in range(int(duration / dt)):
            body.step((torque, 0.0, 0.0), (0.0, 0.0, 0.0), dt)
        expected_rate = torque / 0.123 * duration
        assert body.rate_rad_s[0] == pytest.approx(expected_rate, rel=1e-6)
        assert pointing_error_rad(body.quaternion, IDENTITY) == \
            pytest.approx(0.5 * torque / 0.123 * duration ** 2, rel=1e-5)

    def test_the_quaternion_stays_normalised(self):
        body = RigidBody(inertia_kg_m2=(0.123, 0.096, 0.041),
                         rate_rad_s=(0.3, 0.2, -0.4))
        for _ in range(50000):
            body.step((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.002)
        assert math.sqrt(dot(body.quaternion, body.quaternion)) == \
            pytest.approx(1.0, abs=1e-12)


class TestSlewProfile:
    def test_duration_matches_the_shared_slew_model_exactly(self, config):
        """Not "closely" -- exactly, for whichever limits it is given.
        The profile generator and the scheduler run the same code."""
        for angle_deg in (5.0, 18.1, 42.3, 92.0, 120.0):
            target = quat_from_axis_angle(X, math.radians(angle_deg))
            for limits in (config.slew_limits(), config.profile_limits()):
                profile = SlewProfile.between(IDENTITY, target, limits)
                assert profile.duration_s == pytest.approx(
                    slew_time_s(math.radians(angle_deg), limits))

    def test_the_profile_leaves_the_controller_some_authority(self, config):
        """At full authority the feedforward term alone demands every
        newton-metre the wheels have, so tracking error cannot be
        corrected until the profile stops asking. Measured: the largest
        scenario slew then settled 9.8 s past the bound against an 8 s
        budget."""
        assert 0.5 <= config.profile_authority < 1.0
        assert config.profile_limits().alpha_rad_s2 < \
            config.slew_limits().alpha_rad_s2

    def test_it_starts_and_finishes_at_rest_on_target(self, config):
        target = quat_from_axis_angle((1.0, 0.3, 0.0), math.radians(30.0))
        profile = SlewProfile.between(IDENTITY, target,
                                      config.profile_limits())

        q0, w0 = profile.reference_at(0.0)
        assert pointing_error_rad(q0, IDENTITY) == pytest.approx(0.0,
                                                                 abs=1e-12)
        assert w0 == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)

        q1, w1 = profile.reference_at(profile.duration_s)
        assert pointing_error_rad(q1, target) < 1e-9
        assert w1 == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)

    def test_the_angle_profile_integrates_its_own_rate(self, config):
        """Position and rate references must be consistent, or the
        feedforward term fights the feedback term throughout the slew."""
        target = quat_from_axis_angle(X, math.radians(42.3))
        profile = SlewProfile.between(IDENTITY, target,
                                      config.profile_limits())
        dt = 1e-4
        integrated = 0.0
        t = 0.0
        while t < profile.duration_s:
            integrated += profile.angle_at(t)[1] * dt
            t += dt
        assert integrated == pytest.approx(profile.total_angle_rad, rel=1e-3)

    def test_it_never_exceeds_the_actuator_limits(self, config):
        limits = config.profile_limits()
        target = quat_from_axis_angle(X, math.radians(120.0))
        profile = SlewProfile.between(IDENTITY, target, limits)
        for index in range(2000):
            _, rate = profile.angle_at(profile.duration_s * index / 2000.0)
            assert rate <= limits.omega_max_rad_s + 1e-12

    def test_every_acav_slew_is_triangular(self, spec, config):
        """The scenario was designed so; if it stops holding the wheel
        sizing changed and the feasibility table is stale."""
        limits = config.slew_limits()
        previous = 0.0
        for target in sorted(spec.targets, key=lambda t: t.t_image_s):
            angle = math.radians(spec.slew_between_deg(previous,
                                                       target.cross_track_km))
            assert angle <= limits.crossover_rad
            previous = target.cross_track_km


class TestClosedLoop:
    def test_a_nominal_slew_converges_inside_the_capture_tolerance(self,
                                                                   config):
        target = quat_from_axis_angle(X, math.radians(18.1))
        run = simulate_slew(config, IDENTITY, target, duration_s=40.0)
        assert math.degrees(run.samples[-1].boresight_error_rad) < 0.5

    def test_achieved_slew_time_is_close_to_the_analytic_bound(self, config):
        """The model is a lower bound; tracking a feasible profile should
        land near it. A large excess means the controller is lagging the
        profile, which would show up in E2 as settling time the
        scheduler never budgeted."""
        angle = math.radians(42.3)
        target = quat_from_axis_angle(X, angle)
        run = simulate_slew(config, IDENTITY, target, duration_s=60.0)
        bound = slew_time_s(angle, config.slew_limits())
        settled = settling_time_s(run, config.capture_error_rad,
                                  config.settle_hold_s)
        assert settled is not None, "never settled inside capture tolerance"
        assert settled >= bound * 0.95
        assert settled < bound + 8.0, (
            f"settled at {settled:.1f} s against a {bound:.1f} s bound; the "
            f"scheduler budgets 8.0 s of settling on top, so exceeding it "
            f"would make every tight target fail for a reason unrelated to "
            f"the effect under study")

    def test_it_does_not_overshoot_past_the_capture_tolerance(self, config):
        """A saturated PD controller on a double integrator overshoots
        badly. This is the property the profile generator exists to buy,
        so it is asserted rather than hoped for."""
        target = quat_from_axis_angle(X, math.radians(30.0))
        run = simulate_slew(config, IDENTITY, target, duration_s=60.0)
        settled = settling_time_s(run, config.capture_error_rad,
                                  config.settle_hold_s)
        assert settled is not None
        after = [s for s in run.samples if s.time_s >= settled]
        worst = max(math.degrees(s.boresight_error_rad) for s in after)
        assert worst <= 0.5

    def test_wheels_stay_inside_their_momentum_budget(self, config):
        """If a nominal 42 deg slew saturated the wheels, SAFE would trip
        during normal operation and E5 would have nothing to measure."""
        target = quat_from_axis_angle(X, math.radians(42.3))
        run = simulate_slew(config, IDENTITY, target, duration_s=60.0)
        worst = max(max(abs(h) for h in s.wheel_momentum_Nms)
                    for s in run.samples)
        assert worst < config.momentum_capacity_Nms

    def test_commanded_torque_never_exceeds_the_wheel_limit(self, config):
        target = quat_from_axis_angle((0.3, 1.0, 0.2), math.radians(60.0))
        run = simulate_slew(config, IDENTITY, target, duration_s=60.0)
        for sample in run.samples:
            for axis in range(3):
                assert abs(sample.torque_Nm[axis]) <= \
                    config.max_torque_Nm + 1e-12

    @pytest.mark.parametrize("angle_deg", [13.4, 18.6, 30.4, 42.3])
    def test_every_acav_slew_fits_the_scheduler_s_budget(self, config,
                                                         angle_deg,
                                                         spec):
        """The design property the whole experiment rests on.

        A dynamics-aware scheduler budgets `analytic bound +
        settle_estimate_s` and declines anything that does not fit. If
        the twin cannot actually meet that budget then every target
        fails, E2 measures the controller's inadequacy rather than the
        scheduling effect, and the headline result is an artifact. This
        test is what stops a gain change from doing that quietly.
        """
        target = quat_from_axis_angle(X, math.radians(angle_deg))
        run = simulate_slew(config, IDENTITY, target, duration_s=60.0)
        settled = settling_time_s(run, config.capture_error_rad,
                                  config.settle_hold_s)
        assert settled is not None, f"{angle_deg} deg never settled"

        bound = slew_time_s(math.radians(angle_deg), config.slew_limits())
        budget = bound + spec.scheduling.settle_estimate_s
        assert settled <= budget, (
            f"{angle_deg} deg settled at {settled:.1f} s against a "
            f"{budget:.1f} s budget ({bound:.1f} s bound + "
            f"{spec.scheduling.settle_estimate_s:.1f} s settling)")


class TestCommandedAttitudes:
    def test_nadir_attitude_points_the_boresight_down(self, config):
        position = (6_878_137.0, 0.0, 0.0)
        velocity = (0.0, 1000.0, 7500.0)
        q = nadir_attitude(position, velocity, config.boresight_body)
        boresight = rotate_to_inertial(q, config.boresight_body)
        assert dot(boresight, (-1.0, 0.0, 0.0)) > 0.999

    def test_target_attitude_points_the_boresight_at_the_target(self,
                                                                config):
        position = (6_878_137.0, 0.0, 0.0)
        velocity = (0.0, 1000.0, 7500.0)
        target = (6_371_000.0, 200_000.0, 0.0)
        q = target_attitude(position, velocity, target, config.boresight_body)
        boresight = rotate_to_inertial(q, config.boresight_body)
        expected = [t - p for t, p in zip(target, position)]
        magnitude = math.sqrt(sum(c * c for c in expected))
        assert dot(boresight, [c / magnitude for c in expected]) > 0.999999

    def test_the_boresight_axis_is_the_only_place_plus_z_is_assumed(self):
        """If the live install turns out to mount the boresight the other
        way, one value in the scenario YAML changes and nothing else
        does. This asserts the plumbing actually honours it."""
        position = (6_878_137.0, 0.0, 0.0)
        velocity = (0.0, 1000.0, 7500.0)
        flipped = (0.0, 0.0, -1.0)
        q = nadir_attitude(position, velocity, flipped)
        assert dot(rotate_to_inertial(q, flipped), (-1.0, 0.0, 0.0)) > 0.999
        # ... and +Z then points at zenith, as it should.
        assert dot(rotate_to_inertial(q, Z), (1.0, 0.0, 0.0)) > 0.999


class TestMissionModeChart:
    def _state(self, mode):
        return ModeState(mode=mode, entered_at_s=0.0)

    def _update(self, mode, config, *, error_deg=0.0, rate_deg_s=0.0,
                momentum=0.0, requested=False, profile_done=False,
                now=5.0):
        return update_mode(self._state(mode), config, now,
                           math.radians(error_deg), math.radians(rate_deg_s),
                           momentum, requested, profile_done)

    def test_slew_request_moves_out_of_nadir(self, config):
        state = self._update(MissionMode.NADIR, config, error_deg=30.0,
                             requested=True)
        assert state.mode is MissionMode.SLEW

    def test_slew_holds_until_the_profile_is_done(self, config):
        """`profile_done` is an input, not something the chart infers.

        The reference used to derive it from `state.profile is None`
        while the MATLAB transcription took it as an argument, so the
        two disagreed about a SLEW state with no profile -- caught by
        the cross-check. Deriving it also hid a bug state: a SLEW with
        no profile is a fault, and treating it as finished is the kind
        of plausible recovery that makes a wrong run look normal.
        """
        assert self._update(MissionMode.SLEW, config, error_deg=30.0,
                            profile_done=False).mode is MissionMode.SLEW
        assert self._update(MissionMode.SLEW, config, error_deg=30.0,
                            profile_done=True).mode is MissionMode.SETTLE

    def test_settle_waits_for_the_entry_threshold(self, config):
        assert self._update(MissionMode.SETTLE, config, error_deg=3.0,
                            profile_done=True).mode is MissionMode.SETTLE
        assert self._update(MissionMode.SETTLE, config, error_deg=1.0,
                            profile_done=True).mode is MissionMode.IMAGE

    def test_image_returns_to_slew_for_the_next_target(self, config):
        state = self._update(MissionMode.IMAGE, config, error_deg=30.0,
                             requested=True, profile_done=True)
        assert state.mode is MissionMode.SLEW

    def test_momentum_saturation_trips_safe_from_any_mode(self, config):
        for mode in (MissionMode.NADIR, MissionMode.SLEW,
                     MissionMode.SETTLE, MissionMode.IMAGE):
            assert self._update(mode, config, momentum=1.0).mode is \
                MissionMode.SAFE, mode

    def test_safe_guards_are_evaluated_before_everything_else(self, config):
        """A wheel that saturates during IMAGE must trip SAFE, not be
        masked by an already-satisfied hold. Ordering the transitions the
        other way gives a chart that is correct in every nominal run and
        silently fails to protect the vehicle in exactly the campaign
        that exists to test it."""
        state = self._update(MissionMode.IMAGE, config, momentum=1.0,
                             requested=True, profile_done=True)
        assert state.mode is MissionMode.SAFE

    def test_excess_body_rate_trips_safe(self, config):
        state = self._update(MissionMode.SLEW, config, error_deg=5.0,
                             rate_deg_s=16.0)
        assert state.mode is MissionMode.SAFE

    def test_safe_latches(self, config):
        """An ADCS that re-armed the instant the trip cleared would
        oscillate in and out of SAFE and make E5's replay meaningless."""
        state = self._state(MissionMode.SAFE)
        for _ in range(10):
            state = update_mode(state, config, 9.0, 0.0, 0.0, 0.0,
                                True, True)
        assert state.mode is MissionMode.SAFE


class TestSettlingTime:
    def _run_with(self, errors_deg, dt=0.1):
        from acav.twin_reference import TwinRun, TwinSample
        run = TwinRun()
        for index, error in enumerate(errors_deg):
            run.samples.append(TwinSample(
                time_s=index * dt, quaternion=IDENTITY,
                rate_rad_s=(0.0, 0.0, 0.0),
                boresight_error_rad=math.radians(error),
                torque_Nm=(0.0, 0.0, 0.0),
                wheel_momentum_Nms=(0.0, 0.0, 0.0), mode="IMAGE"))
        return run

    def test_reports_the_start_of_the_sustained_interval(self):
        run = self._run_with([5.0] * 10 + [0.1] * 40)
        assert settling_time_s(run, math.radians(0.5), 2.0) == \
            pytest.approx(1.0)

    def test_a_brief_excursion_restarts_the_clock(self):
        """Otherwise a controller that dips inside tolerance and bounces
        out again scores as settled, and the capture criterion stops
        meaning what section 6 of the spec says it means."""
        run = self._run_with([0.1] * 10 + [5.0] + [0.1] * 40)
        assert settling_time_s(run, math.radians(0.5), 2.0) == \
            pytest.approx(1.1)

    def test_never_settling_is_none_not_zero(self):
        """Conflating the two would quietly bias any campaign average."""
        assert settling_time_s(self._run_with([5.0] * 50),
                               math.radians(0.5), 2.0) is None


class TestTwinStepClosesTheLoop:
    """The per-step orchestration, which had no reference until now.

    ``twin_step`` was the one part of the twin with no counterpart in
    this module, so no fixture could cross-check it and two defects
    survived the whole of C3 inside it. These are the regression
    boundary for both: that the vehicle actually moves, and that SAFE
    damps rather than coasts.
    """

    POSITION = (6_878_137.0, 0.0, 0.0)
    VELOCITY = (0.0, 1_000.0, 7_500.0)

    @staticmethod
    def _target_at(offset_deg):
        """A surface point ``offset_deg`` of geocentric arc downtrack."""
        offset = math.radians(offset_deg)
        return (6_371_000.0 * math.cos(offset),
                6_371_000.0 * math.sin(offset), 0.0)

    def _fly(self, config, duration_s, target=None,
             initial_rate=(0.0, 0.0, 0.0)):
        """Run the loop the way the deployment runs it.

        Each step's *command* becomes the next step's measurement, which
        is what happens in the real loop via STK. That is what makes a
        twin that computes a torque without integrating it observable
        here rather than merely absent.
        """
        quaternion = nadir_attitude(self.POSITION, self.VELOCITY,
                                    config.boresight_body)
        omega = initial_rate
        state = TwinState()
        history = []
        for index in range(int(round(duration_s / config.step_s)) + 1):
            result = twin_step(config, TwinInputs(
                quaternion=quaternion, omega_rad_s=omega,
                position_m=self.POSITION, velocity_m_s=self.VELOCITY,
                time_s=index * config.step_s,
                target_eci_m=target or (0.0, 0.0, 0.0),
                have_target=target is not None), state)
            history.append(result)
            state = result.state
            quaternion = result.quaternion_cmd
            omega = result.rate_next_rad_s
        return history

    @staticmethod
    def _rate(result):
        return math.sqrt(dot(result.rate_next_rad_s, result.rate_next_rad_s))

    def test_the_vehicle_actually_slews_towards_the_target(self, config):
        """The defect this exists to catch.

        ``quaternionCmd`` used to be the input attitude returned
        verbatim: the twin read an attitude from STK, computed a torque,
        and published back what it had just read. Nothing anywhere
        integrated the torque, so the loop had no integrator at all and
        the pointing error never moved -- which looks entirely healthy
        in isolation, and quietly decides every capture by wherever the
        vehicle already happened to point.
        """
        history = self._fly(config, 20.0, target=self._target_at(2.0))
        started = history[0].boresight_error_rad
        finished = history[-1].boresight_error_rad

        assert started > math.radians(20.0)
        assert finished < config.settle_entry_rad
        assert finished < started / 10.0

    def test_it_reaches_image_through_slew_and_settle(self, config):
        history = self._fly(config, 20.0, target=self._target_at(2.0))
        modes = [r.state.mode_state.mode for r in history]

        assert modes[0] is MissionMode.SLEW
        assert MissionMode.SETTLE in modes
        assert modes[-1] is MissionMode.IMAGE
        assert MissionMode.SAFE not in modes

    def test_the_first_second_moves_only_as_far_as_the_wheels_allow(
            self, config):
        """The command is the ACHIEVED attitude, not the reference.

        Publishing the reference would close the loop instantly and
        reinstate exactly the perfect-pointing assumption the study
        exists to challenge, so early steps must make only the progress
        the actuators can actually deliver against a ~23 deg error.

        The profile starts at rest from the current attitude, so the
        very first step commands zero torque and the vehicle genuinely
        does not move -- the motion has to be measured over a window
        rather than a single step.
        """
        start = nadir_attitude(self.POSITION, self.VELOCITY,
                               config.boresight_body)
        history = self._fly(config, 1.0, target=self._target_at(2.0))
        moved = pointing_error_rad(history[-1].quaternion_cmd, start)

        # alpha = max_torque/J * authority puts the ceiling near 0.4 deg
        # after a second from rest; the point is that it is neither zero
        # nor the whole 23 deg error.
        assert 0.0 < moved < math.radians(1.0)
        assert history[-1].boresight_error_rad > math.radians(20.0)

    def test_safe_damps_the_rate_rather_than_coasting(self, config):
        """The second divergence the missing reference hid.

        This module commanded zero torque in SAFE while
        ``+adcs/twin_step.m`` applied rate damping, and nothing could
        see the disagreement. A safe mode that commands no torque leaves
        the tumble that tripped it exactly where it was, so E5 would be
        measuring a recovery that never happens.
        """
        history = self._fly(config, 8.0, initial_rate=(0.279, 0.0, 0.0))

        assert history[0].state.mode_state.mode is MissionMode.SAFE
        # Torque opposes the rate rather than being zero.
        assert history[0].torque_Nm[0] < 0.0
        assert self._rate(history[-1]) < self._rate(history[0])

    def test_safe_latches_once_tripped(self, config):
        """Recovery is an operator action. An ADCS that re-armed itself
        the instant the trip cleared would oscillate in and out of SAFE
        and make E5's deterministic replay meaningless."""
        history = self._fly(config, 12.0, initial_rate=(0.279, 0.0, 0.0))
        assert all(r.state.mode_state.mode is MissionMode.SAFE
                   for r in history)

    def test_safe_cannot_null_a_tumble_the_wheels_cannot_absorb(self, config):
        """An honest limit of this vehicle, not a defect, and E5 rests
        on it: with reaction wheels and no desaturation path the twin
        can only remove ``h_max / J`` of body rate. Start above that and
        a residual tumble persists with the wheels pinned at capacity --
        which is the failure E5 has to be able to observe, so a model
        that quietly recovered would be the worse error.
        """
        authority = config.momentum_capacity_Nms / config.inertia_kg_m2[0]
        history = self._fly(config, 20.0, initial_rate=(0.279, 0.0, 0.0))

        assert self._rate(history[-1]) == pytest.approx(0.279 - authority,
                                                        abs=1e-6)
        assert abs(history[-1].state.wheel_momentum_Nms[0]) == \
            pytest.approx(config.momentum_capacity_Nms)
        assert history[-1].wheels_saturated

    def test_wheel_momentum_survives_a_mode_change(self, config):
        """It is carried on ``TwinState`` rather than ``ModeState``
        because ``update_mode`` builds a fresh ``ModeState`` on every
        transition. A momentum field there would be silently zeroed each
        time the mode changed, and entering SAFE would clear the very
        saturation that tripped it."""
        history = self._fly(config, 20.0, target=self._target_at(2.0))
        changes = [i for i, r in enumerate(history) if r.mode_changed and i]
        assert changes

        for index in changes:
            before = history[index - 1].state.wheel_momentum_Nms
            after = history[index].state.wheel_momentum_Nms
            assert max(abs(h) for h in before) > 0.0
            assert after != (0.0, 0.0, 0.0)

    def test_wheel_energy_accumulates_monotonically(self, config):
        """A running integral of |tau . omega|, carried in the state so
        the twin can publish it -- it is the only party holding both the
        delivered torque and the body rate at the same instant."""
        history = self._fly(config, 12.0, target=self._target_at(2.0))
        energies = [r.state.wheel_energy_J for r in history]

        assert energies[0] == pytest.approx(0.0)   # at rest, no power
        assert all(b >= a for a, b in zip(energies, energies[1:]))
        assert energies[-1] > 0.0

    def test_a_tumble_damped_in_safe_costs_more_than_a_slew(self, config):
        """Sanity on the magnitude rather than the sign: damping a
        17 deg/s tumble runs the wheels at saturation against a large
        rate, which is far more work than acquiring a target from rest."""
        slew = self._fly(config, 15.0, target=self._target_at(2.0))
        safe = self._fly(config, 15.0, initial_rate=(0.279, 0.0, 0.0))
        assert safe[-1].state.wheel_energy_J > slew[-1].state.wheel_energy_J

    def _fly_with_fault(self, config, fault, duration_s=15.0):
        import dataclasses
        return self._fly(dataclasses.replace(config, wheel_fault=fault),
                         duration_s, target=self._target_at(2.0))

    def test_a_saturating_wheel_trips_safe_on_the_injection_step(self, config):
        """E5's treatment, and C1 section 9's stated expectation. The
        fault is applied to the state going INTO the step, before the
        momentum guard runs, so SAFE trips on the very step the fault
        lands -- a one-step lag would make the epoch in the record
        disagree with the epoch the run reacted to."""
        fault = WheelFault(axis=0, at_s=5.0)
        history = self._fly_with_fault(config, fault)
        modes = [r.state.mode_state.mode for r in history]
        step = int(round(fault.at_s / config.step_s))

        assert modes[step - 1] is not MissionMode.SAFE
        assert modes[step] is MissionMode.SAFE
        assert modes[-1] is MissionMode.SAFE            # and latches

    def test_a_dead_wheel_wrecks_pointing_and_trips_nothing(self, config):
        """Measured, and the reason saturation is E5's default. The
        error is about the dead axis, so the controller asks the healthy
        wheels for almost nothing, their momentum stays near zero, and
        no guard ever fires while pointing collapses. A real gap in the
        vehicle's fault detection -- worth reporting, not papering
        over."""
        history = self._fly_with_fault(
            config, WheelFault(axis=0, at_s=5.0, kind=FAILURE), 40.0)

        assert all(r.state.mode_state.mode is not MissionMode.SAFE
                   for r in history)
        assert history[-1].boresight_error_rad > math.radians(45.0)

    def test_the_fault_is_applied_once_not_every_step(self, config):
        """Re-filling the wheel each step would pin it at capacity and
        make E5 a termination rather than an injection -- the same
        reasoning that lets a saturated wheel torque back towards
        zero."""
        history = self._fly_with_fault(config, WheelFault(axis=0, at_s=5.0))
        step = int(round(5.0 / config.step_s))
        filled = abs(history[step].state.wheel_momentum_Nms[0])
        later = abs(history[-1].state.wheel_momentum_Nms[0])

        # The recorded state is AFTER that step's wheel command, which
        # has already unloaded one step's worth of torque -- so the
        # tolerance is exactly that, not an arbitrary epsilon.
        one_step = config.max_torque_Nm * config.step_s
        assert filled >= config.momentum_capacity_Nms - one_step
        assert later < 0.9 * filled          # and it kept unloading

    def test_a_healthy_vehicle_is_unaffected_by_the_fault_machinery(
            self, config):
        assert config.wheel_fault is None
        history = self._fly(config, 20.0, target=self._target_at(2.0))
        assert history[-1].state.mode_state.mode is MissionMode.IMAGE
