"""Cross-language fixtures: the contract between the reference and MATLAB.

The MATLAB twin is a transcription of ``acav/twin_reference.py``. A sign
error in such a transcription does not crash: the twin steers smoothly
to a slightly, or completely, wrong attitude.

The two implementations are therefore held together by data rather than
by review. This module generates input/expected-output pairs from the
Python reference; ``stk_matlab/test_acav_twin.m`` replays them through
the MATLAB functions and asserts agreement to a tolerance far tighter
than anything the study measures.

The fixtures are checked in rather than generated on the fly, so the
MATLAB test needs no Python, and a change to the reference shows up as a
reviewable diff in the expected values.
"""

from __future__ import annotations

import json
import math
from dataclasses import replace
from typing import Dict, List

from acav.attitude import (
    boresight_error_rad,
    quat_align,
    quat_error,
    quat_from_axis_angle,
    quat_multiply,
    quat_normalize,
    rotate_to_body,
)
from acav.fault_injection import FAILURE, SATURATION, WheelFault
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

#: Bumped whenever the meaning of a fixture changes, so a stale JSON is
#: rejected loudly by the MATLAB side instead of silently passing
#: against the wrong expectations.
#:
#: 3: the ``twin_step`` section (per-step orchestration, closed loop);
#:    SAFE-mode torque is rate damping.
#: 4: twin_step accumulates |tau . omega| (wheel energy) in its state.
#: 5: the ``wheel_fault`` section and faulted twin_step cases, pinning
#:    the zero-based axis handling and the one-shot saturation.
#: 6: the ``moving_twin_step`` section (moving geometry, target changes).
FIXTURE_VERSION = 6


def _q(values) -> List[float]:
    return [float(v) for v in values]


def build_fixtures(spec: AcavSpec) -> Dict:
    config = TwinConfig.from_spec(spec)
    return {
        "version": FIXTURE_VERSION,
        "generated_from": "acav/twin_reference.py",
        "note": ("Expected values produced by the Python reference. The "
                 "MATLAB twin must reproduce them; see "
                 "stk_matlab/test_acav_twin.m."),
        "config": {
            "inertia_kg_m2": list(config.inertia_kg_m2),
            "max_torque_Nm": config.max_torque_Nm,
            "momentum_capacity_Nms": config.momentum_capacity_Nms,
            "boresight_body": list(config.boresight_body),
            "natural_frequency_rad_s": config.natural_frequency_rad_s,
            "damping_ratio": config.damping_ratio,
            "profile_authority": config.profile_authority,
            "settle_entry_rad": config.settle_entry_rad,
            "capture_error_rad": config.capture_error_rad,
            "settle_hold_s": config.settle_hold_s,
            "max_body_rate_rad_s": config.max_body_rate_rad_s,
            "step_s": config.step_s,
        },
        "quaternion_algebra": _quaternion_cases(),
        "attitude_construction": _attitude_cases(config),
        "wheels": _wheel_cases(config),
        "dynamics": _dynamics_cases(config),
        "profile": _profile_cases(config),
        "controller": _controller_cases(config),
        "mode_chart": _mode_cases(config),
        "wheel_fault": _wheel_fault_cases(),
        "twin_step": _twin_step_cases(config),
        "moving_geometry": MOVING_GEOMETRY,
        "moving_twin_step": _moving_twin_step_cases(config),
        "closed_loop": _closed_loop_cases(config),
    }


def _quaternion_cases() -> List[Dict]:
    """The algebra, including the sign conventions that have an opposite."""
    a = quat_from_axis_angle((0.0, 0.0, 1.0), math.radians(40.0))
    b = quat_from_axis_angle((1.0, 0.0, 0.0), math.radians(-25.0))
    return [
        {"name": "multiply",
         "a": _q(a), "b": _q(b), "expected": _q(quat_multiply(a, b))},
        {"name": "rotate_inertial_x_into_body_after_90deg_yaw",
         "a": _q(quat_from_axis_angle((0.0, 0.0, 1.0), math.pi / 2.0)),
         "vector": [1.0, 0.0, 0.0],
         "expected": _q(rotate_to_body(
             quat_from_axis_angle((0.0, 0.0, 1.0), math.pi / 2.0),
             (1.0, 0.0, 0.0)))},
        {"name": "error_quaternion_is_body_referenced",
         "a": _q(quat_multiply(a, quat_from_axis_angle((1.0, 0.0, 0.0),
                                                       math.radians(10.0)))),
         "b": _q(a),
         "expected": _q(quat_error(
             quat_multiply(a, quat_from_axis_angle((1.0, 0.0, 0.0),
                                                   math.radians(10.0))), a))},
        {"name": "boresight_error_ignores_roll",
         "a": _q(quat_multiply(a, quat_from_axis_angle((0.0, 0.0, 1.0),
                                                       math.radians(35.0)))),
         "b": _q(a),
         "expected_scalar": boresight_error_rad(
             quat_multiply(a, quat_from_axis_angle((0.0, 0.0, 1.0),
                                                   math.radians(35.0))), a)},
    ]


def _attitude_cases(config: TwinConfig) -> List[Dict]:
    position = (6_878_137.0, 0.0, 0.0)
    velocity = (0.0, 1_000.0, 7_500.0)
    target = (6_371_000.0, 200_000.0, 50_000.0)
    return [
        {"name": "nadir",
         "position": list(position), "velocity": list(velocity),
         "expected": _q(nadir_attitude(position, velocity,
                                       config.boresight_body))},
        {"name": "target",
         "position": list(position), "velocity": list(velocity),
         "target": list(target),
         "expected": _q(target_attitude(position, velocity, target,
                                        config.boresight_body))},
    ]


def _wheel_cases(config: TwinConfig) -> List[Dict]:
    cases = []
    for name, torque, dt, initial, health in (
        ("within_limits", (0.001, -0.0005, 0.0002), 0.1,
         (0.0, 0.0, 0.0), (1.0, 1.0, 1.0)),
        ("torque_saturated", (0.010, 0.0, 0.0), 0.1,
         (0.0, 0.0, 0.0), (1.0, 1.0, 1.0)),
        ("momentum_saturated", (0.002, 0.0, 0.0), 1.0,
         (-0.0195, 0.0, 0.0), (1.0, 1.0, 1.0)),
        ("failed_wheel", (0.002, 0.002, 0.0), 0.1,
         (0.0, 0.0, 0.0), (0.0, 1.0, 1.0)),
    ):
        wheels = ReactionWheels(config.max_torque_Nm,
                                config.momentum_capacity_Nms,
                                momentum_Nms=initial, health=health)
        delivered, saturated = wheels.command(torque, dt)
        cases.append({
            "name": name, "torque_Nm": list(torque), "dt": dt,
            "initial_momentum_Nms": list(initial), "health": list(health),
            "expected_delivered_Nm": list(delivered),
            "expected_momentum_Nms": list(wheels.momentum_Nms),
            "expected_saturated": bool(saturated),
        })
    return cases


def _dynamics_cases(config: TwinConfig) -> List[Dict]:
    cases = []
    for name, rate, torque, wheel_momentum, steps, dt in (
        ("torque_free_tumble", (0.05, -0.03, 0.08), (0.0, 0.0, 0.0),
         (0.0, 0.0, 0.0), 500, 0.01),
        ("constant_torque", (0.0, 0.0, 0.0), (0.001, 0.0, 0.0),
         (0.0, 0.0, 0.0), 200, 0.01),
        ("gyroscopic_with_wheel_momentum", (0.02, 0.01, -0.03),
         (0.0, 0.0, 0.0), (0.005, -0.002, 0.001), 500, 0.01),
    ):
        body = RigidBody(inertia_kg_m2=config.inertia_kg_m2,
                         rate_rad_s=rate)
        for _ in range(steps):
            body.step(torque, wheel_momentum, dt)
        cases.append({
            "name": name, "initial_rate_rad_s": list(rate),
            "torque_Nm": list(torque),
            "wheel_momentum_Nms": list(wheel_momentum),
            "steps": steps, "dt": dt,
            "expected_quaternion": _q(body.quaternion),
            "expected_rate_rad_s": list(body.rate_rad_s),
        })
    return cases


def _profile_cases(config: TwinConfig) -> List[Dict]:
    cases = []
    for angle_deg in (13.4, 42.3, 120.0):
        target = quat_from_axis_angle((1.0, 0.0, 0.0),
                                      math.radians(angle_deg))
        profile = SlewProfile.between((0.0, 0.0, 0.0, 1.0), target,
                                      config.profile_limits())
        samples = []
        for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
            elapsed = profile.duration_s * fraction
            q_ref, omega_ref = profile.reference_at(elapsed)
            angle, rate = profile.angle_at(elapsed)
            samples.append({"elapsed_s": elapsed, "angle_rad": angle,
                            "rate_rad_s": rate, "quaternion": _q(q_ref),
                            "omega_rad_s": list(omega_ref)})
        cases.append({
            "name": f"slew_{angle_deg:g}deg",
            "start": [0.0, 0.0, 0.0, 1.0], "finish": _q(target),
            "expected_duration_s": profile.duration_s,
            "expected_total_angle_rad": profile.total_angle_rad,
            "samples": samples,
        })
    return cases


def _controller_cases(config: TwinConfig) -> List[Dict]:
    reference = quat_from_axis_angle((0.2, -0.9, 0.5), 1.1)
    cases = []
    for name, offset_deg, omega, omega_ref in (
        ("small_error", 1.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        ("large_error", 40.0, (0.01, -0.02, 0.005), (0.0, 0.0, 0.0)),
        ("tracking_with_feedforward", 2.0, (0.05, 0.0, 0.0),
         (0.05, 0.0, 0.0)),
        ("long_way_round", 200.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ):
        q = quat_multiply(reference,
                          quat_from_axis_angle((1.0, 0.0, 0.0),
                                               math.radians(offset_deg)))
        torque = tracking_torque(config, q, omega, reference, omega_ref)
        cases.append({
            "name": name, "quaternion": _q(quat_normalize(q)),
            "omega_rad_s": list(omega),
            "quaternion_reference": _q(reference),
            "omega_reference_rad_s": list(omega_ref),
            "expected_torque_Nm": list(torque),
        })
    return cases


def _mode_cases(config: TwinConfig) -> List[Dict]:
    cases = []
    for name, mode, error_deg, rate_deg_s, momentum, requested, done in (
        ("nadir_to_slew", "NADIR", 30.0, 0.0, 0.0, True, False),
        ("slew_holds_until_profile_done", "SLEW", 30.0, 1.0, 0.0, False,
         False),
        ("slew_to_settle_when_profile_done", "SLEW", 30.0, 1.0, 0.0, False,
         True),
        ("settle_to_image", "SETTLE", 1.0, 0.1, 0.0, False, True),
        ("settle_waits", "SETTLE", 3.0, 0.1, 0.0, False, True),
        ("image_to_slew_on_request", "IMAGE", 30.0, 0.0, 0.0, True, True),
        ("saturation_trips_safe_from_image", "IMAGE", 0.1, 0.0, 1.0, True,
         True),
        ("rate_trips_safe", "SLEW", 5.0, 16.0, 0.0, False, False),
        ("safe_latches", "SAFE", 0.0, 0.0, 0.0, True, True),
    ):
        state = ModeState(mode=MissionMode(mode), entered_at_s=0.0)
        result = update_mode(state, config, 10.0, math.radians(error_deg),
                             math.radians(rate_deg_s), momentum, requested,
                             done)
        cases.append({
            "name": name, "mode_in": mode,
            "boresight_error_rad": math.radians(error_deg),
            "body_rate_rad_s": math.radians(rate_deg_s),
            "momentum_fraction": momentum,
            "slew_requested": requested,
            "profile_done": done,
            "expected_mode": result.mode.value,
        })
    return cases


def _wheel_fault_cases() -> List[Dict]:
    """The wheel-fault schedule, including the axis convention.

    ``axis`` is zero-based in the record, because that is what Python
    writes; MATLAB indexes from one. An off-by-one would fault the wrong
    wheel while every other number looked right, so it is pinned here.
    """
    cases = []
    for name, fault, time_s in (
        ("saturation_before_epoch", WheelFault(axis=0, at_s=10.0), 9.9),
        ("saturation_at_epoch", WheelFault(axis=0, at_s=10.0), 10.0),
        ("saturation_leaves_health_alone", WheelFault(axis=2, at_s=1.0), 5.0),
        ("failure_before_epoch",
         WheelFault(axis=1, at_s=10.0, kind=FAILURE), 9.9),
        ("failure_at_epoch",
         WheelFault(axis=1, at_s=10.0, kind=FAILURE), 10.0),
        ("failure_on_z_axis",
         WheelFault(axis=2, at_s=0.0, kind=FAILURE), 3.0),
        ("partial_derate",
         WheelFault(axis=0, at_s=0.0, health=0.5, kind=FAILURE), 1.0),
    ):
        cases.append({
            "name": name,
            "fault": fault.to_dict(),
            "time_s": time_s,
            "expected_health": list(fault.health_at(time_s)),
            "expected_saturates": fault.saturates_at(time_s),
        })
    return cases


def _twin_step_cases(config: TwinConfig) -> List[Dict]:
    """Per-step orchestration, run closed-loop through the command.

    The section that exercises ``twin_step``: guidance, the mode
    transitions, profile creation, the control law, the actuators and
    the dynamics, in the order and carrying the state that the deployed
    loop does. Each step feeds the previous step's *commanded* attitude
    back in as the next step's measurement, as the real loop does via
    STK, so a twin that computes a torque but never integrates it shows
    up as a boresight error that does not decrease.

    Recording convention, so the MATLAB replay is unambiguous: at each
    checkpoint ``quaternion``/``rate_rad_s``/``wheel_momentum_Nms`` are
    the state going INTO that step, and ``mode``/``torque_Nm``/
    ``boresight_error_rad`` are what the step produced from it.

    Position and velocity are held fixed for the run. This is the
    reference implementation, and orbital motion belongs to the real
    loop -- the same simplification ``simulate_slew`` makes and for the
    same reason.
    """
    position = (6_878_137.0, 0.0, 0.0)
    velocity = (0.0, 1_000.0, 7_500.0)
    # A 2 deg geocentric offset puts the target ~23.5 deg off nadir --
    # inside the scenario's 30 deg limit, and a cross-track slew about
    # the x axis, which is the axis every slew in this study turns
    # about and the one the profile limits are computed for.
    offset = math.radians(2.0)
    target = (6_371_000.0 * math.cos(offset),
              6_371_000.0 * math.sin(offset), 0.0)
    nadir = nadir_attitude(position, velocity, config.boresight_body)

    cases = []
    for (name, have_target, tilt_deg, rate, duration_s, times,
         fault) in (
        # Acquire an off-nadir target: NADIR -> SLEW -> SETTLE -> IMAGE,
        # and the case that proves the vehicle moves at all.
        ("acquire_target", True, 0.0, (0.0, 0.0, 0.0), 20.0,
         (0.0, 4.0, 8.0, 12.0, 16.0, 20.0), None),
        # Held at nadir, started 1.5 deg off: inside settle_entry, so no
        # profile is planned and the tracking loop absorbs it directly.
        ("nadir_hold", False, 1.5, (0.0, 0.0, 0.0), 5.0,
         (0.0, 1.0, 2.0, 3.0, 4.0, 5.0), None),
        # Tumbling above the rate guard: SAFE trips on the first step and
        # latches, and rate damping runs the wheels into their momentum
        # limit. Both phases are pinned: a wheels-only ADCS cannot dump
        # momentum, so it can only remove about h_max/J = 9.3 deg/s of
        # tumble and the rest persists.
        ("safe_rate_damping", False, 0.0, (0.279, 0.0, 0.0), 15.0,
         (0.0, 3.0, 6.0, 9.0, 12.0, 15.0), None),
        # A wheel saturates mid-slew. The momentum guard sees a full
        # wheel on the very step the fault lands and SAFE trips.
        # Checkpoints straddle the epoch, so a one-step lag in applying
        # it shows up as a mode mismatch.
        ("wheel_saturates_mid_slew", True, 0.0, (0.0, 0.0, 0.0), 12.0,
         (0.0, 4.0, 5.0, 6.0, 9.0, 12.0),
         WheelFault(axis=0, at_s=5.0, seed=7)),
        # The same epoch, the other fault kind: the wheel dies instead.
        # Pointing degrades and nothing trips.
        ("wheel_fails_mid_slew", True, 0.0, (0.0, 0.0, 0.0), 12.0,
         (0.0, 4.0, 5.0, 6.0, 9.0, 12.0),
         WheelFault(axis=0, at_s=5.0, seed=7, kind=FAILURE)),
    ):
        q = quat_multiply(nadir, quat_from_axis_angle(
            (1.0, 0.0, 0.0), math.radians(tilt_deg)))
        omega = rate
        state = TwinState()
        run_config = replace(config, wheel_fault=fault)

        wanted = list(times)
        checkpoints = []
        steps = int(round(duration_s / config.step_s))
        for index in range(steps + 1):
            now = index * config.step_s
            result = twin_step(run_config, TwinInputs(
                quaternion=q, omega_rad_s=omega, position_m=position,
                velocity_m_s=velocity, time_s=now, target_eci_m=target,
                have_target=have_target), state)

            if wanted and abs(now - wanted[0]) < config.step_s / 2.0:
                checkpoints.append({
                    "time_s": now,
                    "quaternion": _q(q),
                    "rate_rad_s": list(omega),
                    "wheel_momentum_Nms": list(state.wheel_momentum_Nms),
                    "mode": result.state.mode_state.mode.value,
                    "torque_Nm": list(result.torque_Nm),
                    "boresight_error_rad": result.boresight_error_rad,
                    "wheels_saturated": result.wheels_saturated,
                    "wheel_energy_J": result.state.wheel_energy_J,
                })
                wanted.pop(0)

            state = result.state
            q = result.quaternion_cmd
            omega = result.rate_next_rad_s

        cases.append({
            "name": name,
            "have_target": have_target,
            "position_m": list(position),
            "velocity_m_s": list(velocity),
            "target_eci_m": list(target),
            "initial_quaternion": _q(quat_multiply(
                nadir, quat_from_axis_angle((1.0, 0.0, 0.0),
                                            math.radians(tilt_deg)))),
            "initial_rate_rad_s": list(rate),
            "duration_s": duration_s,
            "fault": None if fault is None else fault.to_dict(),
            "checkpoints": checkpoints,
        })
    return cases


#: Geometry for the moving cases: a circular polar orbit and targets fixed
#: on a rotating Earth. Simple enough that MATLAB evaluates the same
#: formulas (see movingGeometry in test_acav_twin.m) rather than reading
#: thousands of recorded positions.
MOVING_GEOMETRY = {
    "orbit_radius_m": 6_878_137.0,
    "orbit_rate_rad_s": 2.0 * math.pi / 5677.0,
    "earth_radius_m": 6_371_000.0,
    "earth_rate_rad_s": 7.2921159e-5,
    "targets": {
        "TA": {"lat_rad": math.radians(1.0), "lon_rad": math.radians(1.5)},
        "TB": {"lat_rad": math.radians(3.0), "lon_rad": math.radians(-1.5)},
    },
}


def moving_geometry(time_s: float, target_id: str):
    """(position, velocity, target ECI) at ``time_s``; target is zeros if
    ``target_id`` is empty. Mirrored by movingGeometry in test_acav_twin.m."""
    g = MOVING_GEOMETRY
    angle = g["orbit_rate_rad_s"] * time_s
    r, w = g["orbit_radius_m"], g["orbit_rate_rad_s"]
    position = (r * math.cos(angle), 0.0, r * math.sin(angle))
    velocity = (-r * w * math.sin(angle), 0.0, r * w * math.cos(angle))
    if not target_id:
        return position, velocity, (0.0, 0.0, 0.0)
    t = g["targets"][target_id]
    longitude = t["lon_rad"] + g["earth_rate_rad_s"] * time_s
    re = g["earth_radius_m"]
    target = (re * math.cos(t["lat_rad"]) * math.cos(longitude),
              re * math.cos(t["lat_rad"]) * math.sin(longitude),
              re * math.sin(t["lat_rad"]))
    return position, velocity, target


def _target_at(schedule, time_s: float) -> str:
    """The scheduled target id at ``time_s`` ('' for none)."""
    current = ""
    for start_s, target_id in schedule:
        if time_s + 1e-9 >= start_s:
            current = target_id
    return current


def _run_moving(config: TwinConfig, schedule, duration_s: float,
                checkpoint_every_s: float):
    position, velocity, _ = moving_geometry(0.0, "")
    q = nadir_attitude(position, velocity, config.boresight_body)
    initial_q = q
    omega = (0.0, 0.0, 0.0)
    state = TwinState()
    checkpoints, transitions, slew_starts = [], [], []
    previous_mode, previous_slew = None, None
    steps = int(round(duration_s / config.step_s))
    every = int(round(checkpoint_every_s / config.step_s))
    for index in range(steps + 1):
        now = index * config.step_s
        target_id = _target_at(schedule, now)
        position, velocity, target = moving_geometry(now, target_id)
        result = twin_step(config, TwinInputs(
            quaternion=q, omega_rad_s=omega, position_m=position,
            velocity_m_s=velocity, time_s=now, target_eci_m=target,
            have_target=bool(target_id), target_id=target_id), state)
        mode = result.state.mode_state.mode.value
        if mode != previous_mode:
            transitions.append({"time_s": now, "mode": mode})
            previous_mode = mode
        if mode == "SLEW" and (
                result.state.mode_state.slew_started_s != previous_slew):
            slew_starts.append(result.state.mode_state.slew_started_s)
        previous_slew = result.state.mode_state.slew_started_s
        if index % every == 0:
            checkpoints.append({
                "time_s": now,
                "quaternion": _q(q),
                "rate_rad_s": list(omega),
                "wheel_momentum_Nms": list(state.wheel_momentum_Nms),
                "mode": mode,
                "torque_Nm": list(result.torque_Nm),
                "boresight_error_rad": result.boresight_error_rad,
                "wheels_saturated": result.wheels_saturated,
                "wheel_energy_J": result.state.wheel_energy_J,
            })
        state = result.state
        q = result.quaternion_cmd
        omega = result.rate_next_rad_s
    return initial_q, checkpoints, transitions, slew_starts


def _moving_twin_step_cases(config: TwinConfig) -> List[Dict]:
    """twin_step against MOVING geometry, closed-loop, with target changes.

    The ``twin_step`` cases hold position, velocity and target fixed, so
    every path that exists because the geometry moves was invisible to
    the cross-check: the desired-rate feedforward (zero when the desired
    attitude never moves), the slew profile transported with the line of
    sight (the identity), and the target-identity logic -- re-planning a
    slew when the target changes mid-SLEW or in SETTLE, and resetting
    the rate history on a change or loss of target.

    Each case records checkpoints (same convention as ``twin_step``),
    every mode transition, and every slew (re)plan time, so a replan
    that one implementation makes and the other does not is a named
    failure rather than a drift several seconds later.
    """
    # The two SETTLE-related switch times are found by running the TA-only
    # case, so they stay right when the control design changes.
    # ``settle_s`` is the step whose OUTPUT is
    # SETTLE (its input mode is still SLEW, the old profile just ending);
    # one step later the input mode is SETTLE itself.
    _, _, transitions, _ = _run_moving(config, [(0.0, "TA")], 30.0, 30.0)
    settle_s = next(t["time_s"] for t in transitions if t["mode"] == "SETTLE")
    in_settle_s = round(settle_s + config.step_s, 9)

    cases = []
    for name, schedule, duration_s in (
        ("track_moving_target", [(0.0, "TA")], 30.0),
        ("retarget_mid_slew", [(0.0, "TA"), (4.0, "TB")], 35.0),
        ("retarget_as_slew_ends", [(0.0, "TA"), (settle_s, "TB")],
         settle_s + 25.0),
        ("retarget_in_settle", [(0.0, "TA"), (in_settle_s, "TB")],
         in_settle_s + 25.0),
        ("target_lost_to_nadir", [(0.0, "TA"), (20.0, "")], 35.0),
    ):
        initial_q, checkpoints, transitions, slew_starts = _run_moving(
            config, schedule, duration_s, 2.5)
        cases.append({
            "name": name,
            "schedule": [{"start_s": s, "target_id": t} for s, t in schedule],
            "duration_s": duration_s,
            "initial_quaternion": _q(initial_q),
            "checkpoints": checkpoints,
            "mode_transitions": transitions,
            "slew_starts_s": slew_starts,
        })
    return cases


def _closed_loop_cases(config: TwinConfig) -> List[Dict]:
    """Whole-run behaviour: the integration of everything above.

    Coarse on purpose. Step-by-step agreement over thousands of steps
    would fail on floating-point ordering alone; what has to match is
    the trajectory's shape and the numbers the study reports.
    """
    cases = []
    for angle_deg in (18.6, 42.3):
        target = quat_from_axis_angle((1.0, 0.0, 0.0),
                                      math.radians(angle_deg))
        run = simulate_slew(config, (0.0, 0.0, 0.0, 1.0), target,
                            duration_s=40.0, dt=0.01)
        checkpoints = []
        for time_s in (0.0, 5.0, 10.0, 20.0, 30.0, 40.0):
            sample = min(run.samples, key=lambda s: abs(s.time_s - time_s))
            checkpoints.append({
                "time_s": sample.time_s,
                "quaternion": _q(sample.quaternion),
                "rate_rad_s": list(sample.rate_rad_s),
                "boresight_error_rad": sample.boresight_error_rad,
                "wheel_momentum_Nms": list(sample.wheel_momentum_Nms),
            })
        cases.append({
            "name": f"closed_loop_{angle_deg:g}deg",
            "start": [0.0, 0.0, 0.0, 1.0], "target": _q(target),
            "duration_s": 40.0, "dt": 0.01,
            "expected_settling_time_s": settling_time_s(
                run, config.capture_error_rad, config.settle_hold_s),
            "checkpoints": checkpoints,
        })
    return cases


def compare_fixtures(on_disk, expected, tolerance: float = 1e-9,
                     path: str = "") -> List[str]:
    """Differences between two fixture trees, as readable paths.

    Numbers are compared with a tolerance rather than for equality.
    ``sin``, ``cos`` and ``atan2`` come from the platform's libm and can
    differ in the last bit between machines and Python builds, so an
    exact check would report an untouched file as out of date. A
    tolerance of 1e-9 is far below anything the study measures and far
    above last-bit noise, so a reported difference is a real change.
    """
    differences: List[str] = []

    if isinstance(expected, dict):
        if not isinstance(on_disk, dict):
            return [f"{path or '<root>'}: expected an object"]
        for key in sorted(set(expected) | set(on_disk)):
            if key not in on_disk:
                differences.append(f"{path}.{key}: missing on disk")
            elif key not in expected:
                differences.append(f"{path}.{key}: no longer generated")
            else:
                differences += compare_fixtures(on_disk[key], expected[key],
                                                tolerance, f"{path}.{key}")
    elif isinstance(expected, list):
        if not isinstance(on_disk, list) or len(on_disk) != len(expected):
            count = len(on_disk) if isinstance(on_disk, list) else "?"
            return [f"{path}: {count} entries, "
                    f"expected {len(expected)}"]
        for index, (a, b) in enumerate(zip(on_disk, expected)):
            differences += compare_fixtures(a, b, tolerance,
                                            f"{path}[{index}]")
    elif (isinstance(expected, bool) or isinstance(on_disk, bool)
            or expected is None or on_disk is None):
        # Checked before the numeric branch, and on BOTH sides. In
        # Python False == 0.0, so a saturation flag that turned into a
        # number would pass a tolerant numeric comparison, and
        # `settling_time_s` returning None means something different
        # from returning 0.0.
        if type(on_disk) is not type(expected) or on_disk != expected:
            differences.append(f"{path}: {on_disk!r}, expected {expected!r}")
    elif isinstance(expected, (int, float)):
        if not isinstance(on_disk, (int, float)):
            differences.append(f"{path}: {on_disk!r} is not a number")
        elif abs(on_disk - expected) > tolerance * max(
                1.0, abs(expected)):
            differences.append(
                f"{path}: {on_disk!r}, expected {expected!r} "
                f"(differs by {abs(on_disk - expected):.3e})")
    elif on_disk != expected:
        differences.append(f"{path}: {on_disk!r}, expected {expected!r}")

    return differences


def write_fixtures(path: str, spec: AcavSpec) -> Dict:
    payload = build_fixtures(spec)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=False)
        handle.write("\n")
    return payload
