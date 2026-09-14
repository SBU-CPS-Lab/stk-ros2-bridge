"""Cross-language fixtures: the contract between the reference and MATLAB.

The Simulink twin is a transcription of ``acav/twin_reference.py`` into
MATLAB, and a transcription is exactly the kind of work that produces a
sign error somewhere in three hundred lines of quaternion algebra. Such
an error does not crash: the twin flies smoothly to a slightly, or
completely, wrong attitude, and the result reads as a control-design
finding.

So the two implementations are held together by data rather than by
review. This module generates input/expected-output pairs from the
Python reference; ``stk_matlab/test_acav_twin.m`` replays them through
the MATLAB functions and asserts agreement to a tolerance far tighter
than anything the study measures. A transcription error becomes a
failing MATLAB test on the deploy machine rather than a number in a
paper.

The fixtures are checked in rather than generated on the fly, so the
MATLAB test needs no Python, and so a change to the reference shows up
as a reviewable diff in the expected values.
"""

from __future__ import annotations

import json
import math
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
#: 3: added the ``twin_step`` section -- the per-step orchestration had
#:    no counterpart in the reference, so nothing generated fixtures for
#:    it and two defects survived C3 in the one function no cross-check
#:    could see. SAFE-mode torque also changed here (zero -> rate
#:    damping) to match the MATLAB, so closed-loop runs that enter SAFE
#:    have new expected values.
#: 4: twin_step now accumulates |tau . omega| into the state, so the
#:    twin can publish wheel energy on TwinStatus and the run record can
#:    report it. The twin_step cases carry the running total.
FIXTURE_VERSION = 4


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
        "twin_step": _twin_step_cases(config),
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


def _twin_step_cases(config: TwinConfig) -> List[Dict]:
    """Per-step orchestration, run closed-loop through the command.

    The section that exercises ``twin_step``: guidance, the mode
    transitions, profile creation, the control law, the actuators and
    the dynamics, in the order and carrying the state that the deployed
    loop does. Each step feeds the previous step's *commanded* attitude
    back in as the next step's measurement -- which is exactly what the
    real loop does via STK -- so a twin that computes a torque but never
    integrates it shows up here as a boresight error that refuses to
    decrease, rather than as nothing at all.

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
    for (name, have_target, tilt_deg, rate, duration_s, times) in (
        # Acquire an off-nadir target: NADIR -> SLEW -> SETTLE -> IMAGE,
        # and the case that proves the vehicle moves at all.
        ("acquire_target", True, 0.0, (0.0, 0.0, 0.0), 20.0,
         (0.0, 4.0, 8.0, 12.0, 16.0, 20.0)),
        # Held at nadir, started 1.5 deg off: inside settle_entry, so no
        # profile is planned and the tracking loop absorbs it directly.
        ("nadir_hold", False, 1.5, (0.0, 0.0, 0.0), 5.0,
         (0.0, 1.0, 2.0, 3.0, 4.0, 5.0)),
        # Tumbling above the rate guard: SAFE trips on the first step and
        # latches, and rate damping runs the wheels into their momentum
        # limit. Both phases are pinned deliberately -- a wheels-only
        # ADCS cannot dump momentum, so it can only remove about
        # h_max/J = 9.3 deg/s of tumble and the rest persists. That is
        # the vehicle E5 injects faults into, and a fixture that stopped
        # before saturation would hide it.
        ("safe_rate_damping", False, 0.0, (0.279, 0.0, 0.0), 15.0,
         (0.0, 3.0, 6.0, 9.0, 12.0, 15.0)),
    ):
        q = quat_multiply(nadir, quat_from_axis_angle(
            (1.0, 0.0, 0.0), math.radians(tilt_deg)))
        omega = rate
        state = TwinState()

        wanted = list(times)
        checkpoints = []
        steps = int(round(duration_s / config.step_s))
        for index in range(steps + 1):
            now = index * config.step_s
            result = twin_step(config, TwinInputs(
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
            "checkpoints": checkpoints,
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
    exact check reports "out of date" for a file nobody touched -- which
    trains the reader to ignore it. A tolerance of 1e-9 is far below
    anything the study measures and far above last-bit noise, so a
    reported difference is a real change in what the twin must do.
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
        # number would sail through a tolerant numeric comparison -- and
        # `settling_time_s` returning None is meaningfully different
        # from returning 0.0, which is the whole reason it can.
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
