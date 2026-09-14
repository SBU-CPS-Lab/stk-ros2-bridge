# C3 — The ADCS twin

The Simulink/Stateflow digital twin the bridge drives, and the Python
reference that pins its semantics.

## 1. Why there are two implementations

The twin runs in MATLAB on the Windows machine. Its correctness rests on
about three hundred lines of quaternion algebra, rigid-body dynamics and
mode logic, none of which can be tested on the development machine.

A sign error in that code does not crash. The twin flies smoothly to a
slightly — or completely — wrong attitude, STK dutifully reports no
access, the capture rate drops, and the result reads as a control-design
finding. That is the same failure mode the bridge's own bring-up hit
when an ECF-commanded attitude was read back against an inertial
provider: wrong by the Earth-rotation angle, and entirely plausible.

So the semantics are fixed once, in Python, where they are tested:

| Python | MATLAB | What it is |
|---|---|---|
| `acav/attitude.py` | `+adcs/quat.m` | quaternion algebra, conventions |
| `acav/twin_reference.py` | `+adcs/wheels.m`, `rigid_body_step.m`, `slew_profile.m`, `tracking_torque.m`, `mission_mode.m` | dynamics, actuators, guidance, control, modes |
| `acav/twin_fixtures.py` | `test_acav_twin.m` | the contract between them |

`export_twin_fixtures` writes input/expected-output pairs from the
reference; `test_acav_twin.m` replays them through the MATLAB and
asserts agreement to 1e-9. A transcription error becomes a failing test
on the deploy machine rather than a number in the paper.

This has already earned itself: `adcs.quat.align` shipped with a
transposed rotation matrix, which produces a perfectly valid attitude
pointing roughly 180° from the target. It was caught by re-implementing
the MATLAB algorithm in Python and comparing over random geometries,
before the code was ever run.

**No number from the Python reference may appear in the paper.** It has
no flexible modes, no wheel friction, no sensor noise and no Stateflow.
It fixes semantics; the twin produces results.

## 2. Conventions

Every one of these has a plausible-looking opposite, and each is pinned
by a test asserting a *physical* outcome rather than by a comment.

| | |
|---|---|
| Component order | `(x, y, z, w)` — vector part first. Matches the bridge, ROS, and STK's Q1..Q3 / Q4. |
| Direction | `q` rotates a vector **from inertial into body** components. |
| Product | Hamilton, with `R(a ⊗ b) = R(b) R(a)`. |
| Rates | `ω` is body-referenced, rad/s; `q̇ = ½ q ⊗ (ω, 0)`. |
| Boresight | `sensor.boresight_body_axis` in the YAML — the single place `+Z` is assumed. |

The direction convention is fixed by
`test_attitude.py::test_body_yawed_ninety_degrees_about_z`: rotate the
body +90° about z, and the inertial x axis must appear along the body's
**−y**. The opposite convention puts it on +y, and every sign in the
controller follows from which is true.

`boresight_error_rad` is deliberately distinct from total attitude
error. Rotation *about* the boresight does not move the footprint, so a
twin settled in boresight but not in roll still images the target. STK's
access computation cares only about the boresight, so the capture
criterion must too — scoring total attitude error would fail targets the
sensor actually sees.

## 3. Control design

A saturated PD controller on a double integrator overshoots badly, and
that overshoot would appear as settling time the scheduler never
budgeted — E2 would then be measuring the controller's overshoot rather
than the effect under study. So the twin does what an agile EO
spacecraft actually does:

1. an **eigenaxis profile generator** produces a feasible reference
   trajectory using the same triangular/trapezoidal model
   `acav/slew_model.py` gives the scheduler;
2. a **quaternion-feedback controller with rate feedforward** tracks it.

Achieved slew time is then close to the analytic bound by construction,
and settling is what remains to be measured.

### Numbers, and how they were chosen

Measured by sweeping bandwidth against profile authority and recording
the excess of achieved settling over the analytic slew bound, for every
slew in the scenario:

| authority | ω_n = 0.5 | ω_n = 0.7 | ω_n = 1.0 |
|---|---|---|---|
| 1.0 | +9.3 … +13.1 s | +6.3 … +9.8 s | +4.5 … +7.6 s |
| 0.8 | +9.9 … +12.4 s | +6.3 … +7.9 s | **+3.7 … +4.8 s** |
| 0.6 | +10.6 … +13.9 s | +7.1 … +9.6 s | +4.5 … +6.5 s |

The scheduler budgets `settle_estimate_s = 8.0 s`. At ω_n = 0.7 the
largest slew needed +9.8 s and would have missed every tight target for
a reason having nothing to do with scheduling. Chosen: **ω_n = 1.0
rad/s, ζ = 0.707, profile authority 0.8**.

`profile_authority` is why the profile uses derated limits. At full
authority the feedforward term alone demands every newton-metre the
wheels have, leaving the feedback loop nothing to correct tracking error
with; the residual only decays once the profile stops asking. Holding
back 20% costs ~12% on the slew and buys back several seconds of
settling.

`test_twin_reference.py::test_every_acav_slew_fits_the_scheduler_s_budget`
guards this. If a gain change breaks it, E2's headline becomes an
artifact, so the test fails rather than the study quietly degrading.

**Honest caveat for the paper.** A 1.0 rad/s bandwidth is optimistic for
a 6U CubeSat, because the twin models no sensor noise, no attitude
estimation lag and no wheel dynamics. It is a fair basis for comparing
*schedulers* — both face the same vehicle — but it is not a claim about
achievable CubeSat pointing performance, and must not be presented as
one.

## 4. Mission modes

```
NADIR ──slew requested──▶ SLEW ──profile done──▶ SETTLE
  ▲                                                 │
  │                                    error ≤ 2°   │
  │                                                 ▼
  └────────────── slew requested ◀───────────── IMAGE

  any state ──momentum ≥ capacity or rate > 15°/s──▶ SAFE  (latched)
```

Two properties are load-bearing and are asserted by tests:

- **The SAFE guards are evaluated first and unconditionally.** A wheel
  that saturates during IMAGE must trip SAFE rather than be masked by an
  already-satisfied hold. Written the other way round, the chart behaves
  correctly in every nominal run and silently fails to protect the
  vehicle in exactly the campaign (E5) that exists to test it.
- **SAFE latches.** An ADCS that re-armed the instant the trip cleared
  would oscillate in and out and make E5's deterministic replay
  meaningless. Recovery is an operator action.

The Stateflow chart is what the paper shows; `+adcs/mission_mode.m` is
the same logic as a function, which is what the fixtures can exercise.
They must agree, and a change belongs in both.

## 5. What has actually been executed

Run against MATLAB R2023a on the development machine:

| | |
|---|---|
| `test_acav_twin` | **35/35 pass** — the MATLAB reproduces every fixture, closed-loop trajectories included |
| `adcs.build_mission_chart` | builds; SAFE latches (0 outgoing), reachable from all four nominal states, all guards at execution order 1 |
| `build_acav_twin` | writes a real, fully wired `.slx` (R2024a, with `ros2genmsg` run first) |
| `ros2genmsg` | **succeeds**, R2024a, given a Python 3.8-3.11 interpreter and a `-include cstdint` compile flag the dev box needs but the deploy machine should not (see `docs/C4_scheduler.md` section 2) |

So the algorithms, the chart, message generation, and now the signal
wiring itself are all built and code-reviewable. What is NOT verified
on this dev box is whether the wiring actually *type-checks*: a
separate dev-box-only tooling limitation (see `docs/C4_scheduler.md`
section 2) means `build_acav_twin`'s own diagram-update check cannot
currently pass here for ANY custom `stk_msgs` type, wiring mistake or
not — it now warns instead of failing the build, and says so. The
narrower, honest boundary this leaves: the wiring is written and
structurally complete, but its correctness has only been checked by
inspection, not by Simulink's own compiler, until it runs somewhere
without this limitation (the deploy machine, most likely).

Three things this turned up:

- **`jsondecode` returns a struct array only when every object in a JSON
  array has identical field names, and a cell array otherwise.** The
  fixture sections legitimately mix shapes, so the harness normalises
  both through one accessor. The symptom was `Dot indexing is not
  supported for variables of this type`, which says nothing about the
  cause.
- **There is no ROS time-source model parameter.** R2023a's model has
  none, and the ROS 2 Current Time block exposes only `OutputFormat` and
  `SampleTime`. An earlier version set an invented `ROSTimeSource` and
  warned when it failed, which read as a release incompatibility rather
  than as a parameter that never existed. Time follows `/clock` because
  the *node* runs with `use_sim_time` — a launch-time setting.
- **`profile_done` was derived in Python and passed as an argument in
  MATLAB**, so the two disagreed about a SLEW state with no profile. The
  cross-check caught it. It is now an explicit input on both sides,
  which also stops a bug state (SLEW without a profile) from being
  silently treated as "finished".

## 6. Running it

On the Windows machine, with the workspace built and sourced:

```matlab
cd stk_ros2_ws\src\stk_matlab
setup_stk_ros                 % paths + ros2genmsg
test_acav_twin                % cross-check against the reference
build_acav_twin               % generate acav_twin.slx
```

`test_acav_twin` is the C3 equivalent of `verify_acav_scenario.py`:
every check runs, nothing stops at the first failure, and the summary
says what is wrong rather than that something is. Paste its output the
way you paste the verifier's.

If the fixtures are stale it refuses to run rather than checking against
the wrong expectations. Regenerate with:

```
python src\acav\scripts\export_twin_fixtures.py
```

and review the diff — a change in an expected value is a change in what
the twin is required to do.

`build_acav_twin` writes a `.slx`. That file is a **build artifact** and
is not checked in: Simulink's binary format does not diff or merge, so a
reviewer cannot see what changed in a control law and a git conflict in
one is unresolvable. Edit the builder, not the model.

## 7. Open against the live install

- **The boresight is confirmed.** C2 run #4 saw a nadir target inside a
  measured (1.80, 2.20]° cone, so `+Z` is the boresight under the
  scenario's default attitude and `sensor.boresight_body_axis =
  [0, 0, 1]` stands.
- **Block wiring is built.** `build_acav_twin` needs `setup_stk_ros` to
  have run first; it checks for the message types up front and says so,
  rather than dying inside a mask callback with a half-built model open.
  The five Subscribe blocks feed a consolidating `UnpackTelemetry`
  MATLAB Function, which feeds `GuidanceControl`; its outputs go out
  through a Blank-Message/Bus-Assignment/Header-Assignment chain to each
  Publish block. See `docs/C4_scheduler.md` section 2 for a dev-box
  tooling limitation that keeps this from being diagram-update-verified
  here specifically (not a defect in the wiring itself).
- **A design gap the wiring pass exposed, not a wiring gap.**
  `adcs.twin_step`'s `quaternionCmd` is currently `in.quaternion`
  verbatim — the same attitude value just received from STK. Wired as
  designed, the published command equals the last reported state every
  tick, so the vehicle would never actually move in the closed loop.
  `torque` and the mode/settling logic ARE computed correctly (the same
  tested `tracking_torque`/`mission_mode`/`slew_profile` the Python
  reference uses), but nothing yet integrates that torque into a new
  commanded attitude — `adcs.wheels`/`adcs.rigid_body_step` exist and
  are unit-tested but are never called from `twin_step`. `torque` is
  wired to a Terminator for now so the model has no dangling output.
  Fixing this changes validated, fixture-covered control semantics and
  deserves the same rigor C3 gave the rest of this design (a Python
  reference update, new fixtures, re-verification) — deliberately not
  done as a side effect of a wiring pass.
- **The `MissionMode` chart is built but deliberately not wired into the
  signal path.** `adcs.twin_step` already calls `adcs.mission_mode`
  directly as a function to decide `state.mode`, entangled with
  slew-profile initialization the chart alone does not capture. Wiring
  the chart in parallel would either duplicate the mode computation with
  no consumer, or need restructuring `GuidanceControl` to take the
  chart's mode as an input — a control-flow change, not a wiring one. It
  stays in the model as the paper's hybrid-model figure.
