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
| `twin_reference.twin_step` | `+adcs/twin_step.m` | the per-step orchestration that ties those together |
| `acav/twin_fixtures.py` | `test_acav_twin.m` | the contract between them |

That third row was missing until the wiring pass, and its absence is
exactly where two defects lived undetected — see section 8. Every row of
this table has to exist, or the part with no counterpart is the part
nothing checks.

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

Run against MATLAB **R2023a and R2024a** on the development
machine -- both, deliberately: the cross-check used to run only on
R2023a while message generation and model builds happened on R2024a, so
no single version had seen both halves.

| | |
|---|---|
| `test_acav_twin` | **47/47 pass** — the MATLAB reproduces every fixture, closed-loop trajectories and the per-step orchestration included |
| `adcs.build_mission_chart` | builds; SAFE latches (0 outgoing), reachable from all four nominal states, all guards at execution order 1 |
| `build_acav_twin` | writes a fully wired `.slx` **and passes Simulink's diagram update** (R2024a on Linux here, R2023b on the deploy machine), with `ros2genmsg` run first |
| `test_simulink_adapter` | **passes** — `adcs.simulink_step`, the Normal-mode adapter the model calls, matches `adcs.twin_step` over 12 closed-loop steps, gates invalid/zero startup telemetry, and resets on simulation start |
| `ros2genmsg` | **succeeds**, R2024a, given a Python 3.8-3.11 interpreter and a `-include cstdint` compile flag the dev box needs but the deploy machine should not (see `docs/C4_scheduler.md` section 2) |

So the algorithms, the chart, message generation and the signal wiring
are all built, and Simulink's own compiler accepts the wiring.
`build_acav_twin` now treats a diagram-update failure as fatal and does
not save the model.

**Correction.** An earlier version of this section said the diagram
update could not pass on this dev box for any custom `stk_msgs` type,
because of a tooling limitation. That was wrong. The model wired each
Subscribe block's output **1** into `UnpackTelemetry`, and output 1 of
`ros2lib/Subscribe` is the boolean `IsNew` flag; the message bus is
output **2**. Every "cannot determine sizes and/or types" and "is not a
bus signal" error came from reading message fields off a boolean. The
control case that seemed to rule this out (the ROS 2 Current Time block
working) has only one output, so it was never exposed to the wrong port.
With the wiring fixed, the same dev-box setup passes diagram update.

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
  The five Subscribe blocks feed their message output (port 2, not the
  `IsNew` flag on port 1) into a consolidating `UnpackTelemetry` MATLAB
  Function, which feeds `GuidanceControl`; that calls
  `adcs.simulink_step`, which runs `adcs.twin_step` once per telemetry
  timestamp. Outputs go out through Blank-Message/Bus-Assignment/
  Header-Assignment chains to Publish blocks that are enabled only
  once telemetry is valid. The model passes diagram update; see
  section 5.
- **Every outgoing stamp comes from `ScenarioTime`, never from a ROS
  clock.** An attitude command is stamped at the time the commanded
  attitude is reached (this step + `step_s`), and a twin status at this
  step's time. The first live run used ROS Header Assignment blocks,
  which stamp with the Simulink node's own clock, and that node is not
  run on simulation time. Every attitude sample went to STK years
  outside the scenario and every status sample outside every hold
  window. The same run showed that `t` reached `adcs.twin_step` as
  absolute epoch seconds rather than seconds since the window start
  (the function's documented contract), which would have fired every
  E5 fault on the first step. Both are fixed in `build_acav_twin`.
- **The twin now closes its own loop.** See section 8: building the
  wiring exposed that `twin_step` never integrated the torque it
  computed, and that is fixed.
- **The `MissionMode` chart is built but deliberately not wired into the
  signal path.** `adcs.twin_step` already calls `adcs.mission_mode`
  directly as a function to decide `state.mode`, entangled with
  slew-profile initialization the chart alone does not capture. Wiring
  the chart in parallel would either duplicate the mode computation with
  no consumer, or need restructuring `GuidanceControl` to take the
  chart's mode as an input — a control-flow change, not a wiring one. It
  stays in the model as the paper's hybrid-model figure.

## 8. The defect the wiring pass found, and what closed it

Building the model's signal wiring turned up two real defects, both in
`+adcs/twin_step.m`, and both for the same structural reason: it was the
**only** part of the twin with no counterpart in `twin_reference.py`, so
no fixture could be generated for it and the cross-check that caught
`quat.align`'s transposed matrix had nothing to say about it. The table
in section 1 had a hole in it and the hole is where the bugs were.

**The vehicle could not move.** `quaternionCmd` was `in.quaternion`
returned verbatim: the twin read an attitude from STK, computed a
torque, and published back the attitude it had just read. Nothing
anywhere integrated that torque — `adcs.wheels` and
`adcs.rigid_body_step` existed and were fixture-covered, but `twin_step`
never called them. The loop had no integrator at all, so every capture
would have been decided by wherever the vehicle already happened to
point, and E2's whole comparison would have been degenerate while
looking perfectly self-consistent.

The design note that justified it — "in the closed loop STK is the truth
and the dynamics are bypassed" — rested on a misreading of what STK does
with injected attitude. `AddAttitude CBFQuat` stores and interpolates
samples; it does **not** propagate attitude dynamics. STK is the truth
for orbit, geometry and access. The vehicle's attitude *response* is the
twin's to compute. `twin_step` now runs the commanded torque through the
wheels, integrates one step from the state STK reported, and publishes
the result — still the ACHIEVED attitude rather than the reference,
which was the part of the original intent that was right.

**SAFE-mode torque disagreed between the two implementations.** The
reference zeroed the torque; the MATLAB applied rate damping. Rate
damping is correct — the point of SAFE is to stop the vehicle tumbling,
and a safe mode that commands nothing leaves the rate that tripped it
exactly where it was — so the reference was corrected to match, and the
decision now lives in one place (`twin_reference.control_torque`) used by
both `simulate_slew` and `twin_step`. No nominal slew enters SAFE, so no
existing expected value changed: the regenerated fixture diff touches
only the version, the new `step_s` field and the new section.

**An honest limit this pinned, which is not a defect.** With reaction
wheels and no desaturation path, SAFE can only remove about
`h_max / J_xx` ≈ 9.3 °/s of body rate before the wheels saturate. A
tumble larger than that is damped down to a residual and then persists
with the wheels pinned at capacity. That is what this vehicle really
does, it is what E5 injects faults into, and a model that quietly
recovered instead would be the more dangerous error — so the
`safe_rate_damping` fixture deliberately runs through saturation rather
than stopping short of it.

The twin also accumulates a running integral of |τ·ω| in its state and
publishes it on `TwinStatus`, alongside wheel momentum and a saturation
flag. It is the only party holding the delivered torque and the body
rate at the same instant, so integrating anywhere else would need both
republished at the telemetry rate — and a running total survives a
dropped sample where a consumer-side integration would silently lose
that step's contribution. Those three fields are what let C5's run
record report peak momentum, saturation events and wheel energy instead
of leaving them at their defaults; see `docs/C5_experiment_harness.md`
section 7.

E5's wheel fault also enters through `twin_step` and nowhere else: the
health schedule is evaluated against the step's own time and handed to
the actuator model, so a dead axis simply stops delivering torque and a
saturated one starts full. The controller is never told, and that is the
point — it keeps commanding torque the vehicle cannot produce. A twin
that reconfigured itself around the failure would be measuring
fault-tolerant control rather than what E5 asks about. See
`docs/C5_experiment_harness.md` section 3 for the two fault kinds and
which one actually trips SAFE.

`twin_step` now has a fixture section of its own (`twin_step`, fixture
version 5) that replays whole runs closed-loop — each step's command fed
back as the next step's measurement, exactly as the deployed loop does
through STK. A twin that computes a torque without integrating it shows
up there as a boresight error that refuses to decrease, which is the
regression boundary that was missing.
