# The ADCS twin

The MATLAB/Simulink digital twin the bridge drives, and the Python
reference that pins its semantics. Sections 1-3 are the design;
sections 4-6 describe how the twin runs in the loop today; section 7
keeps the design decisions.

## 1. Why there are two implementations

The twin runs in MATLAB on the Windows machine. Its correctness rests on
about three hundred lines of quaternion algebra, rigid-body dynamics and
mode logic, none of which can be tested on the development machine.

A sign error in that code does not crash. The twin flies smoothly to a
slightly — or completely — wrong attitude, STK dutifully reports no
access, the capture rate drops, and the result reads as a control-design
finding. The bridge has the same failure mode
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
exactly where two defects lived undetected (section 7). Every row of
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
guards this. **The table was measured with fixed targets.** In the live
loop the target direction moves, and the guidance compensates for it
(section 7): live slews end within ~1.4° and enter IMAGE immediately, so
these settling margins are conservative for live behaviour. If a gain
change breaks the budget, E2's headline becomes an artifact, so the test
fails rather than the study quietly degrading.

**Honest caveat for the paper.** A 1.0 rad/s bandwidth is optimistic for
a 6U CubeSat, because the twin models no sensor noise, no attitude
estimation lag and no wheel dynamics. It is a fair basis for comparing
*schedulers* — both face the same vehicle — but it is not a claim about
achievable CubeSat pointing performance, and must not be presented as
one.

## 4. Mission modes

```
NADIR --slew requested--> SLEW --profile done--> SETTLE
  ^                                                 |
  |                                     error <= 2 deg
  |                                                 v
  +--------------- slew requested <------------- IMAGE

  any state --momentum >= capacity or rate > 15 deg/s--> SAFE (latched)
```

Two properties are load-bearing and tested: the SAFE guards are
evaluated first, so a wheel that saturates during IMAGE still trips
SAFE; and SAFE latches, so a fault run has one well-defined outcome to
replay. Recovery is an operator action.

**Where the mode is actually decided.** `adcs.twin_step` calls
`+adcs/mission_mode.m`, the transition logic as a function, every step;
that is the mode the twin uses and publishes on `TwinStatus.mode`. The
Stateflow chart `MissionMode` (`adcs.build_mission_chart`) is built into
the model and structurally checked (SAFE latches, guard order), but it
is **not wired into the signal path** and drives nothing
(`simulink/build_acav_twin.m`, the comment at the chart). Mode entry is
entangled with slew-profile initialisation in a way the chart alone does
not capture; wiring it would mean restructuring `GuidanceControl` to
take the chart's mode as an input. Until that is done, the paper may
show the chart as the specification of the mode logic, but must not
describe it as executing. This is a known
limitation (see the developer guide).

## 5. The twin in the loop

**Model.** `simulink/build_acav_twin.m` generates `acav_twin.slx` for
one run: Subscribe blocks (attitude, rate, orbit, setpoint,
ScenarioTime) feed `UnpackTelemetry`, which feeds the MATLAB Function
`GuidanceControl`; it calls `adcs.simulink_step`, which calls
`adcs.twin_step`. Outputs go to Publish blocks: `/stk/cmd/attitude`,
`/acav/twin_status`, `/stk/lockstep/ack` (lockstep) and
`/acav/twin_identity`. Every outgoing stamp comes from the telemetry's
simulation time, never a ROS or wall clock. The `.slx` is a build
artifact; edit the builder.

**Per step** (`adcs.twin_step`): read the setpoint, choose the mode,
plan or continue the slew profile, compute the control torque, pass it
through the wheel model (limits, momentum, faults), integrate the
rigid body one 0.1 s step, and output the achieved attitude, the mode,
the boresight error and the wheel state. The twin owns attitude and body
rate: STK's read-back is checked against the twin's own previous output
(a mismatch beyond about 1e-4 rad stops the twin), except in E1's
explicit ablation.

**Lockstep** (run config `execution_mode` "lockstep"): one step per
new ScenarioTime, then the attitude command for the next step, then the
ack.

**Free-run** (`adcs.simulink_step`): samples arrive at
wall-clock-driven times. With t0 the first ready sample and k the steps
integrated so far, a sample at t integrates
`n = floor((t - t0)/0.1 + 1e-6) + 1 - k` steps; n = 0 holds the last
outputs. Sub-step j runs at `t0 + j*0.1` on the newest sample stamped at
or before it, with that sample's position and target moved forward to
the sub-step's time (`+adcs/geometry_at.m`: position along the
velocity, the Earth-fixed target turned with the Earth). Holding one
sample's geometry over every sub-step pulsed the controller's reference
rate and cost captures; a sample on the step grid is used as received,
so on-grid free-run equals lockstep bit for bit. Sub-step statistics
(occurrences of n > 1, the largest n, the stamp offset) go to
`<run>.logs/twin_substeps.json`; about once per wall second the call
counts go to `<run>.logs/twin_loop.csv`
(`wall_local,calls,ready,fresh,holds,t_s`).

**What a status reports between publishes.** Every integration step,
sub-steps included, feeds `saturation_events` (cumulative rising edges
of the saturation flag) and `max_boresight_error_since_last_rad`
([messages](../bridge/messages.md)).

**Pacing.** Simulink pacing is on at `PollingRateHz` (default 100)
model steps per wall second (`build_acav_twin.m`). Offline on the deploy
host, 200 Hz and above made Simulink busy-wait about one core, while
100 Hz was held to about 66 steps/s. The rate is part of the identity.

**Identity.** `v3 <cfg> <build> <runtime> <polling>Hz`, fixed at reset,
published on the first ready step and then at most once per wall second
(`acav/twin_identity.py`; [architecture](../bridge/architecture.md)
section 9).

**Faults** (`+adcs/wheel_fault.m`): a seeded wheel fault from the run
config, evaluated against the step's own time and applied in the
actuator model only. The controller is not told; it keeps commanding
torque the vehicle cannot produce, which is what a fault run tests. The
fault's axis is zero-based in the record and converted in one place.

**Parameters.** `adcs.config` reads the reference scenario YAML (always
the default path, never a seeded scenario; the runner refuses a scenario
whose twin-relevant sections differ) and the run config for gains,
fault and execution mode.

## 6. Running and testing it

Runs: manual (build, open, Play) or automated (`matlab -batch
run_twin_session`); see [MATLAB client](client.md).

MATLAB tests in `src/stk_matlab`, each run from that folder after
`setup_stk_ros`:

| Test | Checks |
|---|---|
| `test_acav_twin` | Replays the Python reference's fixtures (`acav/twin_fixtures.py`, `export_twin_fixtures`) through the MATLAB, to 1e-9. Refuses stale fixtures. |
| `test_simulink_adapter` | `adcs.simulink_step` against `adcs.twin_step` over closed-loop steps; startup gating; reset. |
| `test_free_run_twin` | Free-run sub-stepping, causal geometry, the TwinStatus fields' classes and values. |
| `test_twin_model_runs` | Builds the model and **runs** it for 1 s. A diagram update passes even when a block's output class is wrong; running it does not (a wrong output class stopped every live run at its first step). |
| `test_twin_identity` | The identity text and the shared fingerprint vector. |
| `test_settle_retarget`, `test_target_rate_reset` | Re-targeting rules. |
| `test_e1_ablation` | E1's explicit mismatch-guard ablation. |

After any change to the twin or to a `.msg` file, run
`test_twin_model_runs` before a live run.

## 7. Design decisions

Each explains why the code is the way it is:

- **The twin integrates its own dynamics.** `AddAttitude` stores and
  interpolates samples; STK does not propagate attitude dynamics. A
  `twin_step` that only published back the attitude it had read would
  never integrate its torque and the vehicle could not move, so it runs
  the torque through the wheels and integrates one step.
- **SAFE applies rate damping**, in both implementations
  (`twin_reference.control_torque`). With no desaturation path, SAFE can
  remove only about `h_max / J_xx` (9.3 deg/s) of body rate before the
  wheels saturate; the fixtures run through that limit.
- **The twin carries its own body rate.** STK's rate is a numerical
  derivative of the attitudes the twin wrote; across a missing sample it
  is enormous (a 155 deg read-back jump became an ~800 deg/s spin).
- **Moving geometry.** Desired-rate feedforward, slew profiles
  transported with the moving line of sight, and re-planning on a change
  of target (including on the step a slew ends) are implemented on both
  sides and covered by the `moving_twin_step` fixtures
  (`test_moving_fixtures.py`).
- **Stamps from ScenarioTime.** Header Assignment blocks stamp with the
  Simulink node's own clock, which is not simulation time; every sample
  would land years outside the scenario.
- **Free-run geometry is propagated, not held**, and the twin's
  loop is paced; see section 5.
