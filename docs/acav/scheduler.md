# C4: The ACAV scheduler

The Python ROS 2 node that decides which target to point at and when,
and scores what actually happened. This is what makes E2 (kinematic vs.
dynamics-aware scheduling) exist as two runnable things rather than two
paragraphs of description.

## 1. What the scheduler does and does not do

It does target **sequencing**: for each target in the scenario, decide
whether to attempt it, and if so, when to switch the guidance target to
it. It does **capture scoring**: combine the twin's own telemetry with
STK's access answer into a `RunResult` via the shared
`acav/metrics.py` criterion. It does **not** compute attitude. Guidance
(turning "point here" into a commanded quaternion, honouring this
vehicle's boresight axis and a velocity-referenced roll) lives entirely
in the twin (`acav/twin_reference.py` / `+adcs/twin_step.m`) — see
section 2 for why that boundary is drawn there and not here.

Two pure modules carry all the logic that matters, and both are fully
offline-tested (`test_scheduling.py`, `test_capture_tracking.py`),
matching this project's own established boundary: `core`-shaped logic
is unit-tested without ROS; only the thin node wiring around it needs a
sourced ROS environment to exercise.

| Module | Role |
|---|---|
| `acav/scheduling.py` | `plan_kinematic` / `plan_dynamics_aware` — the entire difference between the two policies E2 compares |
| `acav/capture_tracking.py` | Buffers telemetry, applies `metrics.evaluate_capture` once a target's access answer is known |
| `acav/scheduler_node.py` | Thin ROS wiring: `SchedulerNode` (lazy rclpy/stk_msgs imports, DI-constructed like `nodes/clock_node.py`) + `main()` |

## 2. Why the scheduler-twin interface needed two new messages

The architecture names `/acav/setpoint` (scheduler → twin) and expects
the twin to report enough back to score a capture. Neither had a wire
format before this phase. Two small additions to `stk_msgs`:

- **`TargetSetpoint`** (`/acav/setpoint`): a target id, an inertial
  position, and a `has_target` flag. Deliberately data, not an
  attitude — the guidance law stays in the twin (see below).
- **`TwinStatus`** (`/acav/twin_status`): the twin's own Stateflow mode
  name and its own `boresight_error_rad`.

They live in `stk_msgs` rather than a new package (see
`docs/03_message_reference.md`'s "Application-level messages" section
for the reasoning) and were verified to `colcon build` cleanly and
round-trip through `ros2 interface show` in the `humble-desktop-full`
container.

**A real interface bug this caught.** `build_acav_twin.m`'s
`SetpointSub` block was wired to `stk_msgs/AttitudeCommand` for
`/acav/setpoint` — a type that carries a quaternion, and cannot carry
the target position `adcs.twin_step`'s `targetEci` input actually
needs. It was never exercised (the model's signal wiring has never been
built — see `docs/C3_adcs_twin.md` section 7), so nothing had yet
depended on the wrong type. Fixed as part of defining this contract,
since C4 is where it is produced.

**`ros2genmsg` now runs in this development environment (2026-09-14),
which is how the two messages above got verified beyond a `colcon
build`.** Two independent blockers, both worked around without
touching the MATLAB installation itself:

1. R2024a's (and R2023a's) ROS Toolbox needs a Python interpreter in
   [3.8, 3.11); the dev box's only system Python is 3.14. Fixed by
   extracting a Python 3.10 (with `venv`/`pip` support — the base
   `osrf/ros:humble-desktop-full` image needs `python3.10-venv` and
   `python3-pip` installed into it first) from that same container and
   pointing MATLAB's ROS Toolbox at it via
   `setpref('ROSToolbox', 'ROS2_PYTHON_EXECUTABLE', <path>)` — an
   internal but stable preference key.
2. This host's GCC 15.2.0 exposes a missing `#include <cstdint>` in
   ~53 of MATLAB's own bundled ROS 2 headers (they relied on a
   transitive include older GCCs provided). Fixed with
   `setenv('CXXFLAGS', '-include cstdint')` /
   `setenv('CPPFLAGS', '-include cstdint')` before calling
   `ros2genmsg` — colcon's CMake invocation picks both up, so no
   vendor file needs editing.

With both set, `setup_stk_ros` completes, `ros2genmsg` builds `stk_msgs`
(including these two new messages) cleanly, and `build_acav_twin`
proceeds past `requireMessages()` to a real, fully wired `.slx`. The
full recipe is in the local (untracked) working notes, since neither
workaround belongs in this repository — the DEPLOY machine has a Python
in range already and should never need the `cstdint` flag at all if its
toolchain is the one MATLAB was built against.

**The "third dev-box limitation" reported here earlier was a wiring bug,
not a tooling one.** This section used to claim that on this
R2024a-on-Linux setup no MATLAB Function or Bus Selector could read a
Subscribe block's output for any custom `stk_msgs` type, and blamed a
GLIBCXX mismatch in MATLAB's ROS 2 server subprocess. The real cause
was simpler: the model wired each Subscribe block's output **1** into
`UnpackTelemetry`. On `ros2lib/Subscribe`, output 1 is the boolean
`IsNew` flag and output 2 is the message bus. Every "cannot determine
sizes and/or types" and "is not a bus signal" error came from reading
message fields off a boolean. The evidence that seemed to point at the
toolchain fits the wiring bug just as well: blocks that never read a
field (a Terminator) could not fail, and the ROS 2 Current Time block
used as a control has a single output, so its port 1 really is the
message.

Wiring port 2 fixed it. The corrected model passes diagram update on
this same dev box, with the same `ros2genmsg` recipe as above, and on
the deploy machine. `build_acav_twin` now fails, rather than warns,
when diagram update fails. See `docs/C3_adcs_twin.md` section 5.

**Who computes the pointing error, and why not the scheduler.** The
twin does, in `TwinStatus.boresight_error_rad`. It already has the
freshest (satellite state, target) pair at the instant it computes that
angle for its own SETTLE→IMAGE transition; a second, independently
written computation of the same angle here would risk exactly the
failure C1 flags for the slew model — two models measuring their own
disagreement instead of the effect under study. The scheduler trusts
it and does not re-derive it from raw telemetry.

**Who supplies target geometry.** The scheduler does, by calling the
bridge's own `query_data_provider` service for each attempted target's
`Cartesian Position/ICRF`, once, at its own image epoch (a target is
static in STK's Fixed frame, so its *inertial* position is a function
of time via Earth's rotation, and only needs evaluating once per
target for a one-shot guidance target). This is a handful of calls for
the whole run, not a per-step cost — response parsing reuses
`core/data_providers.py`'s own case-insensitive column resolution
(`resolve_columns`/`CARTESIAN_POSITION`) rather than assuming a column
naming convention this project has already been burned by assuming
once (see the bridge's own STK 12.6 findings).

**Who decides access.** STK does, via one `compute_access` call per
target, issued only once `sim_time_s` has passed that target's image
epoch by a small margin — asking earlier would ask STK a question
about attitude history that has not happened yet.

## 3. The two policies

Both walk the target list in image-time order and switch the guidance
target at the same instants: immediately after the previous ATTEMPTED
target's image epoch (or window start, for the first). Neither ever
starts a target early "just in case" — every attempted target gets the
full gap the pass geometry allows, which is what makes the comparison
about scheduling, not about how generous either policy is with lead
time.

- **Kinematic** attempts every target, unconditionally — "assumes
  instantaneous, perfect pointing" taken at face value: since
  reorientation is believed to cost no time, there is never a reason to
  decline.
- **Dynamics-aware** declines a target when `AcavSpec.
  estimated_slew_time_s` — the same lower-bound model the twin's
  profile generator is built from — exceeds the time actually
  available. Declining is skip-*aware*: it does not spend the declined
  target's slot, so the following target inherits the extra time.

`test_scheduling.py` pins the reference scenario's stated intent
(`config/acav_scenario.yaml`'s target comments: "a kinematic scheduler
attempts all 7; a dynamics-aware one should decline at least T5 and
T7") as an executable check, and separately confirms that declining T5
measurably extends T6's available time versus the static
`feasibility_table()` (which is deliberately non-reactive — see its own
docstring — and is not what the live scheduler uses to decide).

## 4. Lockstep participation

The bridge's ACAV configuration names `acav_scheduler` as a barrier
participant (`src/acav/config/bridge_params_acav.yaml`);
`SchedulerOptions.participant_name` defaults to the same string. The ack
is published on every `scenario_time` tick before anything slow is
considered. Target-position and access requests are `call_async` with a
done-callback, never a blocking `call()`, so an STK round trip can never
make an ack late.

**Target positions.** Before the first lockstep step is released, each
target's position is resolved as an ICRF trajectory sampled every 10 s
over the window (`TARGET_POSITION_STEP_S`; interpolation error below
0.5 m) and interpolated per tick, rather than one position at the image
epoch.

## 5. Scoring a target

For each attempted target with image epoch t_i:

1. **Hold window.** TwinStatus samples stamped in [t_i - 2 s, t_i] are
   buffered (`acav/capture_tracking.py`). Each sample's pointing error
   for the hold check is `max_boresight_error_since_last_rad` when the
   twin reports it, so an excursion between two published samples in
   free-run counts (`metrics.PointingSample.hold_error_deg`).
2. **Access.** At `t_i + access_query_delay_s` (0.2 s) of scenario time
   the scheduler calls the bridge's `compute_access` from the sensor to
   the target over the hold window. If the satellite's written attitude
   does not yet reach t_i, the bridge answers with the
   `ATTITUDE_NOT_COVERED` prefix ([03_message_reference.md](03_message_reference.md));
   the scheduler records the deferral and asks again on the next tick.
   In lockstep the first request is always covered.
3. **Outcome** (`metrics.evaluate_capture`): access at t_i and the hold
   condition give `captured`, `not_settled`, `no_access` or
   `no_access_and_not_settled`; SAFE at t_i gives `safe_mode`; no sample
   in the hold window gives `no_telemetry`. Each result records how long
   the access answer was deferred (`access_wait_s`, scenario and wall).

**Run end.** After the window's end the scheduler finishes once every
target is accounted for, or after a 15 s wall grace
(`FINALIZE_GRACE_WALL_S`). A target still unscored then is recorded
`access_unverified` if its access was deferred because the attitude
never reached t_i, otherwise `no_access_answer`; either makes the
record unreportable ([C1_case_study_spec.md](C1_case_study_spec.md)
section 6).

**Run-level quantities** (`capture_tracking.RunAggregate`): peak momentum
fraction per axis, wheel energy (the largest cumulative value seen),
whether SAFE was entered, and saturation events: the larger of the
scheduler's own count of rising edges in the published
`wheels_saturated` flag and the twin's cumulative `saturation_events`.
In lockstep the two are equal; in free-run the twin's count includes
sub-steps nobody saw published (the seed-4 fault re-run on 3186642
counted the T3 blip at t = 127.235 s this way).

**The record** (`_write_output`): the result plus provenance (backend
and STK version as reported by the bridge, the attitude provider
observed on BridgeStatus, scenario digest, seed and generator version,
code version, twin identity). The run orchestrator adds the run
conditions afterwards (graphics mode, real-time factor, MATLAB version,
STK process); [C5_experiment_harness.md](C5_experiment_harness.md).

## 6. Running it

The run orchestrator starts the scheduler after the twin's identity has
been checked (`run_acav_run.py`, [08_deploy_runbook.md](08_deploy_runbook.md)).
By hand:

```
ros2 run acav acav_scheduler --ros-args \
    -p scenario_config:=<scenario YAML; default: the installed reference> \
    -p scheduler_type:=dynamics_aware \
    -p output_json:=<record path>
```

It needs a running bridge and a twin publishing `/acav/twin_status`. It
writes `output_json` and shuts down at the end of the run
(`auto_shutdown`, on by default).

## 7. Status

Both policies have run live in lockstep (E2 on the reference scenario
and, on 3186642, the dynamics-aware policy on generated seed 1); see
[09_live_runs_and_results.md](09_live_runs_and_results.md). Offline
coverage: `test_scheduling.py`, `test_capture_tracking.py`,
`test_access_deferral.py`, `test_target_coverage.py`,
`test_twin_status_fields.py`, `test_scheduler_completion.py`.
