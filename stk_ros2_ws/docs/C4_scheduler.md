# C4 — The ACAV scheduler

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

`test_scheduling.py` pins the scenario's own stated intent
(`config/acav_scenario.yaml`'s target comments: "a kinematic scheduler
attempts all 7; a dynamics-aware one should decline at least T5 and
T7") as an executable check, and separately confirms that declining T5
measurably extends T6's available time versus the static
`feasibility_table()` (which is deliberately non-reactive — see its own
docstring — and is not what the live scheduler uses to decide).

## 4. Lockstep participation

`acav_scenario.yaml` names `acav_scheduler` in
`bridge.barrier_participants`; `SchedulerOptions.participant_name`
defaults to the same string, checked against the loaded spec at
construction (a mismatch only warns, since a free-run smoke test has no
barrier to violate, but a lockstep run would time out silently
otherwise).

The ack is published unconditionally, every `scenario_time` tick,
**before** anything that could plausibly be slow is even considered.
Target-position and access-answer service calls are `call_async` plus a
done-callback — never a blocking `call()` — specifically so a slow STK
round trip can never be why a step's ack is late. This is a lighter
version of the same lesson `nodes/bridge_node.py` learned the hard way
(a blocking wait needing an executor thread that a small pool cannot
spare): this node never blocks at all, so it needs no dedicated thread,
just a `MultiThreadedExecutor` for headroom.

## 5. What has actually been executed

Everything below was run against the `osrf/ros:humble-desktop-full`
container on the development machine, not just built.

| Check | Result |
|---|---|
| `colcon build` (whole workspace, incl. the two new messages) | clean |
| `ros2 pkg executables acav` | lists `acav_scheduler` |
| `python3 -c "import acav.scheduler_node"` | succeeds with **no ROS sourced** — confirms the lazy-import discipline holds |
| `ros2 run acav acav_scheduler` standalone (no bridge) | builds its schedule (5 attempted / 2 declined) and idles cleanly |
| `ros2 run stk_bridge bridge_node` (mock, free_run) + `acav_scheduler` together | epoch resolves; `/acav/setpoint` publishes continuously at the fast rate, correctly formatted, degrading to `has_target: false` when a target's position cannot be resolved; `/stk/lockstep/ack` publishes `"acav_scheduler:<step_index>"` every tick |

**A bridge bug this smoke test found and fixed, unrelated to C4's own
logic.** Running `bridge_node` against the mock backend in this
container crashed its free-run status publish every iteration:
`BackendDiagnostics.com_apartment_thread_id` (`uint32`) was assigned
`threading.get_ident()` directly, which is only guaranteed to fit 32
bits on Windows (where this bridge normally runs) — on 64-bit Linux it
routinely does not, and the message library's own bounds assertion
caught it. Masked to the low 32 bits in
`interfaces/ros_publishers.py::publish_diagnostics`; the field is
documented as diagnostic-only, so the mask loses no meaning anything
else depends on. This was never exercised before because mock-backend
testing on Linux is new as of this phase.

The target objects themselves (`*/Target/T1`, etc.) do not exist in the
smoke test above, because `MockBackend` is an in-process Python object
with no state outside the process holding it — `build_acav_scenario.py
--backend mock` run separately builds a scenario into a *different*
`MockBackend` instance than the one `bridge_node` owns, unlike the COM
backend, where both processes are clients of the one real STK.exe. This
is a property of the mock, not a gap in the scheduler: the scheduler's
own logic (which targets, when, scored how) is what section 3's tests
cover, and per C1 section 8, no reported number may come from the mock
regardless.

## 6. Running it

```
ros2 run acav acav_scheduler --ros-args \
    -p scenario_config:=<path to acav_scenario.yaml, default: installed> \
    -p scheduler_type:=dynamics_aware \
    -p output_json:=/tmp/run_dynamics_aware.json
```

Requires a running `stk_bridge` (COM backend, against a live,
C2-verified scenario) and the twin publishing `/acav/twin_status`. On
completion (the scenario window has elapsed and every attempted target
has been scored, or a 10 s grace period after window end) it writes
`output_json` and shuts down, matching `verify_acav_scenario.py`'s
run-once, script-friendly shape for C5's campaign driver to invoke
repeatedly.

## 7. Open items

- **The twin's signal wiring is built** (`docs/C3_adcs_twin.md`
  section 7), turning `/acav/setpoint` and `/acav/twin_status` from
  declared topics into a working loop, and it passes Simulink's diagram
  update (section 2 corrects an earlier wrong diagnosis here). The
  `quaternionCmd` defect that wiring exposed (the twin computed a torque
  but never integrated it, so the vehicle could not slew) is fixed and
  now fixture-covered; see `docs/C3_adcs_twin.md` section 8.
- **No live-STK run yet.** Everything above is verified against the
  mock/free-run combination this environment can reach. The first real
  E2 run is the actual test of sections 2–4's design.
