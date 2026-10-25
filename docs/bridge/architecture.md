# Architecture

How the framework is built today: what each part does, how a run moves
through it in each execution mode, and how a result is tied back to the
code, configuration and scenario that produced it. Every claim names
the file (and function) it describes, so it can be checked against the
code. Design rationale is collected in
[design decisions](design_decisions.md).

## 1. What the framework is

A co-simulation framework that couples **STK 12.6 (2023 R1)**, the
physics and geometry reference, to a **MATLAB R2023b / Simulink**
digital twin of a satellite's attitude control system, over **ROS 2
Humble**. STK propagates the orbit and evaluates access geometry; the
twin integrates the attitude dynamics and the controller; ROS 2 carries
telemetry, commands and the shared clock between them.

The case study built on it, ACAV (Attitude-Coupled Access Verification,
[case study](../acav/case_study.md)), asks whether an imaging
schedule that is feasible on geometry alone is still feasible once the
attitude dynamics are in the loop.

Non-goals: the framework is not a flight software stack, does not
replace STK's own analysis tools, and does not make a free-running
co-simulation deterministic. It measures what lockstep and free-run
cost and change (experiments E3 and E6).

## 2. Components

| Package | Kind | Entry points | Role |
|---|---|---|---|
| `src/stk_msgs` | ament_cmake | -- | All ROS interfaces: telemetry, commands, status, services, actions, plus the application messages `TargetSetpoint` and `TwinStatus`. |
| `src/stk_bridge` | ament_python | `python -m stk_bridge.nodes.bridge_node` (`nodes/bridge_node.py`) | The bridge: the only process that talks to STK. Clock authority, sensing, actuation, services. |
| `src/stk_matlab` | MATLAB (not a colcon package) | `run_twin_session.m`, `simulink/build_acav_twin.m`, `setup_stk_ros.m` | The digital twin (`+adcs`), the Simulink model generator, and the client helpers (`+stkros`). |
| `src/acav` | ament_python | `scripts/run_acav_run.py`, `run_acav_campaign`, `build_acav_scenario`, `verify_acav_scenario`, `acav_scheduler`, `scripts/analyze_coupling.py` | The case study: scenario builder and verifier, scheduler node, single-run orchestrator, campaign runner, analysis. |

**`stk_bridge`** has three layers, and nothing above `backends/` imports
`agi.stk12`:

- `backends/`: one interface (`backends/base.py`, `StkBackend`) with
  four implementations. `com_backend.py` (STK Desktop over COM, the one
  every experiment uses), `connect_backend.py` (STK Connect over TCP
  5001), `engine_backend.py` (STK Engine; no licence on this project's
  host, never exercised), and `mock_backend.py` (analytic, for tests;
  records made with it are never reportable,
  `acav/run_record.py` `unreportable_reasons`). The COM backend only
  attaches to an STK Desktop the operator started; it starts one only
  when `backend.com.start_stk` is set (`com_backend.py`
  `_obtain_application`).
- `core/`: no ROS imports. Time (`time_sync.py`, `free_run_pacer.py`),
  sensing (`sensing.py`, `rate_group.py`, `orbit_batch.py`,
  `orbit_block.py`), actuation (`actuation.py`), scenario setup and
  convention validation (`scenario_manager.py`, `validation.py`),
  the COM worker thread (`com_thread.py`), health and status
  (`health.py`, `step_timing.py`, `graphics_state.py`,
  `startup_phases.py`).
- `interfaces/` and `nodes/`: ROS publishers, subscribers, services,
  actions, QoS profiles, and the node that wires them to `core/`.

**The twin** (`src/stk_matlab`) is generated, not checked in by hand:
`build_acav_twin.m` writes `acav_twin.slx` for one run's configuration.
Its per-step logic is MATLAB code in `+adcs` (`simulink_step.m` is the
Simulink adapter, `twin_step.m` one control step, `geometry_at.m` the
causal target geometry). `run_twin_session.m` builds and runs the model
headless (`matlab -batch`) for the automated runs. See
[ADCS twin](../matlab/adcs_twin.md).

**`acav`**: `scheduler_node.py` chooses targets and scores captures;
`deploy_run_cli.py` (`run_acav_run.py`) runs one run end to end;
`campaign_cli.py` / `campaign_runner.py` (`run_acav_campaign`) plan,
run, resume and report a matrix of runs; `run_record.py` defines the
record; `metrics.py`, `capture_tracking.py` and `coupling_analysis.py`
compute the results. See [scheduler](../acav/scheduler.md),
[experiment harness](../acav/experiment_harness.md) and
[analysis](../acav/analysis.md).

## 3. Deployment topology

Everything runs on one Windows host (topology T1): STK 12.6 Desktop,
the bridge, ROS 2 Humble (Windows binaries, DDS over loopback), MATLAB
R2023b and the case-study nodes. `agi.stk12` in STK 12.6 is in-process
only, so the bridge must run on the STK host. The split topologies
(Linux ROS graph, Connect backend) and STK Engine are possible through
the backend interface but are not used or tested by this project.

The host used for every recorded run has 2 physical cores (4 logical,
Intel Core i5-6300U); this matters for every timing number (E6) and for
free-run behaviour (E3).

## 4. Execution mode: lockstep

The bridge is the clock authority and nothing advances until every
declared participant has acknowledged the current step. The ACAV
configuration declares two participants, `matlab_twin` and
`acav_scheduler` (`src/acav/config/bridge_params_acav.yaml`,
`time.barrier_participants`). One step of
`nodes/bridge_node.py` `_lockstep_iteration`:

```
bridge                                   twin (Simulink)        scheduler
  |  1 (after step 0) wait for the AttitudeCommand stamped for the step
  |    about to be taken; missing -> abort. Apply it (actuate).
  |  2 advance STK time by step_s (0.1 s): t_k
  |  3 publish /clock + ScenarioTime(t_k), then sense and publish telemetry
  |---- telemetry, ScenarioTime(t_k) ------->|                    |
  |                                          | integrate to t_k   |
  |<--- AttitudeCommand (stamp t_k + 0.1) ---|                    |
  |<--- ack "matlab_twin:k" -----------------|                    |
  |<--- ack "acav_scheduler:k" ----------------------------------|
  |  4 wait for every ack of step k (barrier, timeout 30 s)
  v  (repeat)
```

Step 1 is `time.require_attitude_command_each_step` (on for ACAV): an
ack and a command travel on different topics and ROS 2 does not order
them, so the bridge waits for the command itself rather than trusting
the ack's arrival.

Why it is deterministic: the order of every exchange is fixed by the
barrier, and no step depends on wall time. The same inputs give the
same commands, twin status and record signature on any host speed;
E6_smoke is the reference signature every change is checked against.
`test_lockstep_trace_golden.py` pins the iteration's stage order and
epochs. At step 1 the bridge republishes the same initial step until
late participants (MATLAB, the scheduler) have joined
(`time.wait_for_startup_participants`). A barrier timeout aborts the
run by default (`time.on_barrier_timeout: abort`).

## 5. Execution mode: free-run

Simulation time follows the wall clock times the real-time factor:
`t = t0 + (wall - wall0) * rtf` (`core/time_sync.py` `now`). Nothing
waits for anyone, so how far the twin gets per published sample
depends on the host. `nodes/bridge_node.py`:

1. **Start gate** (`_arm_startup_gate`, `_free_gate_iteration`). With
   `time.wait_for_startup_participants` the clock starts paused at
   exactly t = 0, senses once, republishes that state, and opens only
   when every participant has acked. The wait is written to
   `startup_gate.json`; without it the window would run on while MATLAB
   and the scheduler start.
2. **Fixed-rate pacer** (`core/free_run_pacer.py`). Cycle k is
   scheduled at `start + k * period`, the period of the fastest rate
   group (0.1 s). A cycle that finishes early sleeps until the next
   start; one that overruns is followed at once and counted
   (`free_run_pacing.json`: cycles, overruns, period).
3. **Order within a cycle** (`_free_running_iteration`): apply
   commands, sense and publish telemetry, *then* publish /clock and
   ScenarioTime with the same time. Publishing the clock before sensing
   leaves the twin unable to step for a whole sensing time per cycle.
4. **The twin sub-steps** (`+adcs/simulink_step.m`). Simulink runs at
   its own paced rate (PollingRateHz, 100 by default), and on each new
   ScenarioTime integrates as many 0.1 s steps as the clock moved, with
   the commanded geometry propagated causally to each sub-step
   (`+adcs/geometry_at.m`) rather than held. Statistics go to
   `twin_substeps.json` and `twin_loop.csv`.

The run orchestrator applies the mode to the bridge and to the twin's
run config (`deploy_run_cli.py` `write_bridge_config`,
`prepare_run_config`); `paced` and `external` exist in
`core/time_sync.py` but are not used by any experiment.

## 6. Sensing paths

Each step the bridge reads position/velocity and attitude/body rate for
each satellite (`core/sensing.py`), through the data providers that
startup validation proved correct on this install. Two optimisations
cut COM round trips; both are recorded per run (`sensing_mode`,
`orbit_batch_samples` in `summary.json`):

- **Combined attitude/rate** (`sensing.combine_attitude_rate`):
  one `Attitude Segment Quaternions` query serves both attitude and
  rate, after a check at a sample the bridge itself wrote proves that
  its quaternion equals the validated `Attitude Quaternions`. On for
  ACAV in lockstep. **Off in free-run**: that check needs a sense epoch
  that is a written sample, which a free-run epoch never is
  (`bridge_node.py` `_apply_free_run_sensing_policy`). Off for E1, which
  compares read-back conventions (`deploy_run_cli.py`
  `write_bridge_config`).
- **Orbit batching** (`sensing.orbit_batch_samples`, 100 for
  ACAV): position and velocity for the next 100 epochs come from two
  interval queries, used only if the block validates and both ends
  agree with instantaneous reads (1e-6 m, 1e-9 m/s;
  `core/orbit_batch.py`). **Lockstep only**: free-run would read epochs
  ahead of the wall clock. Off for E1.

A channel with no data at an epoch publishes nothing for that epoch,
never a zero or a repeated value (`interfaces/ros_publishers.py`).

## 7. Actuation

The twin publishes an `AttitudeCommand` per step, stamped for the next
step. The bridge applies it with `AddAttitude` on the satellite's
real-time attitude (`core/actuation.py`): the inertial `Quat` or the
Earth-fixed `CBFQuat` form, whichever validation paired with the
read-back provider (`check_attitude_pairing` refuses to start if they
disagree). STK rejects a sample earlier than the newest one it holds,
so actuation keeps stamps monotonic per object, clears the object's
history before its first command (`SetAttitude ClearData`), and records
the newest written stamp (`last_attitude`).

**Access deferral.** `ComputeAccess` over an interval ending at t_i is
answered only once the written attitude reaches t_i; until then the
bridge fails the request with the prefix `attitude not yet written
through the stop time` (`core/actuation.py` `ATTITUDE_NOT_COVERED`,
`attitude_coverage_gap`), and the scheduler asks again
([scheduler](../acav/scheduler.md)). Without it, access was computed
against STK's default attitude beyond the last written sample.

## 8. Health and status

- `BridgeStatus` on `/stk/status`, once per wall second
  (`bridge_node.py` `_publish_status`): state, backend, STK version,
  dropped samples, queue depth, the attitude provider in use, and E6's
  mean sense, actuate and barrier-wait times over the last second with
  their sample count (`core/step_timing.py`). The state comes from
  `core/health.py`: STARTING, CONNECTED, DEGRADED, DISCONNECTED, ERROR.
  During startup the bridge publishes STARTING at every phase change.
- **Startup phases** (`core/startup_phases.py`): connect, each
  scenario-setup step, each validation check, sensing and interfaces,
  timed and written to `bridge_startup_phases.json` as they happen. The
  run orchestrator waits up to 180 s for CONNECTED and records the
  phases in `summary.json`.
- **STK graphics state** (`core/graphics_state.py`): 2D/3D window
  counts and the main window state, sampled at startup, during the run
  and at the end (`stk_graphics.json`). Closing the graphics windows
  and minimising STK cut the E6 step from 120.3 to 70.2 ms, so the
  runner requires "off" unless asked otherwise.
- `BackendDiagnostics` on `/stk/diagnostics`: the backend's own
  counters and the validation report.

## 9. Provenance chain

What ties a number in the paper to the run that produced it:

1. **Run config** (`<run>.runconfig.json`, `deploy_run_cli.py`
   `prepare_run_config`): run id, execution mode, gains and fault for
   the twin. Written before anything starts; an existing file for
   another run is refused.
2. **Twin identity v3** (`acav/twin_identity.py`, published by the
   twin on `/acav/twin_identity`): `v3 <cfg> <build> <runtime>
   <polling>Hz`, the sha256 of the run config the twin parsed, the code
   fingerprint baked in at build and the one recomputed at simulation
   start, and the Simulink pacing rate. Fingerprints hash file text with
   CRLF turned into LF, so the checkout's line endings do not change
   them. The runner starts the scheduler only if all of them match this
   run and this checkout (`check_identity`).
3. **Scenario digest** (`run_record.py` `scenario_digest`): sha256 of
   the scenario YAML with CRLF turned into LF. Records made before that
   normalisation carry the raw digest of the reference scenario; the
   committed map `LEGACY_SCENARIO_DIGESTS` takes it to the normalised
   one.
4. **Run record** (`run_record.py` `RunRecord`): the result (targets,
   outcomes, determinism signature) and its provenance (backend, STK
   version, execution mode, scenario digest/seed/generator version,
   gains, fault, attitude provider observed from the bridge, code
   version, twin identity, graphics mode, real-time factor, MATLAB
   version, STK process). The signature is recomputed from the result
   on every read, and a stored one that differs is an error.
5. **Reportability** (`RunRecord.unreportable_reasons`): not a real
   STK backend; a lockstep target with no twin telemetry in its hold
   window (`no_telemetry`); no access answer (`no_access_answer`);
   access never computed over a written attitude (`access_unverified`).
6. **Target coverage** (`target_coverage_problems`): a record must
   name a known scenario and contain exactly that scenario's targets.
7. **Per-run `summary.json`**: every orchestration step with its
   outcome, plus run conditions (graphics, rtf, start gate, pacing,
   startup phases, STK process, MATLAB session).

`run_acav_campaign report` refuses to pool records whose conditions
differ: graphics mode, twin launch mode, how STK was started, scenario,
diagnostic labels ([experiment harness](../acav/experiment_harness.md)).

## 10. Diagram

```
 +----------------------------- Windows host (T1) ------------------------------+
 |                                                                              |
 |  STK 12.6 Desktop  <--COM (one worker thread)-->  stk_bridge (Python)        |
 |  (started by the operator)                        core/ time, sensing,       |
 |                                                   actuation, validation      |
 |                                                         |  ROS 2 (loopback)  |
 |        +------------------------------------------------+-------------+      |
 |        | /clock, /stk/scenario_time      /stk/<sat>/attitude, rate,   |      |
 |        | /stk/status, /stk/lockstep/ack  orbit;  /stk/cmd/attitude    |      |
 |        v                                                              v      |
 |  MATLAB/Simulink twin  --/acav/twin_status, /acav/twin_identity-->  acav_    |
 |  (matlab -batch,        <--/acav/setpoint-------------------------  scheduler|
 |   run_twin_session.m)                                   ComputeAccess -> bridge
 |                                                                              |
 |  run_acav_run.py: launches bridge, twin, scheduler; checks each step;        |
 |  writes <run>.json (record) and <run>.logs/ (summary.json, logs, topics)     |
 +------------------------------------------------------------------------------+
```

`figures/bridge_architecture.svg` is the original bridge-only drawing;
the per-step sequences are in sections 4 and 5.

## 11. Known constraints

- **One COM thread.** Every STK call runs on the backend worker thread
  (`core/com_thread.py`); ROS callbacks never touch STK. A slow COM call
  delays the whole step.
- **2-core host.** Bridge, STK, MATLAB, the scheduler and DDS share two
  physical cores; free-run results and every timing depend on that load.
- **Message types.** After any `.msg` change, rebuild `stk_msgs` with
  colcon *and* regenerate MATLAB's message types (`setup_stk_ros`,
  [MATLAB client](../matlab/client.md)); the twin's code
  fingerprint covers the message definitions, so a stale model is
  refused by the identity check.
- **Terminal-epoch NO DATA.** At t = 720 s, the scenario's stop time,
  `Attitude Segment Quaternions` returns no data: the body-rate sample
  for that step is not published, the twin cannot acknowledge it, and
  the bridge logs the missing sample. The scheduler has written its record before then;
  results are not affected.
- **PowerShell 5.1 quote stripping.** Windows PowerShell 5.1 removes
  embedded double quotes from arguments passed to native programs, so
  a JSON value typed on the command line (e.g. a fault) arrives broken.
  Pass such values through a file (`--run-config`), a campaign
  placeholder (`{fault_json}`, substituted by Python), or a Python
  launcher; `acav/stk_process.py` builds its PowerShell script without
  double quotes for the same reason.
- **STK started through COM.** An STK Desktop started by automation
  (`AgUiApplication.exe -Embedding`) ran E6 at 95.7 ms against about
  60 ms (one run); runs refuse it
  ([STK notes](stk_notes.md)).
