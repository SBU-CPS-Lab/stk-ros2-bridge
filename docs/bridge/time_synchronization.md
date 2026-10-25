# Time synchronisation

The bridge owns simulation time. It advances STK's clock, publishes it
on `/clock` and `/<prefix>/scenario_time` (`ScenarioTime`: time,
execution mode, real-time factor, step index, paused), and every other
participant takes its time from there. The twin never reads the wall
clock for simulation time. `core/time_sync.py` holds the clock;
`nodes/bridge_node.py` runs the loop on a dedicated thread (section 4).

Two execution modes are used by the experiments, chosen per run by
`run_acav_run.py --execution-mode {lockstep,free_run}`; the runner
writes the mode into the bridge config and into the twin's run config
(`deploy_run_cli.py` `write_bridge_config`, `prepare_run_config`), so
the two cannot disagree. `core/time_sync.py` also accepts `paced` and
`external`; no experiment uses them.

## 1. Lockstep

**Principle.** The bridge advances one step (`time.step_s`, 0.1 s) and
then waits until every participant in `time.barrier_participants` has
acknowledged it. Nothing depends on wall time, so a run's commands,
twin status and record signature are the same on any host speed.

**One iteration** (`_lockstep_iteration`; settings from
`src/acav/config/bridge_params_acav.yaml`):

1. From step 1 on, with `require_attitude_command_each_step: true`,
   wait for the attitude command stamped for the step about to be taken
   (up to `barrier_timeout_s`, 30 s). A missing or rejected command
   aborts the run (`LockstepCommandError`). Apply the commands.
2. Advance: `t_k = t_(k-1) + step_s`, pushed into STK.
3. Publish `/clock` and `ScenarioTime(t_k)`, sense, publish telemetry,
   run the slower rate groups, publish status at 1 Hz wall.
4. Barrier: wait for `"<participant>:<k>"` on `/<prefix>/lockstep/ack`
   from `matlab_twin` and `acav_scheduler`. The twin sends its ack after
   its attitude command for the next step.

**Startup.** With `wait_for_startup_participants: true` the bridge holds
step 1 and republishes it every 2 s (`STARTUP_REPUBLISH_S`) from its
stored samples, without new COM work, until every participant has
joined; that wait is not counted in E6's barrier-wait mean.

**Barrier timeout policy** (`time.on_barrier_timeout`): `abort`
(default and ACAV: the run stops, loudly), `continue` (advances anyway,
logged as no longer deterministic), `wait` (blocks indefinitely).

**What lockstep guarantees, and how it is checked.** The reference
determinism signature is E6_smoke's; every change to the loop is
checked against it by a live E6 run, and offline by
`src/stk_bridge/test/test_lockstep_trace_golden.py`, which drives the
real iteration for 250 steps and compares every stage and epoch with a
golden trace.

## 2. Free-run

**Principle.** Simulation time follows the wall clock times the
real-time factor (`time.real_time_factor`, `--real-time-factor`):

```
t = t_gate + (wall - wall_gate) * rtf          (core/time_sync.py now)
```

Nobody waits for anybody after the start, so what the twin sees per
published sample, and how many sub-steps it integrates at once, depend
on the host's load. E3 measures what that changes.

**Start gate** (`_arm_startup_gate`, `_free_gate_iteration`). The clock
starts paused at exactly t = 0. The bridge senses once, publishes that
state, republishes it every 2 s, and opens the gate when every barrier
participant has acked; the clock then runs from t = 0. The wait is
recorded in `startup_gate.json` and in summary.json
(`free_run_start_gate.wait_s`; e.g. 176.5 s in
results/_archive/20261019_results_cleanup/pilots/E5_P10_free2_3186642,
most of it MATLAB starting and building).
Without the gate the window ran on through MATLAB's start.

**Pacer** (`core/free_run_pacer.py`). Cycles are scheduled at
`start + k * period`, the fastest rate group's period (0.1 s of
simulation time). A cycle that ends early sleeps to the next scheduled
start; a cycle that overruns is followed at once, the schedule is
rebased (no catch-up burst), and the overrun is counted.
`free_run_pacing.json` records cycles, overruns and the period.

**Order within a cycle** (`_free_running_iteration`): apply the
commands that have arrived, sense and publish telemetry, then publish
`/clock` and ScenarioTime with the same time, then the slower rate
groups and status. The clock is published after sensing because the
twin steps only when the telemetry it holds is stamped with the latest
ScenarioTime: published first, it left the twin unable to step for a
whole sensing time per cycle (E3 at rtf 1.0 on 4b26de0: 31% of samples
answered; fixed in 1628866).

**The twin in free-run** (`+adcs/simulink_step.m`). Simulink runs paced
at `PollingRateHz` (100) model steps per wall second. Each time
ScenarioTime has moved, the twin integrates `n` 0.1 s sub-steps to catch
up (zero-order hold of the latest command and telemetry), with the
target geometry propagated causally to each sub-step
(`+adcs/geometry_at.m`) instead of held at the last published value.
In the TwinStatus it publishes, `saturation_events` counts every
sub-step, and `max_boresight_error_since_last_rad` is the largest error
over the sub-steps integrated since the previous status ([messages](messages.md)).
Sub-step statistics go to `twin_substeps.json`; per-wall-second counters
to `twin_loop.csv`.

**Sensing in free-run.** Combined attitude/rate (B) and orbit batching
(C) are off ([architecture](architecture.md) section 6).

**Access in free-run.** The scheduler asks for access at
`t_i + access_query_delay_s`; if the twin's written attitude does not
yet reach t_i, the bridge defers the answer
([messages](messages.md), `compute_access`)
and the scheduler asks again on its next tick. A target whose attitude
never reaches t_i by the end of the run is recorded `access_unverified`
and makes the record unreportable ([scheduler](../acav/scheduler.md)).

## 3. Rates and costs

On the deploy host (2 cores) a lockstep step costs about 60 ms in E6
with graphics off and an operator-started STK (E6_P9_9fed085, in
results/_archive/20261019_results_cleanup/pilots/: 60.0,
61.1 and 60.2 ms; sense 34.3-34.6 ms), so the 7200 steps of the 720 s
window take about 7 minutes of stepping, plus startup. Sensing
dominates. See
[09_live_runs_and_results.md](09_live_runs_and_results.md) for the
measured numbers and [analysis](../acav/analysis.md) for how they are
computed.

## 4. Why the step loop has its own thread

In lockstep the loop blocks on the barrier, and the ack that releases it
arrives through an executor callback. If the loop ran in an executor
thread it would wait for a callback that needs an executor thread: a
deadlock with a small pool, an intermittent one with a large pool. The
loop therefore runs on its own thread; executor threads handle ROS I/O
only, and STK calls go through the single COM worker thread
(`nodes/bridge_node.py` module docstring;
`test_time_sync.py::test_ack_from_another_thread_releases_a_blocked_wait`).

## 5. Out-of-order samples

STK 12.6 rejects (NACKs) a real-time attitude sample earlier than the
newest one it holds (`core/validation.py` `monotonicity_enforced`).
`core/actuation.py` keeps a local monotonicity guard anyway, and clears
an object's attitude history before its first command, since a reused
scenario keeps samples from the previous run.
