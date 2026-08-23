# Time Synchronization

## The bridge is the clock authority

In `lockstep` and `paced` modes, the bridge node publishes `/clock` and
every other node -- including MATLAB/Simulink -- runs with
`use_sim_time:=true`. This is what makes a run reproducible: replace
"wall clock" with "whatever the bridge says," and every node agrees.

## Execution modes

| Mode | STK clock | `/clock` behaviour | Use |
|---|---|---|---|
| `free_run` | Real-time (STK animates on wall clock) | Mirrors wall time | Visualization, demos |
| `paced` | Real-time-factor-scaled | Scaled wall time | Human review at non-1x speed |
| `lockstep` | Stepped by the bridge | Advances only on step | **Default for control work** |
| `external` | Follows another authority | Adopted, not published | Federating with a third simulator |

Configured via `bridge_params.yaml`'s `time:` block, or at runtime via
the `SetExecutionMode` service.

## Lockstep, in detail

Each iteration of the bridge's step loop (`nodes/bridge_node.py`'s
`_lockstep_iteration`):

1. **Apply** commands accumulated in the inbox since the last step.
2. **Advance** simulation time by `step_s` and push it into STK.
3. **Publish** the new state (telemetry, `/clock`, `scenario_time`) --
   participants have the new state in hand *before* being asked to
   acknowledge it.
4. **Wait** for every declared `barrier_participants` entry to
   acknowledge this step index, or apply the configured timeout policy.

### Acknowledging a step from MATLAB

```matlab
client.Commands.acknowledgeStep("matlab_twin", stepIndex);
```

Publish this **after** any commands your control law computed for this
step, never before -- see `CommandPublisher.acknowledgeStep`'s help text
and `interfaces/ros_subscribers.py`'s ordering note. Acking before your
commands land lets the bridge advance past a step whose control input
never arrived.

### Barrier timeout policy

| Policy | Behaviour |
|---|---|
| `abort` | Raises `LockstepBarrierTimeoutError`. **Default.** Fails loudly and immediately. |
| `continue` | Advances anyway, logged at ERROR: *"this run is no longer deterministic."* |
| `wait` | Blocks indefinitely. Pair with a health watchdog or it hangs forever on a dead participant. |

### Why a dedicated thread, not an executor timer

The step loop **blocks** on the barrier wait, and the acknowledgement
that releases it arrives as a ROS message via an executor callback. If
the loop itself occupied an executor thread, it would be waiting on a
callback that needs an executor thread to run -- a deadlock that is
either permanent (small thread pool) or intermittent under load (large
pool). The step loop therefore runs on its own dedicated thread; the
executor's threads handle ROS I/O only. See
`nodes/bridge_node.py`'s module docstring, and
`test/test_time_sync.py::test_ack_from_another_thread_releases_a_blocked_wait`
for the automated regression test of this property.

## Confirmed STK 12.6 behaviour: out-of-order samples are NACKed

Originally assumed STK would silently drop an out-of-order real-time
sample. Confirmed otherwise against a live install: STK 12.6 **NACKs**
it explicitly (`core/validation.py`'s `monotonicity_enforced` check).
`core/actuation.py`'s local monotonicity guard is kept anyway --
catching the violation before the round trip saves the COM call, gives
a counter rather than requiring a log grep, and stays correct
regardless of whether a given backend's behaviour matches this finding.

## Rate expectations

Budget **1-20 Hz** for a closed loop over COM on this project's
reference hardware -- a general expectation, not a guarantee for your
scenario. Run `scripts/benchmark_rates.py` against your actual STK
install and object count before committing to a `step_s` value; object
count, data-provider complexity, and topology (T1 vs T2/T3) all change
the achievable rate materially.
