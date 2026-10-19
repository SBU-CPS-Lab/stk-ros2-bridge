# Message reference

Field-level documentation is in the `.msg`, `.srv` and `.action` files
themselves (`src/stk_msgs/{msg,srv,action}/`); every field has a comment
with its meaning and unit. This page is the index: which topics exist,
who publishes them, and the conventions a consumer has to know.

```
ros2 interface show stk_msgs/msg/TwinStatus
```

## Bridge topics

`<prefix>` is `stk` by default (`topic_prefix`); `<ns>` is the lower-case,
sanitised STK object name (`interfaces/ros_publishers.py`
`topic_namespace`): `SSS-1` becomes `sss_1`.

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `/<prefix>/<ns>/orbit` | `OrbitState` | bridge -> | Position/velocity, SI units. |
| `/<prefix>/<ns>/attitude` | `AttitudeState` | bridge -> | Through the provider validation paired with the command form. |
| `/<prefix>/<ns>/angular_rate` | `AngularRate` | bridge -> | rad/s. |
| `/<prefix>/<ns>/lla` | `LLAState` | bridge -> | Medium rate group. |
| `/<prefix>/<ns>/eclipse` | `EclipseState` | bridge -> | Medium rate group. |
| `/<prefix>/<ns>/state` | `SatelliteState` | bridge -> | Composite; only when orbit and attitude are both present. |
| `/<prefix>/cmd/attitude` | `AttitudeCommand` | -> bridge | The twin's commanded attitude, stamped for the next step. |
| `/<prefix>/cmd/position` | `PositionCommand` | -> bridge | Real-time propagator `SetPosition`; not used by ACAV. |
| `/<prefix>/cmd/angular_velocity` | `AngularRate` | -> bridge | Not used by ACAV. |
| `/<prefix>/lockstep/ack` | `std_msgs/String` | -> bridge | `"<participant>:<step_index>"`, sent after that step's command. |
| `/<prefix>/status` | `BridgeStatus` | bridge -> | 1 Hz wall time; RELIABLE + TRANSIENT_LOCAL. STARTING at each startup phase, then CONNECTED/DEGRADED/... with E6's per-second mean sense, actuate and barrier-wait times. |
| `/<prefix>/diagnostics` | `BackendDiagnostics` | bridge -> | 1 Hz; backend counters and the validation report. |
| `/<prefix>/scenario_time` | `ScenarioTime` | bridge -> | Simulation time, execution mode, real-time factor, step index, paused (the free-run start gate). |
| `/clock` | `rosgraph_msgs/Clock` | bridge -> | Same time as ScenarioTime. |

QoS profiles are in `interfaces/qos.py`. A watcher of `/<prefix>/status`
must request TRANSIENT_LOCAL durability, or it never receives anything
(`deploy_run_cli.py` `_wait_for_bridge_connected_ros`).

## Services and actions (under `/<prefix>/`)

Services: `load_scenario`, `save_scenario`, `reset_scenario`,
`create_object`, `delete_object`, `set_propagator`,
`query_data_provider`, `compute_access`, `get_link_budget`,
`set_animation_time`, `step_simulation`, `set_execution_mode`,
`execute_connect` (raw Connect passthrough, logged at WARNING on every
call). Actions: `run_scenario`, `execute_maneuver` (needs Astrogator),
`compute_coverage`.

**`compute_access` deferral.** For a satellite (or one of its sensors)
whose attitude this bridge writes, a request whose `stop_time` lies
beyond the newest written attitude is answered with `success: false`
and a `message` that starts with

```
attitude not yet written through the stop time
```

(`core/actuation.py` `ATTITUDE_NOT_COVERED`, followed by how far short
the written attitude is). The caller should ask again later; the ACAV
scheduler matches this prefix (`acav/scheduler_node.py`). Objects whose
attitude the bridge never wrote are not deferred.

## Application topics (ACAV)

These carry messages between the case study's own nodes; the bridge
neither publishes nor subscribes to them.

| Topic | Type | Publisher -> subscriber |
|---|---|---|
| `/acav/setpoint` | `stk_msgs/TargetSetpoint` | scheduler -> twin: the target's inertial position and `has_target`. The twin turns it into an attitude. |
| `/acav/twin_status` | `stk_msgs/TwinStatus` | twin -> scheduler: see below. |
| `/acav/twin_identity` | `std_msgs/String` | twin -> run orchestrator: `v3 <cfg> <build> <runtime> <polling>Hz`, about once per wall second once ready (`acav/twin_identity.py`). |

### `TwinStatus` fields

| Field | Unit | Meaning |
|---|---|---|
| `header` | | Stamped with the simulation time of the telemetry the status was computed from, never the wall clock (`build_acav_twin.m` `addStamp`); the scheduler takes it as the sample's time. |
| `mode` | | `NADIR`, `SLEW`, `SETTLE`, `IMAGE` or `SAFE`. |
| `boresight_error_rad` | rad | The twin's own angle between its boresight and the line of sight to the current target, at the last step. |
| `wheel_momentum_nms` | N m s | Wheel momentum vector (physical unit, not a fraction of capacity). |
| `wheels_saturated` | | True at a step where a wheel clamped in torque or momentum. |
| `wheel_energy_j` | J | Cumulative integral of the absolute wheel power since the run began. |
| `saturation_events` | count (uint32) | Cumulative number of steps at which `wheels_saturated` rose from false to true, counted by the twin at **every** integration step, sub-steps included. |
| `max_boresight_error_since_last_rad` | rad | The largest `boresight_error_rad` over every step since the previous TwinStatus, this one included. |

The last two fields were added in d97800c (P10e). In free-run the twin
integrates several 0.1 s steps per published status, so an event between
two publishes was invisible: an E3 free-run fault run counted 0
saturation events where lockstep counted 1. The scheduler now takes the
larger of its own edge count and `saturation_events`, and judges the
capture hold window on `max_boresight_error_since_last_rad`
(`acav/capture_tracking.py`, `acav/metrics.py`). In lockstep every step
is published, so both fields equal what the other fields already show,
and lockstep results are unchanged (golden trace and the E6 signature).
The free-run fault re-run on 3186642 counted the T3 blip at
t = 127.235 s through this field
(results/_archive/20261019_results_cleanup/pilots/E5_P10_free2_3186642).

## Absence, not defaults

If the bridge has no data for a channel at an epoch, nothing is
published for it: never a zero, an identity quaternion or a repeat of
the last value (`interfaces/ros_publishers.py` module docstring). A
consumer must treat "no message this step" as a real state; at
t = 720 s, the scenario's stop time, the body rate is such a case
([00_architecture.md](00_architecture.md) section 11).
