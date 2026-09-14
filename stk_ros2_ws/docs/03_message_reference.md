# Message Reference

Full field-level documentation lives in the `.msg`/`.srv`/`.action`
files themselves (`src/stk_msgs/{msg,srv,action}/`) -- every field has an
inline comment explaining its meaning, units, and any STK-specific
caveat. This page is an index and a "what talks to what" map, not a
duplicate of that content; run `ros2 interface show` for the
authoritative field list.

```
ros2 interface show stk_msgs/msg/AttitudeCommand
ros2 interface show stk_msgs/srv/SetPropagator
```

## Topics

| Topic | Type | Direction | Rate group |
|---|---|---|---|
| `stk/<ns>/orbit` | `OrbitState` | bridge -> consumers | fast |
| `stk/<ns>/attitude` | `AttitudeState` | bridge -> consumers | fast |
| `stk/<ns>/angular_rate` | `AngularRate` | bridge -> consumers | fast |
| `stk/<ns>/lla` | `LLAState` | bridge -> consumers | medium |
| `stk/<ns>/eclipse` | `EclipseState` | bridge -> consumers | medium |
| `stk/<ns>/state` | `SatelliteState` | bridge -> consumers | fast (composite; requires orbit+attitude both present) |
| `stk/cmd/position` | `PositionCommand` | consumers -> bridge | n/a (event-driven) |
| `stk/cmd/attitude` | `AttitudeCommand` | consumers -> bridge | n/a |
| `stk/cmd/angular_velocity` | `AngularRate` | consumers -> bridge | n/a |
| `stk/lockstep/ack` | `std_msgs/String` | consumers -> bridge | n/a; `"<participant>:<step_index>"` |
| `stk/status` | `BridgeStatus` | bridge -> consumers | 1 Hz wall time |
| `stk/diagnostics` | `BackendDiagnostics` | bridge -> consumers | 1 Hz wall time |
| `stk/scenario_time` | `ScenarioTime` | bridge -> consumers | matches `/clock` |
| `/clock` | `rosgraph_msgs/Clock` | bridge -> everyone | matches step/tick rate |

`<ns>` is the lower-cased, sanitised STK object name -- see
`interfaces/ros_publishers.py::topic_namespace`. `SSS-1` becomes `sss_1`.

## Services (all under `stk/`)

`load_scenario` · `save_scenario` · `reset_scenario` · `create_object` ·
`delete_object` · `set_propagator` · `query_data_provider` ·
`compute_access` · `get_link_budget` · `set_animation_time` ·
`step_simulation` · `set_execution_mode` · `execute_connect`

`execute_connect` is the deliberate escape hatch -- see its `.srv`
comment and `interfaces/ros_services.py`'s module docstring on why it
logs at WARNING every call.

## Actions (all under `stk/`)

`run_scenario` · `execute_maneuver` (requires Astrogator; goal rejected
otherwise) · `compute_coverage` (goal rejected if any requested asset is
on the RealTime propagator, since STK Coverage analysis excludes those)

## Application-level messages

Two `stk_msgs` types exist for consumers to talk to EACH OTHER, not to
the bridge, and carry no `stk/` topic of their own -- the bridge never
publishes or subscribes to them:

| Type | Used on (by convention) | Purpose |
|---|---|---|
| `TargetSetpoint` | `/acav/setpoint` | A guidance target: inertial position + a have-target flag. The ACAV scheduler (`acav_scheduler`) publishes it; a closed-loop twin subscribes and turns it into a commanded attitude itself. |
| `TwinStatus` | `/acav/twin_status` | A twin's own mode name, boresight pointing error and reaction-wheel state (momentum, saturation, cumulative wheel energy), streamed back so a scheduler can score captures and report run-level quantities without re-deriving either from raw telemetry. |

They live in `stk_msgs` rather than a separate package because that is
where every other typed interface in this workspace already is, and
because `ros2genmsg` (see `docs/02_matlab_setup.md`) only needs to be
run against one message source, not several.

## Why absence, not defaults

Every telemetry topic follows one rule: **if the bridge has no data for
this epoch, nothing is published, ever** -- not a zeroed/identity
message, not a repeat of the last value. See
`interfaces/ros_publishers.py`'s module docstring for the full
rationale. When building a consumer, treat "no message yet this cycle"
as a real state your code must handle, not an edge case.
