# Troubleshooting

Findings below came from running `check_environment.py` against a real
STK 12.6 install during this project's own development, not from
documentation alone -- each is a case where an initial assumption was
wrong and the fix is now baked into `core/`. Listed here in case a
*different* STK 12.6 install (different patch level, different
licensed modules) surfaces something similar.

## "Data provider ... returned NO DATA (0 columns, 0 rows)"

**Most common cause:** the object has no ephemeris. A satellite created
via `Children.New` has a propagator and default elements but generates
**no ephemeris until `Propagate()` runs** -- every data provider on it
returns zero rows silently until then. `check_environment.py` and
`ScenarioManager.ensure_propagated()` both call `Propagate()`
automatically; if you're driving the COM backend directly outside the
bridge, call `backend.propagate(object_path)` first.

**Second cause:** the object is on the **RealTime propagator**, which
**discards its analytic ephemeris** on switch. It then has state only
within Look-Ahead/Look-Behind of samples actually injected. Querying
before any sample -- or a stale scenario time relative to the newest
sample -- legitimately returns nothing. `core/sensing.py` treats this
case as *staleness* (returns `None`), not an error, specifically for
real-time objects; see its module docstring.

## "unit preferences NOT applied: AngleRate=rad/sec"

Expected on STK 12.6 -- confirmed, not a bug in your install.
`SetCurrentUnit("AngleRate", "rad/sec")` is rejected. The bridge
measures the actual unit at startup instead of trusting the preference
call; see [04_frames_and_units.md](04_frames_and_units.md).

## `quaternion_ordering` validation fails with a large error (tens of degrees)

Almost certainly a **frame-pairing** issue, not a genuine element-order
bug. `AddAttitude CBFQuat` must be read back via `ECF Attitude
Quaternions`; `AddAttitude Quat` via `Attitude Quaternions`. The
validation check tries both pairings automatically and reports which
one round-trips; if BOTH fail, compare the commanded vs. read-back
component magnitudes (sorted) -- matching magnitudes point to a real
ordering fault, differing magnitudes point to a frame fault. See
[04_frames_and_units.md](04_frames_and_units.md) for the full story.

## "could not set Look-Ahead/Look-Behind ... Candidate properties: ['Duration', ...]"

Confirmed on a live STK 12.6 install: neither `LookAheadTime` nor
`LookBehindTime` exist as settable properties on the RealTime
propagator's Object Model surface; `Duration` does (tried automatically
by `com_backend.py`). No equivalent look-behind property was found on
this install -- if your install differs, the logged candidate list is
exactly what to check.

## `SetPropagator ... RealTime` NACKs over Connect

Confirmed: the Connect-command form of switching to the RealTime
propagator fails on STK 12.6 ("Command has failed."), while the
Object Model's `SetPropagatorType(AgEVePropagatorType.ePropagatorRealtime)`
succeeds. `com_backend.py` uses the Object Model path for this specific
operation; note the enum spelling's lowercase "t" in "Realtime"
(`ePropagatorRealtime`), which differs from "RealTime" as spelled
elsewhere in STK's own documentation.

## `SetPosition` argument order

The Connect command is `SetPosition <path> <time> <frame> <x y z>` --
**time before frame**, not frame before time. Both backends were fixed
to match; if extending either backend, check this order against a live
`ExecuteConnect` call before trusting written documentation examples.

## MATLAB: custom message errors that look like network problems

Almost always a MATLAB-release / ROS-2-distro mismatch (see
[02_matlab_setup.md](02_matlab_setup.md)'s table). Confirm your MATLAB
release supports Humble before debugging anything else.

## MATLAB: `ros2genmsg` fails deep inside compilation

Missing MSVC. Install Visual Studio Build Tools with "Desktop
development with C++" -- the error message does not name this
prerequisite directly.

## Lockstep barrier timing out every step

1. Check the participant name matches exactly: `barrier_participants`
   in `bridge_params.yaml` vs. the string passed to
   `CommandPublisher`/`acknowledgeStep`.
2. Check the ack is published for the **current** step index, not a
   stale one -- an ack for the wrong step is logged and ignored, not
   silently accepted (see `core/time_sync.py::LockstepBarrier.acknowledge`).
3. Confirm the ack is published from a **reentrant** callback group if
   you're writing a custom node, not one that's blocked behind other
   work -- see `nodes/bridge_node.py`'s concurrency-model docstring.

## Cross-machine (T2/T3): DDS discovery silent failure

Windows Firewall blocks inbound UDP multicast for RTPS discovery by
default. Confirm `ROS_DOMAIN_ID` matches on both machines, and add a
firewall rule for the STK host if `ros2 topic list` from the other
machine shows nothing. For the Connect backend specifically, also
confirm TCP port 5001 (or your configured port) is reachable and that
Connect is enabled in STK's Edit > Preferences > Connect dialog.
