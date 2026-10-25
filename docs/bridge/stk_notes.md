# Troubleshooting

Two parts. **Part 1** covers the case-study runs: what a failed step in
`<run>.logs/summary.json` means and what to do. **Part 2** lists the
STK 12.6 behaviours found while bringing the bridge up against a real
install; each fix is now in `core/`, and they are listed in case
another install surfaces something similar.

# Part 1: case-study runs

Every run writes `<run>.logs/summary.json` with one entry per step
(`step`, `ok`, `detail`) and the original exception of any step that raised under `errors` (type, message, traceback).
`run.log` has the same lines with times. Start there.

## `idle_check` failed

"Another run is active on ...": a bridge, scheduler or twin from an
earlier run is still publishing. Stop it first (and see "Leftover
MATLAB processes" below). Each wait of the runner runs on its own rclpy
context with no signal handlers, so a shutdown error cannot hide the real
one, which is recorded.

## `bridge_connected` failed

The detail names where the bridge's startup had got to, from
`<run>.logs/bridge_startup_phases.json` (each phase's duration, and the
current phase with its start time). Over 40 runs launch to CONNECTED
took 43.7-76.1 s; the runner waits up to 180 s
(`--bridge-connect-timeout-s`) and stops waiting at once if the bridge
process exits (`bridge_exit_code` in summary.json).

- "no running STK Desktop to attach to": start STK Desktop yourself and
  build the scenario in it. The bridge does not start STK by itself.
- A phase under `validation/...` that runs far longer than the others:
  look at `bridge.log` for that check. One free-run run on an STK that
  automation had started stayed in validation for more than 60 s.
- "no scenario is open" / "expected 'ACAV_SSS1'": build the scenario
  first ([running campaigns](../acav/running_campaigns.md)).

## `stk_process` failed

The bridge attached to an STK Desktop that was started through COM
(`AgUiApplication.exe ... -Embedding`, parent `svchost.exe`), not by
you, or the runner cannot tell which STK it used (none found, or more
than one running). E6 on such an STK stepped at 95.7 ms against about
60 ms (one run). Close it, start STK Desktop yourself, build the
scenario, and run again. `--allow-com-started-stk` runs anyway; the run
records it (`stk_process.started_by`), and `report` will not pool it
with operator-started runs. An STK is started through COM when a script
finds none running and was told it may start one (`--start-stk`,
`backend.com.start_stk`).

## `stk_graphics` failed

The run requires STK's graphics "off": no 2D or 3D Graphics window open
and STK minimised. Close every graphics window and minimise STK, or
pass `--graphics-mode on` deliberately (it changes the step time; such
runs are never pooled with "off" runs). `stk_graphics_end` checks the
same at the end, and "mixed" means it changed during the run.

## `twin_acking` failed

- Manual twin: "no matlab_twin ack seen before timeout": the model was
  not running, or its topics do not match. Rebuild it for this run's
  config and press Play.
- Automated twin: "MATLAB exited (code N) before the twin acked; see
  matlab.log". Read, in order: `matlab_session.json` (`exit`, `error`,
  and `simulink_diagnostics`), `simulink_diagnostics.txt`,
  `matlab.log`, `matlab_console.log`. A build error is in `error`; a
  Simulink run-time error (for example a block's output class) is in
  `simulink_diagnostics`. The runner currently waits out the full ack
  timeout (252.6 s) even after MATLAB has exited.

## `twin_identity` failed

The twin loaded another run's config, was built from other code, or
runs code that differs from this checkout. Rebuild the model for this
run (automated runs always do). After a `.msg` change, also regenerate
MATLAB's message types ([MATLAB client](../matlab/client.md)).

## The run stopped printing: QuickEdit

With QuickEdit on, selecting text in the console pauses every write to
it; one selection held a run for 11.5 min. The runner and the
campaign turn QuickEdit off while they run and restore it afterwards
(`acav/console.py`, `console_quickedit` in summary.json), and write the
console copy of the log from a background thread. Still: do not click
into the console during timing runs.

## Leftover MATLAB processes

A failed automated run can leave `matlab.exe` or `libmwros2server.exe`
running. Before each run the campaign runner kills only processes the
previous run recorded (its launcher's and MATLAB's PIDs) or that those
started, and appends what it killed to `twin_cleanup.jsonl` in the
campaign folder (`acav/campaign_runner.py` `_clean_up_twin`). It never
touches another MATLAB. A single run started by hand is not cleaned up
this way: check Task Manager before the next run.

## Missing body rate at t = 720 s

`no angular_rate for */Satellite/SSS-1 at t=720.000 s` (and the
segment query's NO DATA just before it) at the end of every run is
expected: 720 s is the scenario's stop time, and STK has no
`Attitude Segment Quaternions` data there
([architecture](architecture.md) section 11). The scheduler
has already written the record.

## A JSON argument arrives broken: PowerShell 5.1

Windows PowerShell 5.1 strips embedded double quotes from arguments
passed to native programs, so `--fault-json '{"wheel": ...}'` typed in
PowerShell reaches Python without its quotes. Use a run-config file
(`--run-config`), a campaign placeholder (`{fault_json}`; the campaign
substitutes it in Python and passes an argument list), or a Python
launcher. Literal `{placeholder}` arguments must stay in single quotes
in PowerShell, or the braces are parsed as a script block.

## `report` refuses to pool

`run_acav_campaign report` refuses a directory whose records differ in
graphics mode, twin launch mode, how STK was started, scenario, or that
holds a diagnostic run; the message names each group and its runs. Move
each group to its own directory ([experiment harness](../acav/experiment_harness.md)).

# Part 2: STK 12.6 findings from bring-up

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
call; see [frames and units](frames_and_units.md).

## `quaternion_ordering` validation fails with a large error (tens of degrees)

Almost certainly a **frame-pairing** issue, not a genuine element-order
bug. `AddAttitude CBFQuat` must be read back via `ECF Attitude
Quaternions`; `AddAttitude Quat` via `Attitude Quaternions`. The
validation check tries both pairings automatically and reports which
one round-trips; if BOTH fail, compare the commanded vs. read-back
component magnitudes (sorted) -- matching magnitudes point to a real
ordering fault, differing magnitudes point to a frame fault. See
[frames and units](frames_and_units.md) for the full story.

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
**time before frame**, not frame before time. Both backends follow this order; if extending either backend, check this order against a live
`ExecuteConnect` call before trusting written documentation examples.

## MATLAB: custom message errors that look like network problems

Almost always a MATLAB-release / ROS-2-distro mismatch (see
[MATLAB client](../matlab/client.md)'s table). Confirm your MATLAB
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
