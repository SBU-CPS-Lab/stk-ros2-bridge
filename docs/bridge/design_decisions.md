# Design decisions

The choices that shape the bridge, with the reason for each. The
[architecture](architecture.md) describes what exists; this page says
why it is that way.

## The bridge is the only STK client and the clock authority

Exactly one process talks to STK. Every command and read goes through
the bridge's COM worker thread, so writes and reads are ordered, and
STK's single-threaded COM apartment is never shared. The same process
owns the simulation clock: it publishes `/clock` and `ScenarioTime`, and
no other node advances time. Consumers therefore never need to agree on
a time source, only to follow the one stamp.

## Backends behind one interface, with a layering rule

`StkBackend` (`backends/base.py`) is the only surface the rest of the
bridge sees. Code in `core/`, `interfaces/` and `nodes/` never imports
`agi.stk12`, `win32com`, `pythoncom` or Connect socket code. A mock
backend implements the same interface, so almost the whole bridge, and
every campaign, is testable on a machine with no STK. The rule is
enforced by an AST check ([extending backends](extending_backends.md)).

## SI units, `(x, y, z, w)` quaternions, forced at the STK end

Backends set STK's unit preferences at `connect()` and verify them. The
wire format is metres, seconds, radians, and quaternions in
`(x, y, z, w)` order. Conversions happen once, at the edge, and are
validated at startup rather than trusted
([frames and units](frames_and_units.md)).

## Validate conventions at startup, and refuse to go on

STK has several providers and command forms that look interchangeable
and are not (for example commanding `CBFQuat` but reading an inertial
provider). Startup validation drives known inputs through the real STK
and checks the results; a failing check stops the bridge. A sign or
frame error does not crash a co-simulation, it produces plausible wrong
physics, so the check has to run before any result can exist.

## Absence, not defaults

If there is no data for a channel at an epoch, nothing is published for
it: never a zero, an identity quaternion or a repeat of the last value.
Consumers treat "no message this step" as a real state
([messages](messages.md)).

## Two execution modes, with different promises

**Lockstep** is deterministic: a barrier per 0.1 s step means every
participant has acked step k before the bridge senses k+1, so the same
inputs give the same trace. **Free-run** advances with the wall clock
times a real-time factor, so it measures what asynchronous coupling
costs but promises no determinism. Both are first-class, and a record
says which mode the bridge actually ran in
([time synchronization](time_synchronization.md)).

## Free-run is paced from the cycle start

Cycles run on a fixed-rate schedule anchored at the first cycle's start,
not "sleep for a period after the work". An overrunning cycle is
followed immediately and counted, with no catch-up burst. Within a cycle
the clock is published after sensing so that telemetry and clock stamps
match when the twin reads them.

## The twin carries its own body rate

STK derives a rate numerically from the attitude samples it was given.
Across a missing sample that derivative is enormous. The twin therefore
integrates and publishes its own rate, and uses STK's rate only to
initialise.

## Sensing optimisations are opt-in, proven at runtime, and recorded

Combined attitude/rate reads and orbit batching cut COM round trips, but
each is used only after it has been shown to agree with the plain read
on this install, falls back when the backend's `revision` changes, and
is recorded per run. Free-run and the E1 convention study turn them off.

## A tested Python reference first, then MATLAB

The ADCS twin exists twice: a Python reference
(`acav/twin_reference.py`) and the MATLAB/Simulink implementation. The
semantics are fixed once in Python, where they are unit-tested, exported
as fixtures, and the MATLAB code is tested against them
([ADCS twin](../matlab/adcs_twin.md)).

## Provenance travels with every result

A `RunRecord` stores code version, configuration and scenario digests,
twin identity, STK and MATLAB versions, and run conditions (execution
mode, real-time factor, graphics mode, how STK was started). `report`
refuses to pool records whose conditions differ, and records from the
mock backend are never reportable. Scenario files are named by their
SHA-256 in records, so published scenario YAMLs must not change
([experiment harness](../acav/experiment_harness.md)).
