# Current evidence

The live runs that remain at the top of `results/` after the
2026-10-16 reorganisation, one table per experiment. Every number below
is read from the run's record (`<run>.json`) and `summary.json` by
`results/_analysis/p11/evidence_table.py` (gitignored; rerun it to
reproduce this page). Superseded runs are in `results/_archive/<group>`
with their own index (`results/_archive/README.md`); the engineering
history behind them is in [dev/](dev/).

None of these are final paper runs: the final runs are made on the final
commit, with repeat counts still to be decided
([dev/phase3_backlog.md](dev/phase3_backlog.md)). All runs: deploy host
(2 cores), STK 12.6 COM backend, dynamics-aware scheduler, graphics off
unless stated, every record reportable.

## The reference signature

results/E6_smoke/E6-smoke-1.json (code c291dfa, manual twin) is the
lockstep determinism signature every later change is checked against:
5 of 7 targets attempted and captured (T5, T7 declined), 3 saturation
events, no SAFE. A lockstep run on the reference scenario with no fault
must reproduce it exactly.

## E6: loop cost

Lockstep, reference scenario, automated twin. Per-step means from the
bridge (BridgeStatus, weighted over the run), in ms:

| Folder / run | Code | Step | Sense | Actuate | Barrier wait | Signature = E6-smoke-1 |
|---|---|---:|---:|---:|---:|---|
| E6_P9_9fed085 / seed0 | 9fed085 | 60.0 | 34.6 | 8.2 | 14.1 | yes |
| E6_P9_9fed085 / seed1 | 9fed085 | 61.1 | 34.6 | 8.5 | 14.9 | yes |
| E6_P9_9fed085 / seed2 | 9fed085 | 60.2 | 34.3 | 8.2 | 14.4 | yes |
| E6_P10_3186642 / E6-P10-1 | 3186642 | 95.7 | 57.3 | 16.6 | 18.5 | yes |
| E6_smoke / E6-smoke-1 | c291dfa | 182.6 | 151.4 | 15.5 | 12.1 | (reference) |

- E6-P10-1 ran on an STK Desktop started through COM by the scenario
  build script (`-Embedding`), not by the operator. Same signature,
  1.6x the step time. One run, so a correlation; since 24b5eeb such runs
  are refused, and it must not be pooled with the P9 runs.
- E6-smoke-1 predates the sensing optimisations (B)/(C) and the
  graphics-off requirement (its record has no graphics mode); it is the
  signature reference, not a timing reference.
- E6_P9 seed1's wall time (22.0 min) includes an 11.5 min console stall
  (QuickEdit); its per-step means are not affected.

## E3: lockstep against free-run

Free-run, reference scenario. Lockstep reproduces the reference
signature (E6 above); free-run signatures differ from it, as expected.

| Folder | Code | rtf | Twin | Captured | Step (ms) | Sense (ms) | Start-gate wait (s) |
|---|---|---:|---|---|---:|---:|---:|
| E3_P6_rtf1_8345551 | 8345551 | 1.0 | manual | 5/5 | 139.9 | 124.1 | 57.3 |
| E3_P6_rtf05_8345551 | 8345551 | 0.5 | manual | 5/5 | 208.9 | 125.5 | 45.0 |
| E3_P9_rtf1_9fed085 | 9fed085 | 1.0 | automated | 5/5 | 118.6 | 105.7 | 119.8 |

One run per folder; the spread across repeats is not measured yet.
Barrier wait is 0 in free-run (there is none). The start-gate wait is
mostly MATLAB starting (manual: the operator; automated: launch, build
and simulation start).

## E3: fault replay (seed 4)

Seeded saturation fault on wheel x at t = 218.375 s (`plan_fault(4)`),
reference scenario, automated twin:

| Folder | Code | Mode | Captured | Outcomes T4, T6 | SAFE | Saturation events |
|---|---|---|---|---|---|---:|
| E5_P9d_lock1 | 2ca0ba4 | lockstep | 3/5 | safe_mode, safe_mode | yes | 1 |
| E5_P9d_lock2 | 2ca0ba4 | lockstep | 3/5 | safe_mode, safe_mode | yes | 1 |
| E5_P9d_free1 | 2ca0ba4 | free-run, rtf 1.0 | 3/5 | safe_mode, safe_mode | yes | 0 |
| E5_P10_free2_3186642 | 3186642 | free-run, rtf 1.0 | 3/5 | safe_mode, safe_mode | yes | 1 |

- **Replay reproduces in lockstep:** E5_P9d_lock1 and E5_P9d_lock2 have
  the same determinism signature.
- The free-run runs differ from lockstep and from each other, as
  free-run does.
- **P10e in free-run:** on 2ca0ba4 the free-run run counted 0
  saturation events where lockstep counted 1; the event (the T3 blip)
  fell between two published statuses. With the twin's own counter
  (d97800c, fixed in 3186642) the free-run re-run counts 1; its rise is
  at t = 127.235 s. The operator reports starting STK for E5_P10_free2;
  the run itself does not record which STK process it used (recording
  started with 24b5eeb), so that is not verified from the record.

## E2: scheduler comparison

| Folder | Code | Scenario | Scheduler | Attempted / targets | Captured | Saturation events |
|---|---|---|---|---|---|---:|
| E2_P10_seed1_3186642 | 3186642 | generated seed 1 (v2, 22 targets) | dynamics-aware | 18 / 22 | 18/18 | 12 |

A check that generator version 2 scenarios run end to end (T10, T13,
T16, T17 declined), not an E2 result: there is no kinematic arm. The E1
and E2 runs on 5723028 are in results/_archive/old_code_E1_E2 and are
to be re-run on the final commit.

## E1: convention mismatch

No E1 run on current code; the earlier pair is in
results/_archive/old_code_E1_E2 (5723028).

## Incidents kept as evidence

results/_archive/incidents: E2_P10_seed1_d97800c (the twin stopped at
its first step: a uint32/double class mismatch, fixed in 3186642) and
E5_P10_free1_3186642 (the bridge was still in convention validation
when the runner's 90 s bound expired, on a COM-started STK; the bound is
now 180 s and every startup phase is recorded).

## Live verification session (d0bf796, 2026-10-17)

Operator-started STK (pid 21316, `stk_process` step), automated twin.
Folders: results\LIVE_E6_d0bf796 (the E6 and E1 runs, in one folder by
mistake) and results\LIVE_E3_d0bf796. Read with
`results/_analysis/live_check/summarize_live_check.py`. Earlier attempts
of each run are kept as `*.superseded-*`.

| Step | Latest attempt | Earlier attempts |
|---|---|---|
| a. E6 x1 | record; signature equals E6-smoke-1 | bridge exited: no STK running; bridge exited: no scenario open; MATLAB exited (code 1) when its stop-file and log paths were relative (matlab_console.log:23) |
| b. E1 control | record; 5/5 captured; signature equals E6-smoke-1 | -- |
| b. quaternion_order, guard on | infrastructure failure, not a detection: the twin's ROS 2 node did not start ("Server failed to start. Details: Transport stopped.") | superseded-20261017T191957: the twin's own guard stop, adcs:simulink_step:AttitudeMismatch (quaternion residual 1.41421); superseded-20261017T183741: the same infrastructure failure |
| b. time_offset, guard on | ended by itself at the ack timeout; the report's label: confirmed (MATLAB alive and running, 0 twin statuses and commands, 127 bridge attitude samples; control acked in 112.1 s) | interrupted (KeyboardInterrupt): no result; MATLAB exited (code 1) |
| c. E3 free-run rtf 0.5, fault seed 4 | record; 27.3 min wall; 3/5 captured; 1 saturation event; SAFE (T4, T6); start gate 123.7 s; twin ready calls 71.4 per wall second | -- |

What this does not show: the cause of the twin's "Server failed to
start" (not established); a guard-on quaternion_order result in its
latest attempt (to be retried); anything about repeat-to-repeat spread
(one run per cell).
