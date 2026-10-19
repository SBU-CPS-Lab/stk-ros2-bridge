# Live runs and results

The final runs first (facts about the records, no results), then the
earlier live evidence, kept as history. Superseded runs are in
`results/_archive/<group>` with their own index
(`results/_archive/README.md`); the engineering history behind them is
in [dev/](dev/).

## Final runs (2026-10-19)

Configuration and commands: [08_deploy_runbook.md](08_deploy_runbook.md)
section 8. Deploy host, STK 12.6 COM backend, operator-started STK,
automated twin (`--twin-launch matlab-batch`), each campaign from a
clean tree. Requested runs from each folder's `campaign.json`; records
are the `<run>.json` files, each read with `acav.run_record.read_record`.

| Experiment | Folder(s) under `results/` | Commit | Requested | Records (all reportable) |
|---|---|---|---:|---:|
| E6 | `E6_final_db7cfb6` | 6c32287 (db7cfb6) | 5 | 5 |
| E1 | `E1_final_db7cfb6` | 6c32287 (db7cfb6) | 24 | 12 |
| E3 | `E3_final_db7cfb6` | 6c32287 (db7cfb6) | 66 | 66 |
| E2 | `E2_final_seed{1..10}_91f36b9` (2 each), `E2_final_ref_91f36b9` (2) | 81bc202 (91f36b9) | 22 | 22 |

**E1.** 24 runs requested: 3 repeats x (the control, frame_pairing
guard on and guard ablated, five other types guard on). 12 have a record: the control,
angle_rate_units, distance_units and the frame_pairing read-back
ablation (`readbackEcfAttitudeQuaternions`), 3 each. The other 12 ended
without a record; each one's outcome, as `acav/run_outcome.py`
`classify` gives it from its `<run_id>.logs/summary.json`:

| Arm (3 repeats each, seeds 0-2) | Outcome | Evidence |
|---|---|---|
| quaternion_order, guard on | guard_fail_stop (`adcs:simulink_step:AttitudeMismatch`, quaternion residual 1.41421) | `E1-dynamics_aware-lockstep-seed{0,1,2}-mismatchQuaternionOrder.logs/summary.json` |
| quaternion_conjugate, guard on | guard_fail_stop (residual 1.41118) | `...-mismatchQuaternionConjugate.logs/summary.json` |
| frame_pairing, guard on | guard_fail_stop (residual 0.412327) | `...-readbackEcfAttitudeQuaternions-guardon.logs/summary.json` |
| time_offset, guard on | fail_stop (failed step `twin_acking`) | `...-mismatchTimeOffset.logs/summary.json` |

None was classified interrupted or infrastructure_failure.

**Signature checks.**
- All five E6_final records have the determinism signature of
  `results/E6_smoke/E6-smoke-1.json`.
- The commits between 6c32287 (db7cfb6) and 81bc202 (91f36b9) (2bea0b9..81bc202, old
  89c3c7e..91f36b9) change
  scenario building and orchestration only (`git diff --stat
  6c32287 81bc202`: build_cli, campaign_cli, campaign_runner's resume
  guard, e2_seeds, scenario_builder, the backends' `set_graphics_off`,
  tests, the runbook). Where records at both commits are comparable, the
  signatures are identical: both arms of the E2 reference
  (`E2_final_ref_91f36b9` against
  `results/_archive/20261019_results_cleanup/E2_superseded_db7cfb6/E2_final_ref_db7cfb6`)
  and both arms of seed 1 (`E2_final_seed1_*`, same pair of folders).

**E2 paired report and figures.** `results/E2_final_paired_91f36b9`:
`paired_report.json` and `paired_report.txt` (the paired report over
the ten seed folders, runbook section 8.4), and `figures/`, made by
`src/acav/scripts/make_e2_paper_figures.py` (three figures as PDF, SVG
and PNG, plus `E2_figure_data.json`). The reference arm is reported on
its own (exit 0).

**Moved on 2026-10-19:** the pilot and verification folders to
`results/_archive/20261019_results_cleanup/pilots/`, the E2 attempts on
db7cfb6 to `.../E2_superseded_db7cfb6/`, the operator's E2 attempt
logs (`E2_all_seeds_*.log`) to `.../E2_operator_attempts/`.

## Earlier live evidence (history)

All the folders named in this section are in
`results/_archive/20261019_results_cleanup/pilots/` (incidents: in
`results/_archive/incidents/`). Numbers were read from each run's record
and `summary.json` by `results/_analysis/p11/evidence_table.py`
(gitignored). All runs: deploy host (2 cores), STK 12.6 COM backend,
dynamics-aware scheduler, graphics off unless stated, every record
reportable. None of them is a final run.

### The reference signature

results/E6_smoke/E6-smoke-1.json (code c291dfa, manual twin) is the
lockstep determinism signature every later change is checked against:
5 of 7 targets attempted and captured (T5, T7 declined), 3 saturation
events, no SAFE. A lockstep run on the reference scenario with no fault
must reproduce it exactly.

### E6: loop cost

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

### E3: lockstep against free-run

Free-run, reference scenario. Lockstep reproduces the reference
signature (E6 above); free-run signatures differ from it, as expected.

| Folder | Code | rtf | Twin | Captured | Step (ms) | Sense (ms) | Start-gate wait (s) |
|---|---|---:|---|---|---:|---:|---:|
| E3_P6_rtf1_8345551 | 8345551 | 1.0 | manual | 5/5 | 139.9 | 124.1 | 57.3 |
| E3_P6_rtf05_8345551 | 8345551 | 0.5 | manual | 5/5 | 208.9 | 125.5 | 45.0 |
| E3_P9_rtf1_9fed085 | 9fed085 | 1.0 | automated | 5/5 | 118.6 | 105.7 | 119.8 |

One run per folder here; the final E3 (above) has repeats in every
cell (lockstep 3, free-run rtf 1.0 5, rtf 0.5 3).
Barrier wait is 0 in free-run (there is none). The start-gate wait is
mostly MATLAB starting (manual: the operator; automated: launch, build
and simulation start).

### E3: fault replay (seed 4)

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

### E2: scheduler comparison

| Folder | Code | Scenario | Scheduler | Attempted / targets | Captured | Saturation events |
|---|---|---|---|---|---|---:|
| E2_P10_seed1_3186642 | 3186642 | generated seed 1 (v2, 22 targets) | dynamics-aware | 18 / 22 | 18/18 | 12 |

A check that generator version 2 scenarios run end to end (T10, T13,
T16, T17 declined), not an E2 result: there is no kinematic arm. The E1
and E2 runs on 5723028 are in results/_archive/old_code_E1_E2; the
final runs (above) replace them.

### E1: convention mismatch

Before the final runs, the only E1 pair was on 5723028, in
results/_archive/old_code_E1_E2; the final E1 is above.

### Incidents kept as evidence

results/_archive/incidents: E2_P10_seed1_d97800c (the twin stopped at
its first step: a uint32/double class mismatch, fixed in 3186642) and
E5_P10_free1_3186642 (the bridge was still in convention validation
when the runner's 90 s bound expired, on a COM-started STK; the bound is
now 180 s and every startup phase is recorded).

### Live verification session (d0bf796, 2026-10-17)

Operator-started STK (pid 21316, `stk_process` step), automated twin.
Folders: LIVE_E6_d0bf796 (the E6 and E1 runs, in one folder by
mistake) and LIVE_E3_d0bf796, both now in
results/_archive/20261019_results_cleanup/pilots/. Read with
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
latest attempt; anything about repeat-to-repeat spread (one run per
cell). Since then the final E1 (above) classifies all three guard-on
quaternion_order runs as guard_fail_stop, and the final runs have
repeats per cell.
