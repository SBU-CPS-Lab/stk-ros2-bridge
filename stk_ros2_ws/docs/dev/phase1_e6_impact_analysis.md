# Phase 1 — E6 impact analysis of combined attitude/rate reads (B) and HPOP orbit batching (C)

Read-only analysis made on branch `perf/e6-throughput` before (B) or (C)
was implemented. Code references are to commit `80ee188`, whose bridge,
MATLAB, message, ACAV configuration and scheduler code is identical to
`main@925b484` (`git diff --quiet main HEAD -- stk_ros2_ws/src/stk_bridge
stk_ros2_ws/src/stk_matlab stk_ros2_ws/src/stk_msgs stk_ros2_ws/src/acav/config
stk_ros2_ws/src/acav/acav/scheduler_node.py`).

Conventions in this document:

* **[USER-MEASURED]** — measured by the user in run E6-smoke-1 on commit
  `c291dfa` (bridge.log COM profiles, steady state). Not re-measured here.
* **[HYPOTHESIS]** — hypothesis from code order, not measured.
* **[LIVE CHECK]** — cannot be determined from the repository; needs a
  live STK check (see "Live checks pending").

## Measured input [USER-MEASURED]

* Wall time 26m47s; lockstep loop 21m33s; waiting for the MATLAB twin 2m53s.
* Mean per step 182.6 ms = sense 151.4 + actuate 15.5 + barrier wait 12.1 + other 3.7.
* Each provider query ≈ 25 ms (execute ≈ 10.6 + extract ≈ 14.5); the COM
  DataSets call ≈ 11 ms dominates extraction; Python conversion ≈ 0.1 ms.
* Cartesian Position execute ≈ 27.9 ms versus ≈ 10.7 ms for the other three.
* The four fast queries sum to ≈ 119 ms against 151.4 ms sense (≈ 32 ms unexplained).
* Per read (60 s windows): orbit ≈ 68–73 ms, attitude ≈ 24–26 ms, angular_rate ≈ 24–25 ms.
* At t = 720.000 s 'Attitude Segment Quaternions' returned NO DATA; no
  angular_rate was published for the final step.
* First sense at t = 0.1 s took 716.9 ms (orbit 415 ms).

## §1 Per-experiment impact matrix

Every ACAV run reads the same fast-channel providers each step
(`sensing.py:510`, `:525-527`): `Cartesian Position/ICRF` then
`Cartesian Velocity/ICRF` (`:357`, `:368`), the validated or forced
attitude provider (`:396`), and `Attitude Segment Quaternions` for rate
(`:425`, `:240-248`). Whether (B)/(C) reach an experiment depends only
on the bridge configuration passed to `run_acav_run.py`, which copies
`--bridge-config` per run (`deploy_run_cli.py:170-199`); every E1–E6
runbook command passes `bridge_params_acav.yaml`.

The matrix below is the analysis as made; the E1 rows are superseded by
decision Q3 (E1 excluded from both).

| Exp | (B) active | (C) active | Path changes | Expected output effect | Risk |
|---|---|---|---|---|---|
| E1 control | yes (before Q3) | yes (before Q3) | yes | timing; numerical only if alternate reads differ | medium |
| E1 treatment | no — forced provider keeps original path (`sensing.py:200-223`) | yes (before Q3) | (C) only | timing; orbit values | medium |
| E2 (both) | yes | yes | yes | as E1 control | medium |
| E3 lockstep | yes | yes | yes | as E1 control | medium |
| E3 free-run | n/a — blocked (`deploy_run_cli.py:170-199` never applies the mode; `simulink_step.m:47-50`) | must be OFF: free-run epochs are wall-derived (`bridge_node.py:499`) | — | — | n/a |
| E4 | yes | yes | yes | as E1 control | medium |
| E5 | yes | yes | yes; fault is twin-only (`+adcs/config.m` applyRunConfig, `wheel_fault.m`) | as E1 control | medium |
| E5 replay | yes | yes | — | replay against an original from the other path tests the code change, not replay | high if mixed |
| E6 / smoke | yes | yes | yes | timing changes by design; numerical as E1 control | medium |

## §2 Attitude data path in lockstep (central question for B)

**Writer.** The twin stamps the attitude command at t + step
(`build_acav_twin.m:273-283`). The bridge waits for it (`bridge_node.py:369-378`),
applies it on the COM worker (`:380-381`, `:522`), then advances (`:392`).
`actuation.py:265-267` → `com_backend.py:1231-1240` sends
`AddAttitude <path> Quat "<UTCG ms>" qx qy qz qw` (inertial,
`actuation.default_attitude_inertial: true`). On the first command,
`SetAttitude Standard` and `SetAttitude ClearData` run once
(`actuation.py:182-193`, `com_backend.py:1221-1229`). No interpolation or
segment options are set anywhere in the repository. Each step therefore
writes the sample for t_{k+1} before the bridge senses at t_{k+1}; the read
epoch equals a written sample time except at t = 0.1 s.

**Validated read-back today.** Once at startup, on a temporary satellite
(`validation.py:210-254`): `AddAttitude Quat` then `Attitude Quaternions`
(`:523-569`), < 1° required (`:581`). The rate unit is measured with
`AngVels` via `Body Axes Angular Velocity` or `Attitude Segment Quaternions`
(`:639-698`). No code compares the segment provider's quaternion columns
with anything.

| Property | Attitude Quaternions | Attitude Segment Quaternions | Evidence |
|---|---|---|---|
| Frame | central-body inertial | central-body inertial | docstrings only (`data_providers.py:133-138`, `:176-181`) [LIVE CHECK] |
| Order / scalar | Q1..Q3 vector, Q4 scalar | q1..q3, q4 | same mapping (`sensing.py:401-406`, `conversions.py:176-186`); validated live only for Attitude Quaternions (`validation.py:553-558`) [LIVE CHECK] |
| Direction (inertial→body) | C3 convention | not established | [LIVE CHECK] |
| Time tag | request epoch `when`, not the Time column | same | `sensing.py:325-327`, `:417-418` |
| Units | dimensionless | dimensionless; rates scaled by measured factor | `sensing.py:430-439`, `validation.py:704-713` |

Possible differences: exact-sample interpolation, segment semantics for
real-time attitude, and the basic attitude at t = 0.1 s (which initialises
the twin, `simulink_step.m:19-31`, `:60-66`) are [LIVE CHECK]. They already
differ at the terminal epoch (§7).

Sign: every consumer is sign-invariant — `conversions.py:172` (abs dot,
used at `bridge_node.py:610-611`), `simulink_step.m:53`, `quat.m:59-64`,
`tracking_torque.m:6-8`. No consumer differences consecutive quaternions;
logged `attitude.txt` captures would show sign flips, so offline analysis
must be sign-aware.

## §3 Consumers of the attitude value

| Consumer | Where | Effect of (B) if providers agree up to sign |
|---|---|---|
| ROS AttitudeState | `bridge_node.py:745`; frame from `sensing.py:410-411` | sign only; frame_id stays icrf |
| tf | `bridge_node.py:746-747` | sign only |
| Twin step 1 | `build_acav_twin.m:587-588` → `simulink_step.m` | initial attitude; numerical if not identical |
| Twin steps ≥ 2 | `simulink_step.m:52-63` (guard only, then own endpoint) | none unless residual > 5e-5 (loud abort) |
| Bridge "not as written" | `bridge_node.py:577-617` | none (sign-invariant, 0.1°) |
| Determinism signature | `metrics.py:292-317` (no read-back value) | indirect via step-1 only |
| BridgeStatus.attitude_provider | `bridge_node.py:794-797` → `sensing.py:177-180` | reports the queried provider (decision Q2) |
| Record provenance | `scheduler_node.py:332-361`, `:795` | `attitude_readback_provider` becomes "Attitude Segment Quaternions"; `attitude_provider` stays "" |
| check_record E1 | `campaign_runner.py:278-283` | still non-empty, not rejected (and E1 excluded, Q3) |
| Topic capture | `topic_recorder.py:42-43` | sign may differ |
| Runbook E1 check | `08_deploy_runbook.md:384-387` | reword (Q2) |

## §4 HPOP orbit validity (central question for C)

`scenario_builder.py` sets HPOP (`:246`), classical elements (`:294-330`)
and propagates (`:262`, `:345`). Nothing in the repository configures
drag, SRP, area, mass, Cd/Cr, plug-ins or third bodies. `mass_kg`
(`acav_scenario.yaml:47`) is parsed (`spec.py:183`) and printed (`:392`)
only. HPOP therefore runs on STK's defaults; whether those are
attitude-dependent is [LIVE CHECK]. In the installed agi.stk12 12.6 API,
HPOP drag models are Spherical or Plugin and SRP models Spherical, GPS
or Plugin (`agi/stk12/stkobjects/stkobjects.py` enums AgEDragModel,
AgESRPModel).

| Mechanism during a run | When | Can change SSS-1 orbit? |
|---|---|---|
| Validation satellite create/propagate/probe/delete | startup, before sensing (`validation.py:210-254`, `:279-304`) | different object [LIVE CHECK] |
| `_prepare_realtime_objects` | startup; `realtime_objects: []` → no-op (`scenario_manager.py:226-243`) | no |
| SetAttitude Standard + ClearData | first command; also clears handle cache (`com_backend.py:1222`) | only if forces are attitude-dependent [LIVE CHECK] |
| AddAttitude Quat | every step | only if attitude-dependent and STK re-propagates [LIVE CHECK] |
| Animate SetValues Reset | every step (`com_backend.py:495-499`, `bridge_node.py:284-295`) | [LIVE CHECK] |
| ROS services (set_propagator, create/delete, execute_connect, load/reset) | only if called; ACAV scheduler uses only query_data_provider and compute_access (`scheduler_node.py:314-317`) | not in a normal run |
| compute_access | after each image epoch (`scheduler_node.py:611-645`, `com_backend.py:840-857`) | no write in repository code |
| E5 faults | twin only | no |
| External STK edits | any time | undetectable |

Verdict per experiment E1–E6: "orbit can change after propagation" = NO,
conditional on the live force-model check and the equivalence diagnostic.

## §5 First-query penalty [HYPOTHESIS]

Read order: position, velocity, attitude, rate (`sensing.py:510`, `:357`,
`:368`). Before it on the same FIFO COM worker: the attitude write
(`bridge_node.py:522`) and the fire-and-forget clock advance
(`time_sync.py:392-393` → `bridge_node.py:293-295` →
`com_backend.py:495-499`, `Animate * SetValues … Reset`). Cartesian
Position is always the first provider after a time reset and a new
sample. Hypothesis: a lazy STK recompute charged to the first query. If
so, (C) moves the penalty to the next live query; if it is specific to
HPOP position evaluation, (C) removes it. Measurement: compare the
`execute` mean of the first live query in the COM profile
(`com_backend.py:546-566`) before and after (C), same clean commit and
scenario.

## §6 Unexplained ≈ 32 ms of sense [HYPOTHESIS]

Not LLA/lighting: no callback subscribes to any rate group (no
`subscribe(` call in `stk_bridge/`), `read_lla`/`read_eclipse`
(`sensing.py:453`, `:472`) have no callers, and `_run_rate_groups`
(`bridge_node.py:402`) is outside the timed block (`:400-401`). Inside the
sense timer but outside provider timings: (1) the clock-advance COM call
queued ahead of the sense call (the logged "clock advance" measures only
the submit, `bridge_node.py:391-396`); (2) worker hand-off
(`com_thread.py:259`, `:283`, `:300`); (3) handle resolution excluded from
the profile (`com_backend.py:516-527`); (4) `_is_realtime` →
`list_objects()` at most every 5 s (`sensing.py:277-309`); (5) publishing,
tf, landed check, health (`bridge_node.py:725-750`). (B)/(C) touch none
of (1), (2), (4), (5).

## §7 Terminal epoch

The analysis period ends at window start + 720 s (`scenario_builder.py:209-217`).
On this base the bridge has no end time (`time_sync.py:111-128`), so it
senses at t = 720 s, where the segment provider returned NO DATA
[USER-MEASURED]. Pre-existing and documented
(`09_live_runs_and_results.md:112-115`, `:164-165`, `:194-196`).
(B) must fall back to Attitude Quaternions when the segment query has no
rows, so attitude is still published at 720 s. (C) must never request
epochs ≥ stop time; the terminal epoch uses the instantaneous path (Q4).

## §8 Timing as a reported result

Record fields `mean_step_latency_ms`, `mean_sense_latency_ms`,
`mean_actuate_latency_ms`, `mean_barrier_wait_ms` (`run_record.py:246-249`;
`metrics.py:129-139`), filled from BridgeStatus (`capture_tracking.py:156-185`)
and scheduler ticks (`scheduler_node.py:750-752`); bridge phase timer
(`step_timing.py:45-90`, `ros_publishers.py:358-362`); E6 aggregation,
report and figure (`campaign.py:523-544`, `campaign_cli.py:168-173`,
`report_figures.py:402-437`); 60 s timing log (`bridge_node.py:693-703`);
COM profile (`com_backend.py:560-565`); docs
(`09_live_runs_and_results.md:72-120`, `:139`, `:182-183`;
`C6_analysis.md:97-122`, `:146-150`; `08_deploy_runbook.md:922-1007`).
Labelling: `sensing_mode` and `orbit_batch_samples` at run level (Q2);
`provenance.code_version` (`run_record.py:118`) already separates
code versions.

## §9 Determinism and equivalence check

The signature (`metrics.py:292-317`) covers per-target outcome,
`error_at_image_deg`, `max_error_in_hold_deg` (9 decimals), peak momentum
(9 decimals), saturation count and SAFE — all from twin telemetry. A (B)
error affects it only via step-1 initialisation or by tripping the guard.
For (C), orbit position feeds the line of sight each step: a 1 mm error at
≈ 500 km range is ≈ 2e-9 rad ≈ 1.1e-7 deg (arithmetic, not measured),
above the 1e-9 deg rounding; velocity enters via the roll reference
(`twin_step.m:107-108`). A 1 mm / 1 µm/s audit therefore cannot guarantee
identical signatures. Hence the opt-in equivalence diagnostic (Q5) and
the strict bar (Q6). Diagnostic-run timings are inflated and are never
E6 numbers.

## §10 Pre-existing issues (not fixed in this phase)

| Severity | Issue | Evidence |
|---|---|---|
| HIGH | Gains provenance: record uses `--gains-json` (`scheduler_node.py:793`); twin uses the run-config file (`+adcs/config.m:123-131`); nothing compares them | |
| HIGH (E3) | Execution mode never reaches the bridge; twin rejects non-step gaps | `deploy_run_cli.py:170-199`, `simulink_step.m:47-50` |
| MED | Gain keys not checked against `campaign.GAIN_KEYS` outside RunSpec (`campaign.py:54`, `:82-88`); `config.m` ignores unknown keys | |
| MED | Medium/slow rate groups unused; LLA/eclipse topics documented (`03_message_reference.md:22-23`) never published | no `subscribe(` callers |
| MED (paper) | `mass_kg` never applied to STK; HPOP on STK defaults | `acav_scenario.yaml:47`, `spec.py:183`, `:392` |
| MED (E6 wording) | Clock-advance execution counted inside "sense"; log measures only the submit | `bridge_node.py:391-396`, `:293-295` |
| LOW | "<step>: ok" logged at ERROR when a failed step has no detail | `deploy_run_cli.py:470` |
| LOW | Scheduler TwinStatus subscription best-effort depth 1 | `scheduler_node.py:296`, `:307-309`; `qos.py:27-34` |
| LOW | `time.await_attitude_commands` inert in ACAV | `bridge_node.py:474-476`, `:369-370` |
| LOW | E6 CLI default `--repeats 20` | `campaign_cli.py:205` |
| LOW | `conversions.quaternion_angle_between` (2·acos\|a·b\|) has a precision floor ≈ 3e-8 rad; harmless for the 0.1° landed-attitude check, unusable for sub-1e-8 comparisons (the equivalence diagnostic uses a local 4·atan2 form instead) | `conversions.py:162-173`; `bridge_node.py:610-612` |
| (user) | Startup health timeout at t = 0.1 s; retained `last_error` (`health.py:112`, `:117`) | not investigated |
| MED | `PropagatorName` is not on the STK 12.6 Object Model (absent from the installed agi.stk12 wheel), so `list_objects().propagator_type` from `com_backend.py` `_safe_propagator_name` is probably always `""` on COM. This would affect real-time detection (`sensing.py` `_is_realtime`) and the verifier's printed propagator name. Orbit batching (C) uses `IAgSatellite.PropagatorType` via `get_propagator_type` instead. Not verified live. | `_safe_propagator_name`; agi.stk12 `stkobjects.py` |
| LOW | In E6-diag-1 the runner's `terminate()` did not end the bridge within 30 s: scheduler exit at 15:56:08.133 (`run.log`), bridge still logging until its own terminal-epoch barrier abort at 15:56:37 and `stop()` at 15:56:38 (`bridge.log`). End-of-run summary lines rely on `stop()` running. Not investigated. | `deploy_run_cli.py` `_stop_process`; `results/E6_diag_83c4c77/E6-diag-1.logs/` |
| MED (cost) | `build_acav_twin` bakes `RunConfigPath` into the generated model (`build_acav_twin.m:207`, `:530`), so the twin must be rebuilt for every run — the largest remaining fixed cost per run (3–5 min, user's estimate); candidate for a later phase. | |

## §11 Verdicts (as analysed)

(B): GO WITH CONDITIONS — diagnostic agreement at every lockstep epoch
including t = 0.1 s; compare only on epochs with a written sample; fall
back to Attitude Quaternions when the segment result has no rows; forced
providers stay on the original path; truthful provider reporting;
runbook rewording.

(C): GO WITH CONDITIONS — no attitude-dependent force (live check);
diagnostic shows strict equivalence; lockstep only; never past the stop
time, terminal epoch instantaneous; invalidate also on
`enable_realtime_attitude`/ClearData; implemented in SensingPipeline only
so service queries are unaffected.

## Decisions (user, after review)

* **Q2** BridgeStatus.attitude_provider reports the provider actually
  queried for the published attitude ("Attitude Segment Quaternions" in
  combined mode). Record `sensing_mode` ("separate" | "combined_attitude_rate")
  and `orbit_batch_samples` (1 | N) in summary.json and in the per-run
  bridge_params.yaml copy. Reword the runbook E1 expectation with (B).
* **Q3** E1 excluded entirely: both arms stay on the original path for
  (B) and (C), enforced where the per-run bridge config is written
  (`deploy_run_cli.py:170-199`) when experiment == "E1", with a test.
* **Q4** End time for (C): read the scenario stop time from STK at
  startup; cross-check against `window_duration_s` when available; on
  mismatch log a WARNING and disable batching. Blocks never request
  epochs ≥ stop time; the terminal epoch uses the instantaneous path.
* **Q5** The §9 equivalence diagnostic is its own commit before (B) and (C).
* **Q6** Strict bar: |Δr| ≤ 1e-6 m, |Δv| ≤ 1e-9 m/s, sign-aligned attitude
  angle ≤ 1e-12 rad at every lockstep epoch (bit-identical preferred;
  report exact maxima and exact-zero counts). (C)'s runtime audit defaults
  to the same values (configurable); not loosened without approval.
* **Verdicts after the live results** (user): (B) GO, with the §11
  conditions. (C) GO, on condition that the runs after (C) reproduce the
  reference determinism signature exactly; if a signature changes, stop
  and analyse rather than accept it.

## Live checks

1. **HPOP force model** of `*/Satellite/SSS-1` — **DONE** (see Live results).
2. **Provider equivalence** (segment quaternion vs Attitude Quaternions;
   interval vs instantaneous orbit rows) — **DONE** (see Live results).
3. **First-query penalty** (§5): execute means of the first live query
   before and after (C) — pending.

## Live results E6-BC-1 — measured by the user, single run (code 6238e1f)

Files: `results/E6_BC_1/` (not in the repository); items re-read from the
files during review. One run: nothing here is yet a timing claim.

1. **Determinism.** The signature is identical to E6-smoke-1 (c291dfa) and
   E6-diag-1 (83c4c77); same scenario_sha256 `c8825803…`. Record:
   `sensing_mode` combined_attitude_rate, `orbit_batch_samples` 100,
   `attitude_readback_provider` 'Attitude Segment Quaternions'; 5/5
   captured, no SAFE, 0 dropped samples.
2. **Streams** (user's comparison with E6-smoke-1): not bit-identical, but
   equal to machine precision — `cmd.txt` quaternions max |Δ| 3.7e-15,
   `twin_status` boresight_error_rad max |Δ| 4.8e-14 rad, peak momentum
   fraction ~1e-14 — consistent with the ~26 nm orbit-row differences of
   E6-diag-1, and below the signature's 9-decimal rounding.
3. **(B) and (C) live.** (B) enabled at t = 0.2 s with sign-aligned angle
   0.000e+00 (`bridge.log:141`); terminal-epoch fallback to Attitude
   Quaternions at t = 720 s (`bridge.log:429`). (C) on (`bridge.log:57`),
   exactly one invalidation, revision 11 → 13 at the first command
   (`bridge.log:115`); no "orbit batch fallback" line. Over the run the COM
   profile counts 213 Cartesian Position and 210 Velocity executes, about
   three of each per 100 steps (one interval query plus two audit reads),
   as designed.
4. **Timing** (single runs): step 182.6 → 132.0 ms; sense 151.4 → 88.0 ms;
   actuation 15.5 → 20.1 ms and barrier wait 12.1 → 14.5 ms went up
   (record fields, E6-smoke-1 → E6-BC-1). Loop (twin_acking →
   scheduler_exit) 22m01s → 16m04s; whole run 26m47s → 18m57s (user).
5. **First-query penalty moved to 'Attitude Segment Quaternions'**, the first
   live query after the clock advance (§5 prediction). Its execute time is
   not constant: run-weighted mean 34.9 ms (6853 executes); per 60 s
   profile interval 17.8–31.0 ms up to step ~4050, then 48.0–56.7 ms from
   step ~4420 to the end (t ≈ 405–442 s onward). Per-read attitude in the
   timing lines: 38–65 ms before, 67–79 ms after. In E6-smoke-1 the first
   query (Cartesian Position) stays at ~25–33 ms per interval, mean
   27.9 ms, with no such step. Cause unknown (see hypotheses in the review
   of step 6b); the twin is in IMAGE throughout 270–720 s, and the last
   target (T6) is imaged at 400 s.
6. **Summary lines missing.** Neither "combined attitude/rate summary" nor
   "orbit batch summary" is in `bridge.log`: it ends at 17:26:24.909 (the
   terminal-epoch warning); `run.log` shows scheduler_exit 17:26:25.458 and
   "stopping bridge" 17:26:32.927, so the bridge was terminated before
   stop() ran. Fixed in 2849710 (lines at the terminal epoch and every
   1000 steps).

## Live results — measured by the user, E6-diag-1 on 83c4c77

Files: `results/E6_diag_83c4c77/` (not in the repository). Re-read from
the files during review: `SSS-1_hpop_force_model.txt`,
`E6-diag-1.logs/equivalence_diagnostic_summary.json`,
`E6-diag-1.logs/equivalence_diagnostic.jsonl`, `E6-diag-1.json`.

* **Determinism.** The E6-diag-1 determinism signature is character for
  character identical to E6-smoke-1 (c291dfa); both records carry
  scenario_sha256 `c8825803…ba9`, backend com, lockstep, 5/5 captured,
  no SAFE, 0 dropped samples. The diagnostic did not perturb the loop,
  and E6 repeated exactly on this machine.
* **Force model.** HPOP; EGM2008 21×21; drag on, Spherical, Jacchia-Roberts,
  Cd 2.2, area/mass 0.02; SRP on, Spherical, Cr 1.0, area/mass 0.02,
  dual-cone shadow; third bodies Moon and Sun. Assessment: NO
  attitude-dependent force. Satellite mass in STK is 1000 kg — STK's
  default, confirming that `mass_kg: 8.0` is never applied (§10); the
  area/mass ratio 0.02 is likewise not set by the repository.
* **(B) attitude**: sign-aligned angle 0.0 at all 7199 compared epochs
  (exact_zero 7199/7199), including t = 0.1 s (basic attitude) and the
  first 20 raw dumps — bit-identical.
* **(C) orbit**: |Δr| max 2.59e-8 m, mean 1.34e-8 m; |Δv| max 2.92e-11 m/s,
  mean 1.48e-11 m/s; exact_zero 1385/7199 for both; time_mismatch_ms 0 at
  every epoch. All far below the Q6 bar (1e-6 m, 1e-9 m/s), but not
  bit-identical.
* **Terminal epoch** t = 720 s: `no_segment_result` (segment NO DATA, as on
  main) and `terminal_epoch`; Attitude Quaternions was available there.

**Hypothesis (not fact) for the orbit differences.** The non-zero
differences look like a sample-time offset of about 1e-12 s, not an orbit
difference. Over the 5814 non-zero epochs the median |Δv|/|Δr| is
1.108e-3 s⁻¹, matching a/v = 1.107e-3 s⁻¹ for a circular orbit at
6878.137 km — the ratio a pure time shift produces — and |Δr|/v implies
|Δt| ≈ 1.7e-12 s (median), 3.4e-12 s (max). The exact zeros fall at the
first epoch of each block and at epochs on whole 0.5 s multiples
(block offsets 0, 4, 9, …, 94, 99), and become rarer later in a block
(e.g. offset 89: 47/72). This is consistent with STK forming interval
sample times arithmetically from the block start (start + i·0.1 s, or
accumulated, in double precision), whereas an instantaneous query
evaluates the exact millisecond parsed from its UTCG text: offsets that
are exact binary fractions (0.5 s multiples) agree exactly, others differ
in the last bits of the time. Both paths report the same millisecond
(time_mismatch_ms 0), so the offset is below the text resolution. Not
verified against STK internals.
