# Phase 3 backlog — recorded 2026-10-15, not started

> **Index.** The open and done engineering backlog, oldest first; each
> section is dated. Several items have since shipped (for example the
> proposed `target_generator.py` became `acav/scenario_generator.py`);
> check the code before acting on an older item. Candidate studies not yet decided for the paper are
> recorded here, not in [../C1_case_study_spec.md](../C1_case_study_spec.md).
> The newest section is at the end.

To run after Phase 2 items 3 (E3 free-run) and 4 (MATLAB automation).
All of Phase 3 must land before the final paper runs, because it changes
the scenario and therefore every scenario hash. Then: decide repeat
counts, and make the final runs on the final commit.

Paths below are relative to `stk_ros2_ws/src` and refer to commit
`d02d2c5`.

## 3a — Scenario redesign

**Decision.** GS1 is removed entirely: the ground-station / downlink
experiment is dropped from this paper.

**Goal.** A seeded target generator:

- a documented rule;
- it rejects targets without sensor access;
- it controls the distribution of planning margins (easy / borderline /
  infeasible) and of cross-track positions;
- it fills the window, so no long dead time remains after the last
  target;
- T1–T7 stay in the nominal set.

Generated scenarios go to separate files, never into the tracked
`acav_scenario.yaml`, because that file is part of the twin's code
fingerprint. One fixed "nominal" seed is the scenario for E1, E3, E4, E5
and E6.

**What exists**

- **Targets are already relative to the ground track.**
  - Each target is `t_image_s`, `cross_track_km` and `priority`
    (`acav/config/acav_scenario.yaml:114-191`, `placement:
    ground_track_relative`).
  - The builder creates each one from STK's sub-satellite point
    (`acav/acav/scenario_builder.py:505-512`, helpers in
    `acav/acav/ground_track.py:113-209`).
  - A generator therefore only has to produce `(t_image_s,
    cross_track_km)` pairs. No coordinates are needed.
- **Planning margins can already be computed offline, without STK.**
  - `AcavSpec.feasibility_table()` (`acav/acav/spec.py:276-310`) gives
    `required_s`, `available_s`, `margin_s` and `predicted_feasible` per
    target in time order.
  - It uses `slew_model.slew_time_s` and `off_nadir_angle_rad`
    (`acav/acav/slew_model.py:118`, `:162`) plus `settle_estimate_s`.
  - This is the quantity whose distribution the generator must control.
- **Loading a scenario from a file other than the tracked one:**
  - `AcavSpec.load(path)` (`spec.py:149`);
  - `build_acav_scenario --config PATH` (`acav/acav/build_cli.py:52`,
    `:79`);
  - `run_acav_run.py --scenario` (`acav/acav/deploy_run_cli.py:863`);
  - `run_acav_campaign --scenario` (`acav/acav/campaign_cli.py:251`).

  The scheduler hashes the file it loaded into
  `provenance.scenario_sha256`.
- **Reachability in STK is verified per target** by
  `_check_target_reachable` (`acav/acav/scenario_verifier.py:773`).
- **There is a guard on the scenario's shape:** the test
  `test_scenario_spans_feasible_and_marginal_regimes`
  (`acav/test/test_spec.py:91`) asserts that the tracked scenario spans
  both regimes.

**What must be added**

1. **Remove GS1:**
   - the `ground_station:` block of the YAML (`:199-219`);
   - `_create_ground_station` and the terrain-mask setup in the builder
     (`scenario_builder.py:104`, `:146`, `:197`, `:590-670`);
   - the verifier checks `_check_ground_station_exists`,
     `_check_elevation_constraint` and `_check_terrain_mask`
     (`scenario_verifier.py:309`, `:820-911`);
   - the GS references in `ground_track.py:222` and in the docs
     (`00_architecture.md`, `C1_case_study_spec.md`,
     `09_live_runs_and_results.md`).

   This changes the tracked YAML, and with it the twin fingerprint and
   every scenario hash, so do it before any final run.
2. **The generator** (for example `acav/acav/target_generator.py` and a
   CLI):
   - seeded `random.Random(seed)` with a documented rule;
   - draws image times across the whole window and cross-track positions
     within the reachable off-nadir limit;
   - classifies each gap with `feasibility_table()` into easy (margin
     above a threshold), borderline (|margin| below a threshold) and
     infeasible, and accepts a draw only if the requested class mix and
     spacing hold;
   - ensures that the last image time leaves no more than a set dead time
     before the window end.

   Each choice must be a named, documented parameter.
3. **Access rejection**, in two stages:
   - offline, reject targets beyond the reachable off-nadir angle;
   - in STK, the existing `_check_target_reachable` runs on each
     generated scenario. A target without access fails the build, and the
     generator draws the next seed. It is never silently dropped.
4. **Output:** one YAML per seed, in an ignored directory. Each file is
   the tracked file with *only* `targets.list` replaced, plus a header
   comment recording the seed, the generator version and the rule
   parameters.
   - The MATLAB twin always reads the default YAML
     (`stk_matlab/+adcs/config.m:35-38`).
   - So a test must assert that every section other than the targets is
     byte-identical to the tracked file. That keeps the twin and the
     scheduler on the same vehicle and limits.
5. **Nominal scenario.** T1–T7 plus generated targets after 410 s, from
   one fixed seed, so the window no longer ends in about 310 s of dead
   time.
   - Decide first whether the nominal scenario stays the tracked YAML,
     which keeps the twin fingerprint stable, or becomes a generated file.
     I recommend the tracked one.
   - Update `test_spec.py:91` to the new nominal set.

## 3b — E2 over randomised scenarios

**Goal:**
- N seeds, with both schedulers run on every seed (paired);
- the seed and scenario hash recorded for every run;
- paired statistics (for example a Wilcoxon signed-rank test) and
  confidence intervals in `report`;
- optional later: randomise the window start epoch.

**What exists**

- `build_e2_scheduler_comparison` (`acav/acav/campaign.py:231-252`)
  already pairs both schedulers per index. But it uses one
  `scenario_path` for all runs, and `seed` is `None` unless
  `repeats > 1`.
- `compare_schedulers` (`campaign.py:493`) aggregates without pairing.
- `RunSpec.seed` and `provenance.scenario_sha256`
  (`acav/acav/run_record.py:92-95`) exist, and `check_record` already
  checks the seed.
- scipy 1.15.3 is installed on the deploy machine, which provides
  `scipy.stats.wilcoxon` and bootstrap confidence intervals.

**What must be added**

1. A per-run scenario path in `RunSpec`, substituted by
   `CommandLauncher` (it currently uses `campaign.scenario_path`,
   `acav/acav/campaign_runner.py:487`). Also a campaign builder that
   generates and freezes one scenario file per seed *before* the
   campaign starts, written to the manifest with its sha256.
2. `check_record` verifies `provenance.scenario_sha256` against the
   frozen file of the run's seed.
3. Paired analysis in `report`: per-seed differences (dynamics-aware
   minus kinematic) for capture rate, coverage and the over-promise gap;
   the Wilcoxon signed-rank p-value; and bootstrap confidence intervals
   on the median difference.
4. **The scenario must be rebuilt in STK for every seed**
   (`build_acav_scenario --config <seed file>`). That is an extra
   handoff per seed until item 4 automates it; size N with that in mind.
5. Optional: window-start randomisation. `window_start_utc`
   (`acav_scenario.yaml:24`) is read by the builder. Changing it moves
   the orbit geometry and the HPOP ephemeris, so it needs its own
   verification pass.

## 3c — E1 generalisation

**Goal.** For each convention mismatch (angle units, quaternion order,
quaternion direction, frame, time scale, distance units): is it caught
by the startup validation, and if ablated, what effect does it have?

**What exists**

- The bridge's startup validation (`stk_bridge/stk_bridge/core/
  validation.py`) already checks:
  - distance units (`:379`);
  - geodetic angle units (`:413`);
  - real-time attitude (`:477`);
  - quaternion ordering (`:493`);
  - angle-rate scale (`:629`);
  - monotonicity (`:723`).
- E1 today ablates only the attitude read-back provider
  (`MISPAIRED_ATTITUDE_PROVIDER`, `acav/acav/campaign.py:194-217`),
  using the twin's explicit ablation policy (`+adcs/config.m:109-120`).

**What must be added:**
- a table from each mismatch to the validation check that catches it
  (none yet for quaternion direction or time scale);
- for each one, an opt-in injection point. That is a test hook that
  corrupts one convention at one boundary, never active in a normal run,
  and recorded in provenance like `attitude_provider`;
- an E1 campaign with one treatment per mismatch;
- in `report`: caught or not caught, and the effect on capture or
  pointing when ablated.

Each injection touches the bridge or the twin, so each needs its own
gate and a signature check that normal runs are unaffected.

## Noted during the E3 pilot (do not fix yet)

Recorded 2026-10-15 from the E3 pilot analysis (`e3_pilot_analysis.md`).

**Twin loop load.** The twin's Simulink model runs as fast as MATLAB
allows: 114,737 and 205,422 calls in the two free-run pilot runs, about
160–285 per second. It publishes the lockstep ack and the twin identity
on every ready step, in both modes. On this 2-core machine that is a
candidate cause of the multi-second STK COM calls seen in free-run (up
to 3.6 s). Measure first (per-process CPU during a run) before changing
anything.

**Health monitor during startup waits.** The bridge's health monitor
marks the bridge DISCONNECTED during every startup wait (no COM call
for 10 s), in lockstep and free-run alike, and back to CONNECTED when
the loop starts. Benign.

**Free-run never advances STK's animation time.** Only the lockstep
loop calls `_advance_stk_clock`, so free-run reports `clock advance
summary: executed 0`. Data providers are queried by explicit time and
are unaffected. E6's "sense" therefore includes the clock advance only
in lockstep; E6 is lockstep-only, so this is documented, not changed.

**Lockstep simulation-time precision.** Lockstep time is
`epoch_seconds + sim_time` at about 1.7e9 s
(`stk_bridge/core/time_sync.py:388-392`), where a float keeps only about
2.4e-7 s. `sim_seconds()` is that value minus the epoch, and published
stamps carry the same rounding (`conversions.py:59-83`), e.g. a stamp
nanosecond of `36999936` for 0.037 s. The golden lockstep trace
(`test_lockstep_trace_golden.py`) pins the current behaviour. Consumers
of epoch-derived time and their tolerances (read-only survey, commit
`1b561a5`):

| Consumer | Path:line | Tolerance | Below ~2.4e-7 s? |
|---|---|---|---|
| Rate group due check | `stk_bridge/core/rate_group.py:65` | 1e-9 s on `sim_seconds()` | **Yes**: the fast group skips 50 of 250 lockstep steps (no callbacks, so harmless; BridgeStatus rate-group statistics are misleading) |
| Written-sample match (B verification) | `stk_bridge/nodes/bridge_node.py:263` | 1e-6 s on `epoch_seconds` | No (about 4× margin) |
| Attitude-landed check | `stk_bridge/nodes/bridge_node.py:912` | 1e-6 s | No (about 4× margin) |
| Orbit block epochs and grid | `stk_bridge/core/orbit_block.py:33`, `:45` | rounded to integer ms | No |
| Fixed-frame interval sample count | `stk_bridge/backends/com_backend.py:641` | `floor(... + 1e-7)` steps | **Yes**: about 4.8e-7 s / 0.1 s is 4.8e-6 steps, so an interval query could lose one sample (only one-off validation queries use this path today) |
| Capture hold window | `acav/capture_tracking.py:246`, `acav/metrics.py:181` | ±1e-9 s on `time_s` from `scheduler_node.py:466` | **Yes**, but the current image epochs are whole seconds, which convert exactly; fractional image times from the 3a generator could include or drop the boundary sample |
| Twin-stamp sanity check | `acav/scheduler_node.py:102` | 5 s | No |
| Twin telemetry hold | `stk_matlab/simulink/build_acav_twin.m:649` | 1000 ns | No (about 4× margin) |
| Twin lockstep time-gap guard | `stk_matlab/+adcs/simulink_step.m:149` | 1e-6 s | No |
| Twin free-run step count | `stk_matlab/+adcs/simulink_step.m:79` | 1e-6 steps (1e-7 s) | **Yes**: a sample within about 1e-7 s before a step boundary can count one step short (free-run times are wall-clock times anyway) |
| Offline coupling analysis | `acav/coupling_analysis.py` (`_stamp`) | integer seconds subtracted first | Fixed in `1b561a5` |

Any change here alters lockstep behaviour and must be checked against
the golden trace and the E6-smoke-1 signature.

## Noted after the 72be372 live session (do not fix yet)

Runs: `results/_archive/freerun_fixes/E6_P2d_72be372`, `results/_archive/freerun_fixes/E3_P2d_rtf1_72be372`,
`results/_archive/freerun_fixes/E3_P2d_rtf05_72be372` (measured 2026-10-15; per-run CPU
samples in `<run>.cpu.csv`; means below are over gate opening, or
scheduler start in lockstep, to run completion). The host has 2 physical cores and 4
logical processors; "cores" below are logical-processor seconds per
second, so the ceiling is 4.

**The code fingerprint depends on line endings.** `adcs.code_fingerprint`
(`stk_matlab/+adcs/code_fingerprint.m:58-64`) and
`acav/twin_identity.py:75-77` hash each file's bytes as stored on disk.
With `core.autocrlf true` the working tree holds CRLF while the index
holds LF, so the same commit checked out with other line-ending
settings gives a different fingerprint for identical code. Proposal:
hash LF-normalised text (CRLF → LF before hashing) in both, together,
with the shared test vector updated. Not changed now: it changes every
recorded twin identity.

**Access can be evaluated before the image epoch is bracketed.** The
scheduler asks STK for access at `t_i + 0.2 s` of scenario time
(`acav/scheduler_node.py:91`, `:610-618`), assuming the attitude at
`t_i` is already written. In lockstep it is: every read-back at a
written stamp matches it (0.000°). In free-run the twin's commands
lag, and STK's attitude beyond the newest written sample is about the
target's off-nadir angle away from it (T1 19.7°, T3 4.5°, T4 25.4°,
T6 23.5°, i.e. near nadir). At rtf 1.0, T2 scored `no_access` with a
pointing error of 0.0018°: the newest command at or before 110.0 s was
stamped 109.959 s; the next (110.537 s) could not be written before the
bridge sample at 110.547 s; access was requested at ≥ 110.2 s and
answered by 110.602 s. At rtf 0.5, T4 was close: the next command could
not be written before 250.257 s, after the 250.2 s request, and access
was True. STK's attitude at the moment of the access computation is not
logged, so this is strongly supported, not proven. Proposed rule:
compute access only once a command stamped ≥ `t_i` has been written
for the satellite (the bridge already tracks the newest written stamp,
`stk_bridge/core/actuation.py:284`), retrying until then, with a
timeout outcome. Lockstep satisfies the condition at the first query,
so lockstep is unchanged.

**Twin-side stalls at rtf 1.0.** 11 command gaps over 3 s (largest
16.05 s at t ≈ 502 s) while the bridge kept sampling (16–115 bridge
samples inside each). During every gap MATLAB used 0.75–1.16 cores,
about its whole-run mean (1.08), and the sum over all roles was
1.25–2.25 cores of 4 logical. So the MATLAB process was not short of CPU
time; what it spent it on cannot be told from these logs (Simulink loop,
ROS message handling, or hyper-thread sharing of a physical core with
the bridge). Twin loop iterations with valid data per wall second fell
from 159 (pilot, rtf 1.0, bridge sleeping about 0.1 s per cycle) to 27
(rtf 1.0, bridge overrunning 79 % of cycles), and were 132 at rtf 0.5
(7 % overruns), on the same twin code. That points at contention with
the busier bridge, not at the twin change. Measure before changing.

**The twin's Simulink loop is unpaced.** `build_acav_twin.m:143-146`
sets fixed step 0.1 s, normal mode and `StopTime inf`, with no pacing,
so Simulink steps as fast as it can and calls `adcs.simulink_step` every
step; the ack and identity publishers fire on every ready step
(`build_acav_twin.m:483-486`). MATLAB uses about 1 core in every mode
(E6 lockstep 1.11, free-run 1.08 and 1.18). Options, each to be
measured:
- Simulink pacing (`EnablePacing`, a rate well above real time, e.g.
  polling at 100–200 Hz): lockstep determinism unaffected (the barrier
  orders the steps); adds up to one poll interval of latency to each
  lockstep step (E6); changes `build_acav_twin.m`, hence the
  fingerprint; low risk.
- Publish the identity once per second or on change (transient-local
  durability lets a late subscriber still get it), and the ack once per
  step rather than every Simulink step: no effect on determinism;
  reduces DDS traffic; changes the fingerprint; low–medium risk (the
  orchestrator and the barrier must still see them).
- A blocking receive (MATLAB `ros2subscriber`/`receive` with a timeout
  instead of Subscribe blocks): largest saving, but replaces the model's
  I/O; high risk.

**Should the free-run bridge yield when it overruns?** At rtf 1.0 the
bridge overran 4583 of 5802 cycles and so never slept. A minimum sleep
(for example 5–10 ms per cycle) would leave CPU to the twin, but it
changes what "free-run at rtf 1.0" means: the bridge would no longer run
as fast as the host allows. If adopted, it must be a recorded run
condition like the pacing, and the rtf 1.0 arm re-run. Alternatives
outside the code: process priority or CPU affinity, also recorded.

## Recorded 2026-10-17 (P10 live session and P11)

**Candidate studies, not decided** (moved here from the experiment list,
[../C1_case_study_spec.md](../C1_case_study_spec.md) section 9):

- A communication-step study: how the result changes with the exchange
  step (0.1 s today) in lockstep.
- An offline file-exchange baseline: STK ephemeris and attitude
  exchanged by files, without the live loop, as the conventional
  workflow the paper compares against.

**Open items:**

- **Done (7675185): free-run fault runs are part of the E3 matrix.**
  Earlier note: **No committed command for free-run fault runs.** `build_e5_fault_campaign`
  plans seeded fault runs in lockstep only. The free-run fault runs
  (results/E5_P9d_free1, results/E5_P10_free2_3186642) were started by
  results/_analysis/p9d/launch_fault_run.py (gitignored), which plans the
  fault as the builder does and calls `deploy_run_cli.main` in-process.
  E3's fault replay needs a committed path (an execution-mode option on
  the E5 builder, or an E3 fault arm) before the final runs.
- **The twin-ack wait does not end when MATLAB exits.** The d97800c
  incident (results/_archive/incidents/E2_P10_seed1_d97800c) waited
  252.8 s after launch although MATLAB had exited at about 112 s. The
  bridge wait now fails fast on exit (3d143bc); the twin-ack wait could
  do the same with `MatlabBatchTwin.exit_code()`.
- **The MissionMode Stateflow chart is not wired into the model's signal
  path** (`simulink/build_acav_twin.m`, comment at the chart). The mode
  is decided by `adcs.mission_mode` inside `adcs.twin_step`. Wiring the
  chart means making `GuidanceControl` take the chart's mode as an input.
  Until then the paper may not describe the chart as executing
  ([../C3_adcs_twin.md](../C3_adcs_twin.md) section 4).
- **Repeat counts** for E2 seeds, E3 and E6 are still to be decided; the
  runbook writes them as placeholders.
- **MATLAB start/build time varies.** Launch to first ack took
  103.4-113.2 s in 11 automated runs and 152.2 s in one (E5_P10_free2:
  every MATLAB phase slower; cause not determined). The ack timeout
  (252.6 s) still has 100 s of margin over the slowest.

## Left after the live verification session (recorded 2026-10-17)

Code for E1 (six mismatch types), E2 (paired analysis across seeds) and
E3 (mode x fault matrix) is in place; the live verification session
(docs/08_deploy_runbook.md section 7) checks it end to end. Still open:

- **Repeat counts**: `<N_REPEATS>`, `<N_SEEDS>` (up to the ten generated
  scenarios; n <= 5 cannot reach p < 0.05), `<N_FAULT_SEEDS>` and the
  fault-seed list.
- **Free-run rtf 0.5 wall time** with an automated twin: measured for the
  first time in the verification session; the campaign estimate needs it.
- **MissionMode chart**: wire it into the executed loop, or keep it as
  the specification of `adcs.mission_mode` and say so in the paper
  (twin change: fingerprint, `test_twin_model_runs`).
- **Final runs** on one final commit, operator-started STK, by protocol.
- **Paper text** from the final records only.
- **The E4 comments** in the reference scenario YAML, the generated
  scenarios, `+adcs/config.m` and `build_acav_twin.m`: clean them at the
  next planned scenario or twin fingerprint change.

## After the live verification session on d0bf796 (recorded 2026-10-17)

Fixed since: every path handed to MATLAB is absolute (6afe951); the E1
report and the live-check summarizer classify a run without a record as
interrupted, infrastructure failure, guard fail-stop or other (5e0d0f3);
leftover matlab.exe / libmwros2server.exe are recorded before MATLAB
starts and a leftover ROS 2 server is waited on or refused (01bf18d);
the plan uses the measured rtf 0.5 time, 27.3 min from one run
(57dbbfd).

Still open:
- Retry E1 quaternion_order guard-on: its latest attempt was an
  infrastructure failure. The guard stop seen in an earlier attempt is
  evidence the path works, not the session result.
- The cause of "Server failed to start. Details: Transport stopped." is
  not established; watch `twin_leftovers` in the next sessions.
- One folder per experiment (the runbook now sets one per experiment).
- The rtf 0.5 time rests on one run; per-rate repeats for E3 (rtf 0.5
  with fewer repeats) are not supported by the matrix builder yet.
- Repeat counts, the MissionMode decision, final runs and paper text as
  listed above.

## Old open issues: state at the code freeze (recorded 2026-10-17)

Not fixed: each fix would change the dynamics, the twin fingerprint or
the determinism signature.

| Issue | Current state | Affects E1/E2/E3/E6 results? | What the paper must say |
|---|---|---|---|
| `mass_kg` / A/m not applied to HPOP | `acav_scenario.yaml:47` mass is read (`acav/spec.py:183`) and only printed (`spec.py:392`); `scenario_builder.py:246` sets HPOP without mass or areas, so STK defaults apply | The same orbit model in every run and arm: no comparison is biased; absolute orbit and access times are those of STK's defaults | HPOP with STK default mass and areas, not the spacecraft's 8 kg |
| Gains provenance | Gains are recorded as configured (`scheduler_node.py:200-205`, `:859`), not observed from the twin; nominal runs carry `{}`, meaning the twin defaults in `+adcs/config.m` (duplicated in `acav/twin_reference.py` `TwinConfig`) | No: every final run uses the nominal gains (no gain sweep; E4 removed) | The twin's gains are the nominal defaults (C3 section 3) |
| 4 skipped tests | `test_conversions.py:20` (needs a sourced ROS 2 workspace), `test_qos_lockstep.py:4` (rclpy), `test_flake8.py:16`, `test_pep257.py:12` (ament linters): not run by the offline suite | No: conversions and QoS are exercised by every live run; the two others are style checks | Nothing; they are unverified in the offline suite, not in the live runs |
| PropagatorName / rate groups | `scenario_builder.py:294` names the propagator in its SetState from the build report (`HPOP` unless the fallback applied); rate groups are `bridge_params_acav.yaml:120-123` (fast 10 Hz, medium 1 Hz, slow 0.1 Hz); the free-run pacer period is the fastest group's | No change between runs; the fallback would be visible in the build report | The step is 0.1 s; HPOP as built (check the build report of the final session) |
