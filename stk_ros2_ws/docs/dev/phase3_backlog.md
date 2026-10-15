# Phase 3 backlog — recorded 2026-10-15, not started

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
