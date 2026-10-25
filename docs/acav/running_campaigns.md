# Running the experiments

An operator manual for the Windows deploy host. Every command here was
checked against its parser (`acav/deploy_run_cli.py` `_parser`,
`acav/campaign_cli.py` `_parser`, `acav/build_cli.py` `build_parser`).
Commands are PowerShell, run from `stk-ros2-bridge` in a shell where ROS 2
and the workspace are sourced. Sections 4 and 5 write repeat counts as
placeholders (`<N_REPEATS>`, `<N_SEEDS>`, `<N_FAULT_SEEDS>`); the
counts used for the study are in section 7.

## 1. One-time setup

1. STK 12.6, ROS 2 Humble, Python with `agi.stk12`, Visual Studio Build
   Tools, and the workspace build: [installation](../installation.md).
2. MATLAB R2023b with ROS Toolbox and Simulink, and the message types:
   `setup_stk_ros` ([MATLAB client](../matlab/client.md)).
3. Check the bridge against STK once:
   `python src\stk_bridge\scripts\check_environment.py --backend com`
   (STK Desktop open; it attaches, it does not start one).
4. The offline tests:

```powershell
$env:PYTHONDONTWRITEBYTECODE = "1"
python -m pytest src/acav/test src/stk_bridge/test -q -p no:cacheprovider -rs
```

After any code change, before the next `ros2 run`: `colcon build
--symlink-install`, then `. install\local_setup.ps1` in the shell you
run from. `ros2 run` executes the installed package, so a new module or
entry point is not seen until then. After a `.msg` change also run, in
MATLAB, `setup_stk_ros('Rebuild', true)`.

## 2. Per-session checklist

Before the first run of a session, in this order:

- [ ] Close every editor and any other program that opens the project
      tree; close the GUI MATLAB. `git status` is clean and on the intended commit
      (campaigns refuse a dirty tree unless `--allow-dirty`).
- [ ] **Start STK Desktop yourself** (not through a script) and build
      the scenario in it (section 3). Runs refuse an STK started
      through COM (`stk_process` step).
- [ ] Close every 2D and 3D Graphics window in STK and minimise STK
      (`stk_graphics` step; graphics on raised the E6 step from 70.2 to 120.3 ms).
- [ ] No CPU sampler running for timing runs (E6, E3).
- [ ] Do not click into, select text in, or resize the console while a
      run is going. The runner turns QuickEdit off, but leave the
      console alone.
- [ ] No leftover `matlab.exe` / `libmwros2server.exe` from an earlier
      run (Task Manager).

## 3. Build and verify the scenario

Reference scenario (E1, E3, E6):

```powershell
python src\acav\scripts\build_acav_scenario.py --backend com --verify
```

`--scenario <yaml>` builds a seeded scenario instead; for E2 the
`run-seeds` verb does all the building itself (section 7.4).

The build attaches to the STK you opened and fails if none is running
(it never starts STK; `--start-stk` would, and runs then refuse it). If
a scenario is open, it is closed first (logged as "closed the open
scenario"; the build fails if it stays open). After a successful build
and verify, every 2D/3D Graphics window is closed and STK is minimised,
then the state is read back with the run guard's rule: exit 1 with "STK
graphics is not off after the build" otherwise (`--leave-graphics`
skips this). Exit code 0 means built, verified and graphics off. Every
run must be given the same `--scenario` as the scenario open in STK
(the runner checks the file, not STK).

## 4. The experiments

All four use the same per-run command after `--`; the campaign
substitutes the `{placeholders}` per run in Python and passes them as
an argument list, so keep each one in **single quotes** (PowerShell
otherwise parses braces as a script block). The continuation backtick
must be the last character on its line. `--twin-launch matlab-batch`
starts and stops MATLAB per run; manual mode prompts you instead.

**Absolute output folders.** Every command below sets an absolute folder per experiment (`$outE1`, ... from `Join-Path (Get-Location)`) and passes that. MATLAB runs from `src\stk_matlab`, so a relative folder handed to it pointed somewhere else: in the live session on d0bf796 MATLAB could not write `simulink_diagnostics.txt` and `matlab.log` was never made. Since 6afe951 the runner makes every path it gives MATLAB absolute anyway; the absolute folders keep every other tool independent of the current directory too. Use one folder per experiment: E6 and E1 records in one folder cannot be reported separately.

How to check any finished run (two commands):

```powershell
(Get-Content <dir>\<run_id>.logs\summary.json | ConvertFrom-Json).steps | Format-Table step, ok, detail -AutoSize
Select-String -Path <dir>\<run_id>.json -Pattern '"reportable"', '"determinism_signature"'
```

Every step `ok` True, and `"reportable": true`. For a failed step see
[STK notes](../bridge/stk_notes.md) Part 1.

Outputs per run, in the campaign folder: `<run_id>.json` (the record),
`<run_id>.runconfig.json`, `<run_id>.logs\` (summary.json, run.log,
bridge.log, scheduler.log, matlab.log and matlab_session.json for
automated runs, bridge_startup_phases.json, stk_graphics.json, topic
captures, free-run: startup_gate.json, free_run_pacing.json,
twin_substeps.json, twin_loop.csv), plus `campaign.json` and, after
cleanups, `twin_cleanup.jsonl`.

Measured wall time per automated run ([case study](case_study.md) section 8): lockstep about
10-11 min, free-run at rtf 1.0 about 15-16 min, free-run at rtf 0.5
27.3 min (one automated run; `acav/campaign.py`
`MEASURED_WALL_MIN`).

### 4.1 E1: convention mismatch

Purpose: what each interface-mismatch type does to the loop, and whether
the twin's guard sees it ([case study](case_study.md)
section 9). Matrix per repeat: the control; frame_pairing guard on and
guard ablated; one guard-on run per other type (`--e1-types`, default all
six). Lockstep, reference scenario. `plan` prints
`E1 runs = N_REPEATS x (1 control + 2 frame_pairing + 5 other types)`.
Duration: at most `<N_REPEATS>` x 8 x about 10.5 min; guard-on attitude
arms and the time offset are expected to stop early (not timed yet).

```powershell
$outE1 = Join-Path (Get-Location) 'results\E1_<commit>'
ros2 run acav run_acav_campaign plan --experiment E1 --output-dir $outE1 --repeats <N_REPEATS>

ros2 run acav run_acav_campaign run --experiment E1 --output-dir $outE1 --repeats <N_REPEATS> --twin-launch matlab-batch -- `
    python src\acav\scripts\run_acav_run.py `
        --run-id '{run_id}' --experiment '{experiment}' --scheduler '{scheduler}' `
        --execution-mode '{execution_mode}' --seed '{seed}' `
        --output '{output}' --scenario '{scenario}' --backend '{backend}' `
        --attitude-provider '{attitude_provider}' --run-config '{run_config}' `
        --gains-json '{gains_json}' --fault-json '{fault_json}' `
        --mismatch '{mismatch}' --mismatch-guard '{mismatch_guard}' `
        --bridge-config src\acav\config\bridge_params_acav.yaml

ros2 run acav run_acav_campaign report --experiment E1 --output-dir $outE1 --repeats <N_REPEATS>
```

Check: each record's `provenance.mismatch` / `mismatch_guard` names its
arm, and the frame_pairing records read back through
`ECF Attitude Quaternions`. A run expected to stop (guard on with an
attitude mismatch, or the time offset) leaves no record but a
`summary.json`; `report` counts it as a fail-stop in its arm. Send back
the folder, fail-stops included.

### 4.2 E2: scheduler comparison

Purpose: the headline. Matrix: kinematic and dynamics-aware, lockstep,
2 runs per scenario: the reference, and each seeded scenario
`acav_seed<k>.yaml` for k in 1..`<N_SEEDS>` (one campaign folder per
scenario; build that scenario in STK first, section 3). Duration: about
22 min per scenario.

```powershell
$outE2 = Join-Path (Get-Location) 'results\E2_seed1_<commit>'
$scenario = 'src\acav\config\scenarios\generated\acav_seed1.yaml'
ros2 run acav run_acav_campaign plan --experiment E2 --output-dir $outE2 --scenario $scenario

ros2 run acav run_acav_campaign run --experiment E2 --output-dir $outE2 --scenario $scenario --twin-launch matlab-batch -- `
    python src\acav\scripts\run_acav_run.py `
        --run-id '{run_id}' --experiment '{experiment}' --scheduler '{scheduler}' `
        --execution-mode '{execution_mode}' --seed '{seed}' `
        --output '{output}' --scenario '{scenario}' --backend '{backend}' `
        --attitude-provider '{attitude_provider}' --run-config '{run_config}' `
        --gains-json '{gains_json}' --fault-json '{fault_json}' `
        --bridge-config src\acav\config\bridge_params_acav.yaml

ros2 run acav run_acav_campaign report --experiment E2 --output-dir $outE2 --scenario $scenario
```

For the reference scenario leave out `--scenario`. `report` refuses a
folder that mixes scenarios. Send back one folder per scenario. To
analyse the seeds as pairs, give one folder with `--output-dir` and the
others with `--include-dir` ([analysis](analysis.md) section 8).
For the seeded scenarios as a whole, `run-seeds` (section 7.4) does the
builds, runs and both reports in one command.

`report` finds a seeded scenario's file by its digest among the
reference, `--scenario`, and the YAML files in `--scenario-dir`
(default: `src\acav\config\scenarios\generated` under the current
directory) and next to the package; run it from `stk-ros2-bridge` or give
`--scenario-dir`. The paired report writes no figures and writes its
JSON to a file only with `--json PATH` (otherwise it prints it); it
only reads the seed folders.

### 4.3 E3: lockstep against free-run, with fault replay

Purpose: what lockstep buys and what free-run costs, with and without a
seeded wheel fault. One campaign (`build_e3_matrix`): for the no-fault
cell and each of `<N_FAULT_SEEDS>` seeded faults, lockstep x 2
(`--repeats-lockstep`), free-run rtf 1.0 x `<N_REPEATS>`, free-run rtf
0.5 x `<N_REPEATS>` (`--repeats-rtf05`), reference scenario, in that
order. `plan` prints the run count
(`E3 runs = (1 + N_FAULT_SEEDS) x (2 lockstep + 2 x N_REPEATS free-run)`
with the defaults, the per-mode form when either option is given), the
measured wall time per run kind, the combined E1 + E2 + E3 hours, the resume rule and
every run's command. The rate and the fault reach each run through
`{real_time_factor}`, `{fault_json}` and `{run_config}`, substituted by
the campaign in Python: nothing changes between blocks and PowerShell's
quote stripping does not apply.

```powershell
$outE3 = Join-Path (Get-Location) 'results\E3_<commit>'
ros2 run acav run_acav_campaign plan --experiment E3 --output-dir $outE3 --repeats <N_REPEATS> --fault-seeds <N_FAULT_SEEDS>

ros2 run acav run_acav_campaign run --experiment E3 --output-dir $outE3 --repeats <N_REPEATS> --fault-seeds <N_FAULT_SEEDS> --twin-launch matlab-batch -- `
    python src\acav\scripts\run_acav_run.py `
        --run-id '{run_id}' --experiment '{experiment}' --scheduler '{scheduler}' `
        --execution-mode '{execution_mode}' --seed '{seed}' `
        --output '{output}' --scenario '{scenario}' --backend '{backend}' `
        --attitude-provider '{attitude_provider}' --run-config '{run_config}' `
        --gains-json '{gains_json}' --fault-json '{fault_json}' `
        --mismatch '{mismatch}' --mismatch-guard '{mismatch_guard}' `
        --real-time-factor '{real_time_factor}' `
        --bridge-config src\acav\config\bridge_params_acav.yaml

ros2 run acav run_acav_campaign report --experiment E3 --output-dir $outE3 --repeats <N_REPEATS> --fault-seeds <N_FAULT_SEEDS>
```

Check, in addition to section 4: free-run runs have `free_run_start_gate`
ok and a `free_run_pacing` entry in summary.json. `report` gives the
lockstep replay check (each lockstep cell's two runs share one
determinism signature: the fault replay) and the free-run spread per
rate and fault (saturation events, captures, SAFE, distinct
signatures). Then the coupling analysis (section 5). Old E5 folders
still report with `--experiment E5`.

### 4.4 E6: loop cost

Purpose: the measured per-step cost. Matrix: `<N_REPEATS>` identical
lockstep runs, reference scenario, graphics off, operator-started STK,
no CPU sampler, nothing else running. Duration: `<N_REPEATS>` x about
10.5 min.

```powershell
$outE6 = Join-Path (Get-Location) 'results\E6_<commit>'
ros2 run acav run_acav_campaign plan --experiment E6 --output-dir $outE6 --repeats <N_REPEATS>

ros2 run acav run_acav_campaign run --experiment E6 --output-dir $outE6 --repeats <N_REPEATS> --twin-launch matlab-batch -- `
    python src\acav\scripts\run_acav_run.py `
        --run-id '{run_id}' --experiment '{experiment}' --scheduler '{scheduler}' `
        --execution-mode '{execution_mode}' --seed '{seed}' `
        --output '{output}' --scenario '{scenario}' --backend '{backend}' `
        --attitude-provider '{attitude_provider}' --run-config '{run_config}' `
        --gains-json '{gains_json}' --fault-json '{fault_json}' `
        --bridge-config src\acav\config\bridge_params_acav.yaml

ros2 run acav run_acav_campaign report --experiment E6 --output-dir $outE6 --repeats <N_REPEATS>
```

Check: every run's `determinism_signature` equals E6-smoke-1's
(results\E6_smoke\E6-smoke-1.json); `stk_process.started_by` is
`operator`. The report gives mean sense, actuate and barrier wait per
step (mean and spread over runs).

### What to send back

The whole campaign folder (records, `.logs`, `campaign.json`,
`twin_cleanup.jsonl` if present), the commit (`git log -1 --oneline`),
and anything unusual you saw during the session. Do not edit or move
files inside it; superseded attempts stay as `*.superseded-*`.

## 5. Diagnostic runs

**A single run** (outside a campaign), e.g. one automated lockstep run:

```powershell
python src\acav\scripts\run_acav_run.py `
    --run-id E6-check-1 --experiment E6 --scheduler dynamics_aware `
    --execution-mode lockstep --seed 1 --backend com `
    --output results\E6_check_<commit>\E6-check-1.json `
    --bridge-config src\acav\config\bridge_params_acav.yaml `
    --twin-launch matlab-batch --diagnostic-label check
```

`--diagnostic-label` keeps it out of any report. Other options:
`--execution-mode free_run --real-time-factor 0.5`, `--graphics-mode on`
(deliberately graphics on), `--scenario <file>`,
`--bridge-connect-timeout-s` (default 180), `--allow-com-started-stk`
(only for a deliberate comparison; recorded).

**CPU sampler.** Only for diagnostic runs, never for E6 or E3 timing
runs: sample the bridge, STK, MATLAB and its children (including
`libmwros2server.exe`) and the scheduler into `<run>.cpu.csv`. There is
no committed sampler script; use your own and record it with the run.

**Coupling analysis** (any folder, no STK needed):

```powershell
python src\acav\scripts\analyze_coupling.py results\E3_rtf1_<commit>
```

Writes `_coupling\coupling.csv` and one `.coupling.json` per run
([analysis](analysis.md) section 7).

## 6. Failure handling and resume

- **A run failed.** Read its `summary.json` steps and `errors`
  ([STK notes](../bridge/stk_notes.md) Part 1). Fix the cause,
  then re-issue the same `run` command: completed runs are skipped, and
  every run with a failed step (MATLAB not stopping cleanly included) is
  run again. Its previous files are renamed `*.superseded-<time>`, never
  deleted.
- **The campaign was interrupted** (crash, reboot): re-issue the same
  `run` command. Do not delete partial files.
- **Code changed between attempts:** resuming over records from other
  code is refused; start a new folder for the new commit
  (`--resume-across-code-versions` only if you mean to mix).
- **Leftover MATLAB:** the campaign kills only processes the previous
  run recorded and logs them in `twin_cleanup.jsonl`; after a single
  run, check Task Manager yourself.
- **`report` refuses:** it names the groups it will not pool (graphics,
  twin launch, how STK was started, scenario, diagnostic runs). Move
  each group to its own folder; never edit a record.
- **MATLAB failed:** `matlab_session.json` (`exit`, `error`,
  `simulink_diagnostics`), `simulink_diagnostics.txt`, `matlab.log`,
  `matlab_console.log`, in that order. After a twin or `.msg` change run
  `test_twin_model_runs` in MATLAB before the next run.

## 7. Reference campaigns

The configurations used for the study, each from a clean tree. Commands
write one folder per experiment, never reused between experiments.

**Before starting, every session:**

- [ ] Section 2's checklist. You open STK Desktop yourself; no script
      starts it. Close every 2D/3D Graphics window and minimise STK.
- [ ] No leftover twin processes:
      `Get-Process matlab, libmwros2server -ErrorAction SilentlyContinue`
      prints nothing (the runner also records them, step
      `twin_leftovers`, and refuses a leftover ROS 2 server after 60 s).
- [ ] One output folder per experiment, never reused between
      experiments (each block sets its own absolute variable).
- [ ] Let every run end by itself: a time_offset run ends at the twin-ack
      timeout, about 4 to 5 minutes after MATLAB starts. No Ctrl+C: an
      interrupted run is no result and must be repeated.

**Repeat counts and why:**

| Experiment | Runs | Reason |
|---|---|---|
| E6 | 5 | lockstep is deterministic; repeats show the timing spread |
| E1 | 6 types, N_REPEATS = 3 (24 runs) | lockstep: repeats show reproducibility of each arm |
| E3 | lockstep 3, free-run rtf 1.0 5, rtf 0.5 3, N_FAULT_SEEDS = 5 (66 runs) | lockstep repeats show reproducibility (the replay check needs at least two); free-run repeats show variation |
| E2 | reference + seeds 1..10, two arms (22 runs) | with 10 seeds the smallest attainable exact two-sided Wilcoxon p is 0.00195, so Holm over the four metrics can still reach 0.05 |

**E2 analysis, declared before the runs:** primary metric
`capture_rate`; secondary `coverage_rate`, `wheel_energy_J`,
`wheel_saturation_events`, Holm-adjusted together with the primary
(`report --experiment E2 ... --include-dir`, [analysis](analysis.md) section 8).

**Estimated duration** (measured wall time per automated run; lockstep
9.5-11.4 min from five runs, free-run rtf 1.0 15.3-16.2 min from three
runs, free-run rtf 0.5 27.3 min from 1 run): E6 0.79-0.95 h, E1
3.80-4.56 h, E2 3.48-4.18 h, E3 18.69-19.71 h; together 26.77-29.40 h.
Fail-stop E1 arms end sooner, so the E1 figure is an upper bound.

### 7.1 E6, 5 runs

```powershell
$outE6final = Join-Path (Get-Location) 'results\E6_final_<commit>'
ros2 run acav run_acav_campaign plan --experiment E6 --output-dir $outE6final --repeats 5
ros2 run acav run_acav_campaign run --experiment E6 --output-dir $outE6final --repeats 5 --twin-launch matlab-batch -- `
    python src\acav\scripts\run_acav_run.py `
        --run-id '{run_id}' --experiment '{experiment}' --scheduler '{scheduler}' `
        --execution-mode '{execution_mode}' --seed '{seed}' --output '{output}' `
        --scenario '{scenario}' --backend '{backend}' --attitude-provider '{attitude_provider}' `
        --run-config '{run_config}' --gains-json '{gains_json}' --fault-json '{fault_json}' `
        --mismatch '{mismatch}' --mismatch-guard '{mismatch_guard}' --real-time-factor '{real_time_factor}' `
        --bridge-config src\acav\config\bridge_params_acav.yaml
ros2 run acav run_acav_campaign report --experiment E6 --output-dir $outE6final --repeats 5
```

### 7.2 E1, six types, 3 repeats

```powershell
$outE1final = Join-Path (Get-Location) 'results\E1_final_<commit>'
ros2 run acav run_acav_campaign plan --experiment E1 --output-dir $outE1final --repeats 3
ros2 run acav run_acav_campaign run --experiment E1 --output-dir $outE1final --repeats 3 --twin-launch matlab-batch -- `
    python src\acav\scripts\run_acav_run.py `
        --run-id '{run_id}' --experiment '{experiment}' --scheduler '{scheduler}' `
        --execution-mode '{execution_mode}' --seed '{seed}' --output '{output}' `
        --scenario '{scenario}' --backend '{backend}' --attitude-provider '{attitude_provider}' `
        --run-config '{run_config}' --gains-json '{gains_json}' --fault-json '{fault_json}' `
        --mismatch '{mismatch}' --mismatch-guard '{mismatch_guard}' --real-time-factor '{real_time_factor}' `
        --bridge-config src\acav\config\bridge_params_acav.yaml
ros2 run acav run_acav_campaign report --experiment E1 --output-dir $outE1final --repeats 3
```

### 7.3 E3: lockstep, then rtf 1.0, then rtf 0.5

The campaign runs the three modes in that order by itself.

```powershell
$outE3final = Join-Path (Get-Location) 'results\E3_final_<commit>'
ros2 run acav run_acav_campaign plan --experiment E3 --output-dir $outE3final --repeats 5 --fault-seeds 5 --repeats-lockstep 3 --repeats-rtf05 3
ros2 run acav run_acav_campaign run --experiment E3 --output-dir $outE3final --repeats 5 --fault-seeds 5 --repeats-lockstep 3 --repeats-rtf05 3 --twin-launch matlab-batch -- `
    python src\acav\scripts\run_acav_run.py `
        --run-id '{run_id}' --experiment '{experiment}' --scheduler '{scheduler}' `
        --execution-mode '{execution_mode}' --seed '{seed}' --output '{output}' `
        --scenario '{scenario}' --backend '{backend}' --attitude-provider '{attitude_provider}' `
        --run-config '{run_config}' --gains-json '{gains_json}' --fault-json '{fault_json}' `
        --mismatch '{mismatch}' --mismatch-guard '{mismatch_guard}' --real-time-factor '{real_time_factor}' `
        --bridge-config src\acav\config\bridge_params_acav.yaml
ros2 run acav run_acav_campaign report --experiment E3 --output-dir $outE3final --repeats 5 --fault-seeds 5 --repeats-lockstep 3 --repeats-rtf05 3
```

### 7.4 E2: seeds 1..10, then the reference arm, in one command

Open STK and do nothing else; the command builds every scenario itself.
The tree must be clean; folders are suffixed with HEAD's short hash
(`--commit-tag` may name it, and must then match HEAD).

```powershell
$results = Join-Path (Get-Location) 'results'
ros2 run acav run_acav_campaign run-seeds --experiment E2 --output-dir $results --seeds 1-10 --twin-launch matlab-batch -- `
    python src\acav\scripts\run_acav_run.py `
        --run-id '{run_id}' --experiment '{experiment}' --scheduler '{scheduler}' `
        --execution-mode '{execution_mode}' --seed '{seed}' --output '{output}' `
        --scenario '{scenario}' --backend '{backend}' --attitude-provider '{attitude_provider}' `
        --run-config '{run_config}' --gains-json '{gains_json}' --fault-json '{fault_json}' `
        --mismatch '{mismatch}' --mismatch-guard '{mismatch_guard}' --real-time-factor '{real_time_factor}' `
        --bridge-config src\acav\config\bridge_params_acav.yaml
```

Per seed k: build and verify `acav_seed<k>.yaml` (section 3; graphics
checked off), then run E2 into `results\E2_final_seed<k>_<commit>`. A
non-zero build stops that seed only. A seed whose folder already holds a
usable record for both runs from this commit is skipped; records from
another commit are refused, not resumed. Then the reference is rebuilt,
its arm runs into `results\E2_final_ref_<commit>`, and two reports are
produced: the reference arm on its own, and the ten seeds as pairs
(`--include-dir`; the paired analysis refuses non-seeded records). The
summary lists, per seed and for the reference: build, run, usable/
requested runs and folder, plus both report exit codes. The exit code
is non-zero if anything failed; rerunning the same command resumes.

The paired report can also be made on its own, into a separate folder
(the seed folders are only read):

```powershell
$out = Join-Path (Get-Location) 'results\E2_final_paired_<commit>'; New-Item -ItemType Directory -Force $out | Out-Null; $inc = 2..10 | ForEach-Object { '--include-dir'; Join-Path (Get-Location) "results\E2_final_seed${_}_<commit>" }; ros2 run acav run_acav_campaign report --experiment E2 --output-dir (Join-Path (Get-Location) 'results\E2_final_seed1_<commit>') @inc --json (Join-Path $out 'paired_report.json') | Tee-Object (Join-Path $out 'paired_report.txt')
```
