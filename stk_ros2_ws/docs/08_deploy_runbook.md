# Deploy runbook: running the experiments

An operator manual for the Windows deploy host. Every command here was
checked against its parser (`acav/deploy_run_cli.py` `_parser`,
`acav/campaign_cli.py` `_parser`, `acav/build_cli.py` `build_parser`).
Commands are PowerShell, run from `stk_ros2_ws` in a shell where ROS 2
and the workspace are sourced. Repeat counts that are not decided yet
are written as placeholders (`<N_REPEATS>`, `<N_SEEDS>`,
`<N_FAULT_SEEDS>`): never substitute a number nobody decided.

## 1. One-time setup

1. STK 12.6, ROS 2 Humble, Python with `agi.stk12`, Visual Studio Build
   Tools, and the workspace build: [01_installation_windows.md](01_installation_windows.md).
2. MATLAB R2023b with ROS Toolbox and Simulink, and the message types:
   `setup_stk_ros` ([02_matlab_setup.md](02_matlab_setup.md)).
3. Check the bridge against STK once:
   `python src\stk_bridge\scripts\check_environment.py --backend com`
   (STK Desktop open; it attaches, it does not start one).
4. The offline tests:

```powershell
$env:PYTHONDONTWRITEBYTECODE = "1"
python -m pytest src/acav/test src/stk_bridge/test -q -p no:cacheprovider -rs
```

After any `.msg` change: `colcon build --symlink-install` and, in
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

Reference scenario (the default for E1, E3, E6 and E2's reference arm):

```powershell
python src\acav\scripts\build_acav_scenario.py --backend com --verify
```

A seeded scenario (E2), e.g. seed 1:

```powershell
python src\acav\scripts\build_acav_scenario.py --backend com --verify `
    --scenario src\acav\config\scenarios\generated\acav_seed1.yaml
```

Both build into the STK you opened and fail if none is running;
`--start-stk` would start one through COM, which runs then refuse.
Exit code 0 means built and verified. To switch back to the reference,
build the reference again; every run must be given the same
`--scenario` as the scenario open in STK (the runner checks the file,
not STK). Building the 22-target seeded scenario takes several minutes
(not timed precisely; measure it in your session).

## 4. The experiments

All four use the same per-run command after `--`; the campaign
substitutes the `{placeholders}` per run in Python and passes them as
an argument list, so keep each one in **single quotes** (PowerShell
otherwise parses braces as a script block). The continuation backtick
must be the last character on its line. `--twin-launch matlab-batch`
starts and stops MATLAB per run; manual mode prompts you instead.

How to check any finished run (two commands):

```powershell
(Get-Content <dir>\<run_id>.logs\summary.json | ConvertFrom-Json).steps | Format-Table step, ok, detail -AutoSize
Select-String -Path <dir>\<run_id>.json -Pattern '"reportable"', '"determinism_signature"'
```

Every step `ok` True, and `"reportable": true`. For a failed step see
[06_troubleshooting.md](06_troubleshooting.md) Part 1.

Outputs per run, in the campaign folder: `<run_id>.json` (the record),
`<run_id>.runconfig.json`, `<run_id>.logs\` (summary.json, run.log,
bridge.log, scheduler.log, matlab.log and matlab_session.json for
automated runs, bridge_startup_phases.json, stk_graphics.json, topic
captures, free-run: startup_gate.json, free_run_pacing.json,
twin_substeps.json, twin_loop.csv), plus `campaign.json` and, after
cleanups, `twin_cleanup.jsonl`.

Measured wall time per automated run (C1 section 8): lockstep about
10-11 min, free-run at rtf 1.0 about 15-16 min, free-run at rtf 0.5
about 38 min (one manual run).

### 4.1 E1: convention mismatch

Purpose: what the convention validation prevents. Matrix: 2 lockstep
runs (validated pairing; mis-paired read-back with the twin's guard
ablated), reference scenario. Duration: about 25 min.

```powershell
ros2 run acav run_acav_campaign plan --experiment E1 --output-dir results\E1_<commit>

ros2 run acav run_acav_campaign run --experiment E1 --output-dir results\E1_<commit> --twin-launch matlab-batch -- `
    python src\acav\scripts\run_acav_run.py `
        --run-id '{run_id}' --experiment '{experiment}' --scheduler '{scheduler}' `
        --execution-mode '{execution_mode}' --seed '{seed}' `
        --output '{output}' --scenario '{scenario}' --backend '{backend}' `
        --attitude-provider '{attitude_provider}' --run-config '{run_config}' `
        --gains-json '{gains_json}' --fault-json '{fault_json}' `
        --bridge-config src\acav\config\bridge_params_acav.yaml

ros2 run acav run_acav_campaign report --experiment E1 --output-dir results\E1_<commit>
```

Check: the treatment's `summary.json` shows the forced provider, and
its record's `provenance.attitude_readback_provider` is
`ECF Attitude Quaternions`. Send back the folder.

### 4.2 E2: scheduler comparison

Purpose: the headline. Matrix: kinematic and dynamics-aware, lockstep,
2 runs per scenario: the reference, and each seeded scenario
`acav_seed<k>.yaml` for k in 1..`<N_SEEDS>` (one campaign folder per
scenario; build that scenario in STK first, section 3). Duration: about
22 min per scenario.

```powershell
$scenario = 'src\acav\config\scenarios\generated\acav_seed1.yaml'
ros2 run acav run_acav_campaign plan --experiment E2 --output-dir results\E2_seed1_<commit> --scenario $scenario

ros2 run acav run_acav_campaign run --experiment E2 --output-dir results\E2_seed1_<commit> --scenario $scenario --twin-launch matlab-batch -- `
    python src\acav\scripts\run_acav_run.py `
        --run-id '{run_id}' --experiment '{experiment}' --scheduler '{scheduler}' `
        --execution-mode '{execution_mode}' --seed '{seed}' `
        --output '{output}' --scenario '{scenario}' --backend '{backend}' `
        --attitude-provider '{attitude_provider}' --run-config '{run_config}' `
        --gains-json '{gains_json}' --fault-json '{fault_json}' `
        --bridge-config src\acav\config\bridge_params_acav.yaml

ros2 run acav run_acav_campaign report --experiment E2 --output-dir results\E2_seed1_<commit> --scenario $scenario
```

For the reference scenario leave out `--scenario`. `report` refuses a
folder that mixes scenarios. Send back one folder per scenario. To
analyse the seeds as pairs, give one folder with `--output-dir` and the
others with `--include-dir` ([C6_analysis.md](C6_analysis.md) section 8).

### 4.3 E3: lockstep against free-run, and fault replay

Purpose: what lockstep buys and what free-run costs. Matrix:
`<N_REPEATS>` lockstep and `<N_REPEATS>` free-run runs, identical
configuration, reference scenario; one campaign at rtf 1.0 and one at
rtf 0.5 (`--real-time-factor` in the run command; it applies to the
free-run runs, lockstep ignores the wall clock). Duration per campaign:
`<N_REPEATS>` x (about 10.5 min + about 15.5 min) at rtf 1.0;
`<N_REPEATS>` x (about 10.5 min + about 38 min) at rtf 0.5.

```powershell
ros2 run acav run_acav_campaign plan --experiment E3 --output-dir results\E3_rtf1_<commit> --repeats <N_REPEATS>

ros2 run acav run_acav_campaign run --experiment E3 --output-dir results\E3_rtf1_<commit> --repeats <N_REPEATS> --twin-launch matlab-batch -- `
    python src\acav\scripts\run_acav_run.py `
        --run-id '{run_id}' --experiment '{experiment}' --scheduler '{scheduler}' `
        --execution-mode '{execution_mode}' --seed '{seed}' `
        --output '{output}' --scenario '{scenario}' --backend '{backend}' `
        --attitude-provider '{attitude_provider}' --run-config '{run_config}' `
        --gains-json '{gains_json}' --fault-json '{fault_json}' `
        --bridge-config src\acav\config\bridge_params_acav.yaml `
        --real-time-factor 1.0

ros2 run acav run_acav_campaign report --experiment E3 --output-dir results\E3_rtf1_<commit> --repeats <N_REPEATS>
```

For rtf 0.5, use another folder (`results\E3_rtf05_<commit>`) and
`--real-time-factor 0.5`. `--repeats` must be at least 2.

Check, in addition to section 4: free-run runs have
`free_run_start_gate` ok and a `free_run_pacing` entry in summary.json.
Then the coupling analysis (section 5).

**Fault replay (lockstep).** `<N_FAULT_SEEDS>` seeded wheel-fault runs,
one per seed, through the E5 builder:

```powershell
ros2 run acav run_acav_campaign plan --experiment E5 --output-dir results\E5_lock_<commit> --seeds <N_FAULT_SEEDS>

ros2 run acav run_acav_campaign run --experiment E5 --output-dir results\E5_lock_<commit> --seeds <N_FAULT_SEEDS> --twin-launch matlab-batch -- `
    python src\acav\scripts\run_acav_run.py `
        --run-id '{run_id}' --experiment '{experiment}' --scheduler '{scheduler}' `
        --execution-mode '{execution_mode}' --seed '{seed}' `
        --output '{output}' --scenario '{scenario}' --backend '{backend}' `
        --attitude-provider '{attitude_provider}' --run-config '{run_config}' `
        --gains-json '{gains_json}' --fault-json '{fault_json}' `
        --bridge-config src\acav\config\bridge_params_acav.yaml

ros2 run acav run_acav_campaign report --experiment E5 --output-dir results\E5_lock_<commit> --seeds <N_FAULT_SEEDS>
```

The fault reaches the run through `{fault_json}` and `{run_config}`,
substituted by Python, so PowerShell's quote stripping does not apply.
Reproduction: run the same campaign into a second folder and compare
each run's `determinism_signature` between the two (they must be equal;
E5_P9d_lock1 and E5_P9d_lock2 are an example pair). `report` checks
that every seeded fault tripped SAFE.

**Fault runs in free-run: no committed command yet.** The E5 builder
plans lockstep runs only; the free-run fault runs so far were started
by an uncommitted analysis script ([dev/phase3_backlog.md](dev/phase3_backlog.md),
2026-10-17). Do not improvise one; this needs a decision first.

### 4.4 E6: loop cost

Purpose: the measured per-step cost. Matrix: `<N_REPEATS>` identical
lockstep runs, reference scenario, graphics off, operator-started STK,
no CPU sampler, nothing else running. Duration: `<N_REPEATS>` x about
10.5 min.

```powershell
ros2 run acav run_acav_campaign plan --experiment E6 --output-dir results\E6_<commit> --repeats <N_REPEATS>

ros2 run acav run_acav_campaign run --experiment E6 --output-dir results\E6_<commit> --repeats <N_REPEATS> --twin-launch matlab-batch -- `
    python src\acav\scripts\run_acav_run.py `
        --run-id '{run_id}' --experiment '{experiment}' --scheduler '{scheduler}' `
        --execution-mode '{execution_mode}' --seed '{seed}' `
        --output '{output}' --scenario '{scenario}' --backend '{backend}' `
        --attitude-provider '{attitude_provider}' --run-config '{run_config}' `
        --gains-json '{gains_json}' --fault-json '{fault_json}' `
        --bridge-config src\acav\config\bridge_params_acav.yaml

ros2 run acav run_acav_campaign report --experiment E6 --output-dir results\E6_<commit> --repeats <N_REPEATS>
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
([C6_analysis.md](C6_analysis.md) section 7).

## 6. Failure handling and resume

- **A run failed.** Read its `summary.json` steps and `errors`
  ([06_troubleshooting.md](06_troubleshooting.md) Part 1). Fix the cause,
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
  `test_twin_model_runs` in MATLAB before the next live session.
