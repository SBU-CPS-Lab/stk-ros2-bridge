# Deploy runbook — running a campaign on the Windows machine, offline

This is for whoever has STK 12.6 on their own machine and is running a
campaign there while the analysis happens somewhere else -- the machine
this project's own analysis runs on has no live STK install at all.
Follow this top to bottom for a first run. It is written to be run by
someone who was not in the room when this pipeline was built, so it
says what to type and what to check after each step, not just what the
final command is.

**Live validation status:** E6 has been run on Windows with STK and MATLAB;
see [09_live_runs_and_results.md](09_live_runs_and_results.md). The E1 and E2
comparisons have each been run once (2026-10-08), from modified checkouts;
their results are recorded there too. After the common
setup in sections 1–3, follow section 4 for a first smoke test, section 5 for
E6, [section 5A](#5a-e1--two-runs-to-test-attitude-frame-consistency) for E1,
or [section 5B](#5b-e2--compare-kinematic-and-dynamics-aware-scheduling) for E2.

## 1. Before any of this: the base bridge must already work

Do **not** start here if `check_environment.py --backend com` has never
passed on this machine. Do the base bridge installation and
verification first:

1. [01_installation_windows.md](01_installation_windows.md) -- ROS 2,
   the `agi.stk12` wheel, building the workspace, `check_environment.py`
   green against live STK.
2. [02_matlab_setup.md](02_matlab_setup.md) -- MATLAB with ROS Toolbox,
   `ros2genmsg` working for this project's message types. The recipe in
   that doc (and in the ROS-Toolbox-Python-version table) was worked out
   against a Linux dev box with no STK; if anything there does not match
   what you see on Windows, that is expected -- the underlying MATLAB/
   Python version constraints are the same, the paths will differ.

Build the whole workspace, including `acav`, not just `stk_bridge`:

```powershell
cd <path-to>\stk_ros2_ws
colcon build --symlink-install
. install\local_setup.ps1
```

## 2. Build and verify the ACAV scenario

```powershell
python src\acav\scripts\build_acav_scenario.py --backend com
python src\acav\scripts\verify_acav_scenario.py --backend com
```

Leave the scenario open in STK afterwards. The ACAV bridge config uses
`scenario.use_existing: true` and checks that its name is `ACAV_SSS1`.
A missing or differently named scenario produces an error without creating
or replacing a scenario. `backend.com.attach_to_existing` selects the STK
application only; it does not select this scenario reuse policy.

The ACAV configuration leaves `scenario.realtime_objects` empty: STK
retains the orbit generated from the scenario YAML, while MATLAB sends
attitude commands. The first attitude command enables streamed attitude
without changing the orbit propagator. If an earlier bridge run switched
the satellite to RealTime, close that scenario and repeat the two commands
above to restore it before launching the bridge again.

**Do not proceed past a FAIL.** Paste the verifier's output back for
analysis if anything fails; its detail text is written to be the fix
instruction, but a genuinely new finding (a different STK patch level,
a different licensed module set) is exactly the kind of thing this
project has needed a live round-trip to fix before, four times during
the base bridge build alone.

## 3. The ACAV-specific bridge config

The base bridge's `config/bridge_params.yaml` lists only `matlab_twin`
as a lockstep barrier participant. The case study adds a second one,
`acav_scheduler` (`config/acav_scenario.yaml`'s own `bridge:` section
says so), so it needs its own config file rather than an edit to the
base one that would break the plain bridge phases:

```
src/acav/config/bridge_params_acav.yaml
```

Read its header comment -- it documents exactly what differs from the
base file and why. If your STK session's scenario is named something
other than `ACAV_SSS1`, or you are using a non-`com` backend, edit that
file's `scenario.scenario_name` / `backend.type` to match, the same as
you would for the base bridge's own config.

## 4. One manual run, to prove the loop closes at all

Before trusting a multi-run campaign, run **one** E6 measurement run by
hand. This exercises the same script `run_acav_campaign` will drive
automatically later, but lets you watch each step happen.

```powershell
mkdir results\E6_smoke
python src\acav\scripts\run_acav_run.py `
    --run-id E6-smoke-1 --experiment E6 --scheduler dynamics_aware `
    --execution-mode lockstep --seed 1 --backend com `
    --output results\E6_smoke\E6-smoke-1.json `
    --run-config results\E6_smoke\E6-smoke-1.runconfig.json `
    --bridge-config src\acav\config\bridge_params_acav.yaml
```

Do not create the `--run-config` file by hand. If it does not exist,
`run_acav_run.py` writes it before starting the bridge, from this run's
`--run-id`, `--experiment`, `--gains-json`, `--fault-json` and
`--attitude-provider` (the same fields a campaign writes), as UTF-8
without a byte-order mark. Without `--run-config` it goes beside the
output as `<output>.runconfig.json`. The absolute path is written to
`run.log` and `summary.json` and printed at the MATLAB handoff. An
existing file is left unchanged, but the run stops before startup if it
is not a JSON object, has a byte-order mark, or names a different run id.

What should happen, in order:

1. The script starts the bridge node directly (`python -m
   stk_bridge.nodes.bridge_node` with `config_file:=...bridge_params_acav.yaml`,
   so stopping it stops the real process) and waits for
   `BridgeStatus.state == CONNECTED`, up to 90 s by default
   (`--bridge-connect-timeout-s` to change it).
2. It prints a block telling you to start MATLAB and the Simulink twin,
   and waits for Enter. **This handoff is deliberately manual** --
   see `acav/deploy_run_cli.py`'s own module docstring for why an
   unattended MATLAB step is not scripted here yet. Run, in MATLAB:
   ```matlab
   cd stk_ros2_ws\src\stk_matlab
   setup_stk_ros
   build_acav_twin('RunConfigPath', 'results\E6_smoke\E6-smoke-1.runconfig.json')
   ```
   then open the generated `acav_twin.slx` and press **Play**. Only
   then press Enter in the PowerShell window.
3. The script does **not** just trust that keypress -- it then polls
   `stk/lockstep/ack` for a message starting `matlab_twin:` for up to
   60 s (`--twin-ack-timeout-s`). If this times out, the twin has not
   delivered an ack. It gates all publishers until telemetry is valid;
   confirm the orbit was not switched to RealTime and that the model was
   rebuilt after the last update (its telemetry subscribers must be
   Reliable, matching `qos.telemetry_reliability: reliable` in the ACAV
   bridge config). Check the Subscribe/Publish block topics
   in the model against `stk/lockstep/ack` and
   `stk/status`, not the MATLAB console for errors (there may be none;
   a silently-wrong topic name just never acks).
4. It launches `ros2 run acav acav_scheduler` with this run's
   parameters and waits for it to exit. `auto_shutdown:=true` means the
   scheduler exits on its own once its schedule completes -- you should
   not need to kill it by hand.
5. It prompts you to stop the Simulink model (it has `StopTime = inf`;
   nothing stops it automatically) and press Enter, then stops the
   bridge.

Confirm the run produced a record:

```powershell
Get-Content results\E6_smoke\E6-smoke-1.json | Select-String backend_type
```

should show `"backend_type": "com"`. If it says `"mock"`, the bridge
fell back silently -- the run completed but is **not reportable**
(`acav/run_record.py::RunRecord.reportable`); check `results\E6_smoke\
E6-smoke-1.logs\bridge.log` for why COM was not used.

## 5. A full E6 campaign

Once step 4 has worked at least once:

```powershell
ros2 run acav run_acav_campaign plan `
    --experiment E6 --output-dir results\E6 --repeats 5

ros2 run acav run_acav_campaign run `
    --experiment E6 --output-dir results\E6 --repeats 5 -- `
    python src\acav\scripts\run_acav_run.py `
        --run-id '{run_id}' --experiment '{experiment}' --scheduler '{scheduler}' `
        --execution-mode '{execution_mode}' --seed '{seed}' `
        --output '{output}' --scenario '{scenario}' --backend '{backend}' `
        --attitude-provider '{attitude_provider}' --run-config '{run_config}' `
        --gains-json '{gains_json}' --fault-json '{fault_json}' `
        --bridge-config src\acav\config\bridge_params_acav.yaml
```

In PowerShell, keep every `{placeholder}` inside single quotes as shown.
Unquoted braces are parsed as script blocks instead of literal arguments.
The continuation backtick must be the last character on its line.

`plan` launches nothing -- it just prints the matrix and writes
`results\E6\campaign.json`, so confirm the run count and IDs are what
you expect first. `run` is safe to re-issue if it is interrupted:
completed runs are skipped (`--no-resume` to force a redo). `--` splits
`run_acav_campaign`'s own flags from the launch command that follows
it -- everything after `--` is passed through to `run_acav_run.py`
verbatim, with `{placeholders}` substituted per run.

**5 repeats, not the CLI's default of 20**, because this first campaign
is about proving the pipeline works end to end and building confidence
in the manual MATLAB handoff, not about the statistics yet -- each run
needs a human to press Play in MATLAB and Enter twice, so 20 is a lot
to do by hand the first time through. Raise it once a handful of runs
have gone cleanly.

Every run's logs land in `results\E6\<run_id>.logs\` (see
`acav/deploy_run_cli.py`'s own docstring for exactly what is captured):
`run.log` (this orchestration's own narrative and step-by-step
outcome), `bridge.log`, `scheduler.log`, and `summary.json`.

### Before a campaign whose results will be cited

Run it from a **clean checkout of one commit on `main`**: `git status`
must show no modified tracked files. Every run now records the code
version it ran (the commit, or `<commit>+dirty` when tracked files were
modified): in `run.log`, `summary.json`, the run configuration, the
record's `provenance.code_version` and the campaign's `campaign.json`.
`run_acav_campaign report` warns unless all of a campaign's records come
from one clean commit. Results already obtained, and what is still to be
run, are recorded in `09_live_runs_and_results.md`.

## 5A. E1 — two runs to test attitude-frame consistency

E1 compares **two** lockstep runs using the same scenario, controller and
dynamics-aware scheduler. No `--repeats` or seed sweep is required:

| Run | STK attitude read-back | MATLAB policy |
|---|---|---|
| Control | Validated inertial provider | Strict command/read-back mismatch guard |
| Treatment | `ECF Attitude Quaternions`, intentionally Earth-fixed against inertial commands | Explicit `e1_readback_ablation`; consume measured attitude without the mismatch stop |

The question is whether this deliberately inconsistent interpretation changes
capture performance. Fewer captures or SAFE in treatment can be a scientific
outcome; a Python exception, lost connection or failed startup is not that
outcome. Do not assume the treatment must fail before measuring it.

`run_acav_run.py` applies each run's provider to the bridge itself: it
copies `--bridge-config` to `<run_id>.logs\bridge_params.yaml` with
`sensing.force_attitude_provider` set from the run (the forced provider for
the treatment, empty for the control) and launches the bridge with that
copy. The shared YAML is not edited, and no hand-made bridge files are
needed. The bridge reports the provider it actually used on its status
topic; the scheduler records it in `provenance.attitude_readback_provider`,
and the campaign runner rejects an E1 record without it.

### 5A.1 PowerShell — prepare the environment and scenario

Use the same Visual Studio/ROS-enabled PowerShell environment that successfully
ran E6. Stop old bridge, scheduler and Simulink runs first. Set `$py` to the
ROS-compatible Python explicitly (on the deploy machine a different Python,
Inkscape's, comes first on `PATH`). The paths below are examples; use your
own checkout and interpreter.

```powershell
cd <checkout>\stk_ros2_ws
$py = '<path to the ROS-compatible python.exe>'
& $py -c "import sys, yaml, colcon_core; print(sys.executable)"
if ($LASTEXITCODE -ne 0) { throw 'Check the ROS Python environment.' }

# After source updates, rebuild using the established ROS/Visual Studio shell:
& $py -m colcon build --symlink-install
if ($LASTEXITCODE -ne 0) { throw 'Workspace build failed.' }
. .\install\local_setup.ps1
```

`$py` is a session variable; define it again in a new terminal. An
undefined `$py` makes `& $py` fail before Python starts, while
`$LASTEXITCODE` can still hold an earlier command's exit code.

Follow sections 1–2 for the ROS underlay and STK prerequisites. If generating
a fresh scenario, save/close the previous scenario in STK first, then run:

```powershell
& $py src\acav\scripts\build_acav_scenario.py --backend com
if ($LASTEXITCODE -ne 0) { throw 'Scenario build failed.' }
& $py src\acav\scripts\verify_acav_scenario.py --backend com
if ($LASTEXITCODE -ne 0) { throw 'Scenario verification failed.' }
```

Leave the verified `ACAV_SSS1` scenario open. Do not switch its orbit
propagator to RealTime. Use a clean committed checkout for attributable
results and keep the same code/configuration for both runs.

### 5A.2 PowerShell — plan and launch the two runs

Use a new output directory; never overwrite an earlier campaign.

```powershell
$e1Dir = Join-Path (Get-Location).Path 'results\E1'
if (Test-Path -LiteralPath $e1Dir) {
    throw 'results\E1 already exists. Resume it deliberately or choose a fresh directory.'
}
New-Item -ItemType Directory -Path $e1Dir | Out-Null
ros2 run acav run_acav_campaign plan --experiment E1 --output-dir $e1Dir
if ($LASTEXITCODE -ne 0) { throw 'E1 plan failed.' }

ros2 run acav run_acav_campaign run `
    --experiment E1 --output-dir $e1Dir --stop-on-error -- `
    $py src\acav\scripts\run_acav_run.py `
        --run-id '{run_id}' --experiment '{experiment}' --scheduler '{scheduler}' `
        --execution-mode '{execution_mode}' `
        --output '{output}' --scenario '{scenario}' --backend '{backend}' `
        --attitude-provider '{attitude_provider}' --run-config '{run_config}' `
        --gains-json '{gains_json}' --fault-json '{fault_json}' `
        --bridge-config src\acav\config\bridge_params_acav.yaml
if ($LASTEXITCODE -ne 0) { throw 'E1 campaign incomplete; inspect the run logs.' }
```

The control runs first, then the treatment. Each needs the MATLAB handoff in
5A.3. Wait for `bridge_connected: com` and the prompt to start the twin; do
not launch another bridge manually. If interrupted, stop the old processes
and reissue the `run` command: completed runs are kept and skipped.

### 5A.3 MATLAB/Simulink — build and run the twin for each run

In MATLAB, stop the previous simulation first. Close the old generated model
without saving transient changes, then use the absolute run-config path the
runner printed for **this** run (`<output dir>\<run_id>.runconfig.json`):

```matlab
cd(fullfile('<checkout>', 'stk_ros2_ws', 'src', 'stk_matlab'))
if bdIsLoaded('acav_twin')
    close_system('acav_twin', 0);
end
clear adcs.simulink_step
setup_stk_ros
runConfigPath = '<absolute path of this run''s .runconfig.json>';
assert(isfile(runConfigPath), 'Wrong run configuration path.');
build_acav_twin('RunConfigPath', runConfigPath)
open_system('acav_twin')
```

Press **Run/Play** in Simulink, then press Enter in the waiting PowerShell.
Expect `twin_acking: ok`, followed by scheduler startup and geometry
preparation. At the final prompt, stop Simulink and then press Enter. Keep
STK and its scenario open for the next run.

For the treatment, repeat the whole block with the treatment's run-config
path; do not reuse the control's model. MATLAB should warn that E1's
mismatch guard is disabled and measured attitude drives feedback. The control
must not print that warning.

Before investing in the full treatment run, confirm the bridge really reads
through the forced provider. In another PowerShell, from the workspace:

```powershell
# Replace results\E1 if you selected a different directory.
$logs = 'results\E1\E1-dynamics_aware-lockstep-readbackEcfAttitudeQuaternions.logs'
Select-String -Path "$logs\bridge.log" -Pattern 'ATTITUDE READ-BACK PROVIDER FORCED'
Select-String -Path "$logs\scheduler.log" -Pattern 'bridge attitude read-back'
```

The first line must name `ECF Attitude Quaternions`; the second must say
`FORCED`. Startup validation may also mention that provider while testing
pairings, which alone proves nothing. If either line is missing, or the
scheduler logs an error that the read-back differs from the run's provider,
stop and investigate. SAFE or missed captures may be outcomes; transport
failures or an unexpected MATLAB exception need diagnosis, not
classification as a valid treatment.

### 5A.4 PowerShell — report and inspect the outputs

After both runs have produced their result files:

```powershell
New-Item -ItemType Directory -Force -Path (Join-Path $e1Dir 'figures') | Out-Null
ros2 run acav run_acav_campaign report --experiment E1 --output-dir $e1Dir `
    --json (Join-Path $e1Dir 'figures\E1-report.json')
if ($LASTEXITCODE -ne 0) { throw 'E1 report needs review; inspect the messages above.' }
```

The summary JSON is deliberately inside `figures`, away from the top-level
per-run JSON records. Expect:

| Artifact under the E1 output directory | Meaning |
|---|---|
| `campaign.json` | Two planned run identities and code version |
| `<run_id>.runconfig.json` | MATLAB configuration for that run |
| `<run_id>.json` | Capture outcomes, pointing errors, actuator metrics and provenance |
| `<run_id>.logs/` | Bridge/scheduler/runner logs, `bridge_params.yaml` (the bridge settings actually used), summary and captured topics |
| `figures/E1-report.json` | Machine-readable comparison |
| `figures/E1.png`, `.pdf`, `.svg` | Control versus treatment capture-rate figure |

Reporting reads existing results; it does not rerun the simulation. The three
image files are three formats of one chart. Use PNG for a quick review and
PDF/SVG for vector artwork in the paper.

`capture_rate` is captures divided by attempted targets; `coverage_rate` is
captures divided by all scenario targets. For example, five captures from
five attempts in a seven-target scenario means 100% capture rate and 71.4%
coverage. A target marked `not_attempted` is not a failed capture attempt.
`claim_holds: true` for E1 means both comparison arms are present; it does
not establish statistical significance or prove degradation by itself.

Check `backend_type=com`, `execution_mode=lockstep`, both scheduler exits,
the recorded code/scenario fingerprints, and `attitude_readback_provider`
in both records: `Attitude Quaternions` (the validated provider) for the
control and `ECF Attitude Quaternions` for the treatment. E1 always runs
the original sensing path: `run_acav_run.py` turns off the combined
attitude/rate read and orbit batching for both arms, so both
`<run_id>.logs\summary.json` files must show `"sensing_mode": "separate"`
and `"orbit_batch_samples": 1`. A control record naming `Attitude Segment
Quaternions` means the combined read was active, and the pair is not a
valid E1 comparison. Compare target outcomes, pointing errors,
SAFE and actuator metrics, not only the capture label. Capture here means
satisfying the simulated imaging criteria; it does not generate a camera
image.

Keep the complete E1 directory for analysis. Do not rewrite outcomes to fit
the hypothesis. The known end-of-window missing body-rate sample at
`t=720s` must be distinguished from a failure during the experiment.

### 5A.5 Results so far, and sharing

The 2026-10-08 pair and its reading are recorded in
`09_live_runs_and_results.md` section 6. When describing E1, state that the
treatment deliberately disables the convention-mismatch guard: it does not
show a silent failure of the protected nominal system. Share the complete
output directory using section 6.

## 5B. E2 — compare kinematic and dynamics-aware scheduling

E2 asks whether accounting for the time needed to reorient the spacecraft
improves delivered imaging performance. The current campaign CLI plans two
lockstep runs, in this order:

| Run | Scheduling decision |
|---|---|
| `E2-kinematic-lockstep` | Attempt every target, without rejecting targets for insufficient slew time |
| `E2-dynamics_aware-lockstep` | Decline a target when the estimated slew time exceeds the available time |

Both runs use the same physical twin, controller, target set and capture
criteria. Kinematic planning does not make the simulated spacecraft rotate
instantaneously. Both runs use validated attitude conventions and the strict
mismatch guard: **do not reuse E1's treatment bridge YAML or MATLAB model**.
No seed sweep is required for this two-run comparison. Do not assume that
the dynamics-aware scheduler will win before measuring both results.

### 5B.1 PowerShell — prepare a clean environment and scenario

Use the Visual Studio/ROS-enabled PowerShell from sections 1–2. Stop previous
bridge, scheduler and Simulink runs. Commit any intended source/documentation
changes before collecting attributable results, then keep that revision and
the configuration unchanged across both runs.

```powershell
cd <checkout>\stk_ros2_ws
$py = '<path to the ROS-compatible python.exe>'
& $py -c "import sys, yaml, colcon_core; print(sys.executable)"
if ($LASTEXITCODE -ne 0) { throw 'Check the ROS Python environment.' }

# Rebuild after source updates, using the established ROS/Visual Studio shell.
& $py -m colcon build --symlink-install
if ($LASTEXITCODE -ne 0) { throw 'Workspace build failed.' }
. .\install\local_setup.ps1
& $py -c "from stk_msgs.msg import BridgeStatus; print('stk_msgs import OK')"
if ($LASTEXITCODE -ne 0) { throw 'Load the ROS underlay and workspace overlay.' }
```

Build and verify the scenario as in section 2. Save/close the previous STK
scenario before rebuilding; leave the newly verified `ACAV_SSS1` open:

```powershell
& $py src\acav\scripts\build_acav_scenario.py --backend com
if ($LASTEXITCODE -ne 0) { throw 'Scenario build failed.' }
& $py src\acav\scripts\verify_acav_scenario.py --backend com
if ($LASTEXITCODE -ne 0) { throw 'Scenario verification failed.' }
```

Do not switch the orbit propagator to RealTime. Between the two E2 runs,
leave the scenario open and stop the previous Simulink model before starting
the next one; do not manually modify STK objects or controller settings.

### 5B.2 PowerShell — plan and check the two runs

Keep this PowerShell open so `$py` and `$e2Dir` remain available. Use a fresh
directory for a new campaign; never overwrite the E1 results.

```powershell
$e2Dir = Join-Path (Get-Location).Path 'results\E2'
if (Test-Path -LiteralPath $e2Dir) {
    throw 'results\E2 already exists. Resume it deliberately or choose a fresh directory.'
}
New-Item -ItemType Directory -Path $e2Dir | Out-Null
ros2 run acav run_acav_campaign plan --experiment E2 --output-dir $e2Dir
if ($LASTEXITCODE -ne 0) { throw 'E2 plan failed.' }

# E2 must use the normal bridge configuration, not the E1 forced provider.
& $py -c "import yaml; from pathlib import Path; c=yaml.safe_load(Path('src/acav/config/bridge_params_acav.yaml').read_text(encoding='utf-8-sig')); assert not c.get('sensing', {}).get('force_attitude_provider'), 'Clear the E1 forced provider before E2'; print('Normal attitude provider selection enabled')"
if ($LASTEXITCODE -ne 0) { throw 'E2 bridge configuration needs review.' }
```

Confirm the two run IDs listed above. Planning writes `campaign.json` but
does not run the simulation. The campaign runner creates each run's MATLAB
configuration immediately before launching that run; do not create empty
run-config files manually or copy the E1 ablation configuration.

### 5B.3 PowerShell — launch the campaign

```powershell
ros2 run acav run_acav_campaign run `
    --experiment E2 --output-dir $e2Dir --stop-on-error -- `
    $py src\acav\scripts\run_acav_run.py `
        --run-id '{run_id}' --experiment '{experiment}' --scheduler '{scheduler}' `
        --execution-mode '{execution_mode}' `
        --output '{output}' --scenario '{scenario}' --backend '{backend}' `
        --attitude-provider '{attitude_provider}' --run-config '{run_config}' `
        --gains-json '{gains_json}' --fault-json '{fault_json}' `
        --bridge-config src\acav\config\bridge_params_acav.yaml
if ($LASTEXITCODE -ne 0) { throw 'E2 campaign incomplete; inspect the run logs.' }
```

Keep the placeholder quotes: PowerShell interprets unquoted braces as script
blocks. The backtick must be the last character on its line. Wait for
`bridge_connected: com` and the MATLAB handoff prompt. Do not launch a second
bridge yourself. This command runs the two cases sequentially and still needs
the manual MATLAB handoff for **each** case.

### 5B.4 MATLAB/Simulink — complete each handoff

For the first run, use the following absolute path. If you selected another
output directory, use the corresponding path printed by the runner instead.

```matlab
cd(fullfile('<checkout>', 'stk_ros2_ws', 'src', 'stk_matlab'))
% Stop any previous simulation before closing its model.
if bdIsLoaded('acav_twin')
    close_system('acav_twin', 0);
end
clear adcs.simulink_step
setup_stk_ros
runConfigPath = fullfile('<checkout>', 'stk_ros2_ws', 'results', 'E2', 'E2-kinematic-lockstep.runconfig.json');
assert(isfile(runConfigPath), 'Wait for the runner or correct the run-config path.');
runConfig = jsondecode(fileread(runConfigPath));
assert(strcmp(runConfig.experiment, 'E2'), 'Expected an E2 configuration.');
assert(strcmp(runConfig.attitude_mismatch_policy, 'strict'), 'E2 requires the strict guard.');
assert(isempty(runConfig.attitude_provider), 'E2 must not force a mismatched provider.');
build_acav_twin('RunConfigPath', runConfigPath)
open_system('acav_twin')
```

Press **Run/Play**, then Enter in PowerShell. Expect `twin_acking: ok`,
scheduler startup and `target geometry ready; releasing initial lockstep
barrier` in `scheduler.log`. At completion, stop Simulink when prompted and
press Enter to let the campaign proceed.

The runner then launches the dynamics-aware case. At its MATLAB handoff,
repeat the entire MATLAB block above, replacing only the `runConfigPath`
assignment with:

```matlab
runConfigPath = fullfile('<checkout>', 'stk_ros2_ws', 'results', 'E2', 'E2-dynamics_aware-lockstep.runconfig.json');
```

Rebuild the model for that configuration, press Play, then Enter in PowerShell.
Neither case should print the E1 guard-ablation warning. Let both runs finish;
missed captures can be legitimate results, whereas exceptions or a bridge
failure during execution need diagnosis. If interrupted, reissue 5B.3 after
stopping old processes: the campaign runner checks completed records and
resumes. Do not rerun the fresh-directory preparation or use `--no-resume`
unless you intend a new execution.

If the next run exits before `topic capture started` or bridge startup,
inspect the PowerShell exception as well as `summary.json`. The preflight
check rejects publishers left on the status, scenario-time, ack or clock
topics. Stop and close the previous twin, confirm the previous bridge and
scheduler have exited, and allow ROS discovery to observe their departure
before resuming 5B.3. Keep STK open. An empty `steps` list locates the failure
before startup but does not by itself prove a publisher conflict; retain the
actual exception. Do not delete the successful first result to restart the
second run.

### 5B.5 PowerShell — generate and interpret the report

After both result files exist and both scheduler exits have been checked:

```powershell
New-Item -ItemType Directory -Force -Path (Join-Path $e2Dir 'figures') | Out-Null
ros2 run acav run_acav_campaign report --experiment E2 --output-dir $e2Dir `
    --json (Join-Path $e2Dir 'figures\E2-report.json')
if ($LASTEXITCODE -ne 0) { throw 'E2 report needs review; inspect the messages above.' }
```

This reads existing results without rerunning STK. Expect two result JSONs,
two run-config JSONs, two `.logs` directories, `campaign.json`, and:

| Output in `figures/` | Meaning |
|---|---|
| `E2-report.json` | Numerical comparison grouped by scheduler |
| `E2_capture_coverage.png`, `.pdf`, `.svg` | Capture rate and coverage side by side |
| `E2_outcomes.png`, `.pdf`, `.svg` | Counts of successful, failed and unattempted targets |

Report all three metrics together:

- **Capture rate:** captured / attempted targets.
- **Coverage rate:** captured / all scenario targets.
- **Over-promise gap:** (attempted - captured) / attempted targets.

A scheduler that attempts only one easy target can have 100% capture rate
and poor coverage. Declined targets are `not_attempted`, not failed captures.
Inspect pointing errors and failure categories to explain the difference;
also review SAFE and actuator-limiting episodes using section 8's definition.
`claim_holds: true` only confirms the presence of both scheduler groups;
it does not establish superiority or statistical significance.

Check matching scenario/code fingerprints, normal attitude-provider selection,
strict MATLAB policies and real COM provenance. Retain shutdown warnings and
distinguish them from incomplete runs. One run per policy is a scenario
comparison, not a statistical estimate across missions. Do not substitute
an old E1 or E6 record for the new dynamics-aware E2 run.

### 5B.6 Results so far, and sharing

The 2026-10-08 pair and its reading are recorded in
`09_live_runs_and_results.md` section 7. Archive the complete directory for
review, as in section 6 (choose a new archive name if it already exists):

```powershell
Compress-Archive -Path $e2Dir -DestinationPath E2_run.zip
```

## 6. Sending results back for analysis

Zip and send the whole experiment output directory, not just its JSON
records. For E6:

```powershell
Compress-Archive -Path results\E6 -DestinationPath E6_run.zip
```

For E1, run the report in section 5A.4 first, then use:

```powershell
Compress-Archive -Path results\E1 -DestinationPath E1_run.zip
```

For E2, run the report in section 5B.5 and archive `results\E2` as shown in
section 5B.6. Each bundle should contain only that campaign's outputs.

If that archive already exists, choose a new archive name. Use your actual
output directory if it differs from these examples.

That includes: the `*.json` records, the `*.runconfig.json` files,
every run's `*.logs\` directory (including the `bridge_params.yaml` the
run's bridge was given), `campaign.json`, and, if you also ran
`run_acav_campaign report` (see below), the JSON report and
`figures\*.{pdf,svg,png}`.

Before sending, run the report locally -- it needs no STK or MATLAB, so
it is a good first sanity check even on the machine that just ran the
campaign:

```powershell
ros2 run acav run_acav_campaign report --experiment E6 --output-dir results\E6 --repeats 5
```

If the `unreportable` list is nonempty, check those
specific runs' `run.log` for the COM-fell-back-to-mock warning from
step 4 before sending -- it is the single most likely reason a run
would not count.

## 7. Fixed after the first real attempt

**"bridge_connected: no BridgeStatus received at all" after ~90 s, even
though the bridge log showed it running fine.** Not a timeout, and not
anything wrong on the deploy machine -- a QoS mismatch in this script.
`{prefix}/status` is published RELIABLE + TRANSIENT_LOCAL (so a
late-joining watcher sees current state immediately, per
`stk_bridge/interfaces/qos.py`), and an earlier version of
`deploy_run_cli._wait_for_bridge_connected_ros` subscribed VOLATILE.
DDS never matches a VOLATILE reader to a TRANSIENT_LOCAL writer at
all -- not "arrives late", *never arrives*, with no error on either
side, which is exactly what made this look like a hang rather than a
config problem. Fixed by having both wait functions reuse the bridge's
own `status_qos()` / `command_qos()` instead of hand-rolling a profile.
If you pulled this repo before this fix, update and try again before
chasing anything else -- this was the actual cause of the report above,
not a sign your STK/bridge setup is wrong.

### Lockstep restart and diagnostics

After updating the lockstep code, stop the previous twin and coordinator,
rebuild `stk_bridge` and `acav`, then load ROS and the workspace in a fresh
terminal. Keep the prepared STK scenario open. Start only `run_acav_run.py`;
do not also launch a separate bridge or scheduler. The coordinator now owns
both node processes directly and stops both on exit. It rejects an already
active graph before starting a new run.

At the MATLAB prompt, rebuild `acav_twin` with the new run configuration and
open the newly generated model, rather than reusing an older loaded model.
Run it, then press Enter in the coordinator terminal. The bridge holds step
1 during this handoff and republishes it for late subscribers. Subsequent
steps require both participants' acknowledgements; timeout aborts the bridge
without advancing again. The twin processes each telemetry timestamp once
and waits for matching attitude, orbit and rate timestamps.

ACAV now explicitly uses **reliable telemetry on both the bridge and the
generated model**. The generic bridge still defaults to best effort: using
that default configuration with the ACAV model will not match its reliable
subscriptions. Reliable delivery is appropriate here because the barrier
prevents a newer step from replacing a missing sample. Commands and acks
remain reliable. The ACAV barrier and COM operation timeouts are 30 wall
seconds; neither changes `step_s` or the experiment duration.

An acknowledgement for a previously accepted step can arrive again after
the bridge advances. It is ignored at debug level and never satisfies the
new barrier. An unmatched acknowledgement still produces a warning. Look
for `lockstep barrier timeout ... missing acks` and preceding COM/sensing
errors to diagnose a real stall; a duplicate ack alone is not evidence of
one. If a timeout remains, preserve `bridge.log`, `scheduler.log`, `run.log`
and the MATLAB Diagnostic Viewer output before reusing the same run ID.

### First full run (2026-09-26): the twin's clock

The run connected, the twin acked, and the scheduler ran for two hours
-- but every target failed with `no telemetry samples in the hold
window`, the twin sat in SAFE with its wheels saturated, and the bridge
aborted at step 7201. One cause behind all of it: **the Simulink model
stamped its messages with wall-clock time.** Its header blocks used the
Simulink ROS node's own clock, and nothing runs that node on simulation
time. So:

- the bridge wrote every attitude command to STK at a 2026 time stamp,
  years outside the scenario -- STK's attitude inside the scenario never
  changed, the twin kept seeing the same attitude and no rate, poured
  torque into the wheels until they saturated, and the momentum guard
  latched SAFE;
- the scheduler filed every twin status ~73 million seconds after the
  scenario epoch, where no hold window can be.

The model now stamps every message from the `ScenarioTime` of the step
it is processing: an attitude command at the time the commanded attitude
is reached (this step + `step_s`), a twin status at this step's time.
Separately, the run could not end: with targets unscored, the scheduler
waited for more simulation time past the window's end, which the
scenario does not have. It now waits in wall time instead and always
writes its record.

**After pulling this update, rebuild everything** -- the `.slx` is a
build artifact, and the old one still stamps with the wall clock:

```powershell
colcon build --symlink-install --packages-select stk_msgs stk_bridge acav
```

```matlab
build_acav_twin('RunConfigPath', 'results\E6_smoke\E6-smoke-1.runconfig.json')
```

**Check it within the first minute of the run**, not after two hours:

```powershell
ros2 topic echo --once /acav/twin_status
```

`header.stamp.sec` must be near `1717237800` (the scenario epoch, June
2024), not today's date. The bridge and scheduler now also say so
themselves: a wall-clock stamped command is rejected with an ERROR in
`bridge.log`, and the scheduler logs one ERROR naming the clock
mismatch. The bridge log also gains a line every 60 s with the step
rate and the sense / actuate / barrier-wait breakdown -- the first run
managed only ~1.1 steps/s, and that line is how we find out where the
time goes.

**Automatic topic capture:** `run_acav_run.py` now records the five topics
before starting the bridge, without additional terminals. The run's `.logs`
folder (for example `results/E6_smoke/E6-smoke-1.logs/`) contains UTF-8
`status.txt`, `attitude.txt`, `rate.txt`, `cmd.txt`, and `twin_status.txt`.
Messages use YAML with `---` separators, like `ros2 topic echo`.
Capture stops during normal teardown, errors, or Ctrl+C. `topics_summary.json`
records received-message counts and recorder errors; an empty file means no
message was received, not proof that the publisher was healthy. These passive
best-effort subscriptions do not participate in the lockstep barrier and are
not a lossless recording guarantee. Reusing the same run/output path replaces
its logs; use a new run/output path to preserve a previous run. For multiple
configured satellites, attitude/rate filenames include the satellite namespace.

### Second run (2026-09-27): attitude frames, and a reused scenario

Scheduling worked this time -- every target was finalised -- but every
one ended in SAFE with no access, and the satellite's attitude in STK
looked aimless. Two causes:

- **The twin and STK disagreed about the attitude frame** (the
  colleague's diagnosis, confirmed in the code). The twin computes
  attitude from ICRF position, velocity and target vectors, but the
  bridge read attitude back in the Earth-fixed form (`ECF Attitude
  Quaternions`) and wrote the twin's commands in the Earth-fixed form
  (`AddAttitude CBFQuat`). The numbers went round unchanged, so the twin
  believed it was on target, while STK's actual attitude was rotated by
  the Earth-rotation angle: no access anywhere, and a phantom ~108 deg
  pointing error at t=0 that started a pointless slew.
  **ACAV is now inertial end to end**: the twin commands with
  `AddAttitude Quat` and reads back `Attitude Quaternions`. The bridge
  refuses to start if the attitude read-back does not match the
  configured command form.
- **The previous attempt's attitude was still in STK.** Both attempts
  in this bundle used the same open scenario, and STK keeps an object's
  real-time attitude samples between runs and rejects any sample earlier
  than the newest it holds. Every command of the second attempt was
  rejected until t=58 s -- as far as the first attempt had got -- while
  STK replayed the first attempt's attitude, and the twin's fight
  against it saturated the wheels into SAFE at t=10.6 s. The bridge now
  clears that history before its first command, and a new startup check
  (`attitude_history_clears`) proves the clear works on this install.

Rebuild both sides again after pulling (same commands as above: `colcon
build`, then `build_acav_twin`). Then, **in `bridge.log`, within the
first minute:**

- `validated: attitude='Attitude Quaternions'` -- not `ECF ...`;
- `[ ok ] attitude_history_clears` in the validation summary. If it
  says FAIL, rebuild the ACAV scenario in STK before every run until
  that is resolved;
- `cleared any real-time attitude samples ...` right after
  `real-time attitude enabled`;
- no `AddAttitude rejected` lines. The first, 10th, 100th, ... are
  logged, with the likely cause;
- the 60 s timing line now breaks sensing down per read (orbit,
  attitude, angular rate) and adds the clock advance. Sensing was
  ~0.9 s of every ~1 s step in this run, so that breakdown is what the
  next speed-up will be aimed at.

The bridge also stopped at step 5901 (t = 590.1 s) waiting for the
twin, with no error from the bridge side. One possible cause is now
handled: if one telemetry channel fails to read, the bridge retries it
once and logs a warning, instead of silently publishing an incomplete
step the twin would wait on forever. **Save the MATLAB Diagnostic
Viewer and Command Window output with the next bundle.** If the twin
itself stopped, that is the only place it will say why.

### Third run (2026-09-27): a race between the twin's two messages

The fixes from the second run held -- attitude read back as `Attitude
Quaternions`, `attitude_history_clears` passed (so ClearData works on
this install), and no command was rejected -- but the vehicle still spun
"illogically" and every target ended in SAFE. The attitude and command
captures show why:

- **On 7% of steps, STK's attitude was not the one the twin had just
  commanded.** The twin publishes its attitude command and its lockstep
  ack on two different topics, and ROS 2 does not guarantee they arrive
  in order. When the ack arrived first, the bridge advanced and read the
  next step's attitude before that step's command had landed -- and STK,
  holding no sample for that moment, reported the scenario's basic
  (nadir) attitude instead: a jump of up to 180 deg in one step.
  (The bridge's "commands late" counter showed only 70 of these 265;
  the rest were overwritten by the following command before being
  applied.)
- **The twin then took on the body rate STK computed across that jump.**
  A 1.8 deg jump turned a 1.2 deg/s slew into a steady 8 deg/s spin; a
  155 deg jump produced ~800 deg/s. No reaction wheel can remove that,
  so the wheels saturated and SAFE latched.

Fixed on both sides. The bridge now waits, after the barrier, for each
commanded object's attitude for the next step before it advances
(`time.require_attitude_command_each_step` in ACAV), so message order no longer matters;
and it warns whenever STK's attitude at a step is not the one written
for it. The twin now carries its own body rate from step to step, taking
STK's only once, to start -- STK's rate is just a numerical derivative of
the attitudes the twin itself wrote, so it carries no new information,
and across a missing sample it is wildly wrong.

After pulling: `colcon build` and restart MATLAB (or run `clear
functions`) so the updated `+adcs/simulink_step.m` is loaded; the model
itself does not need rebuilding. **In `bridge.log`**, the 60 s timing
line now ends with `commands late 0, rejected 0, missing 0; attitude not
as written 0` -- all four should stay at 0. Any non-zero value names the
step and the cause in a warning just above it.

The same timing line shows where each step's ~0.8 s goes: orbit read
~380 ms, angular rate ~200 ms, attitude ~180 ms, everything else ~25 ms.
Making the loop faster means reading STK less, not tuning ROS or MATLAB.

### Attitude dynamics ownership and the AngVels rejection

The MATLAB twin integrates and retains both achieved quaternion and
instantaneous body rate. STK provides orbit/access and attitude readback;
its rate derived from quaternion samples must not replace the integrator's
endpoint rate. The twin checks each readback against its previous achieved
command (including equivalent opposite quaternion signs). A time gap or
attitude mismatch stops the twin with an explicit diagnostic.

Do not send `AddAttitude Quat` followed by `AddAttitude AngVels` at the same
epoch. AngVels propagates to a later time; it does not set an existing
sample's rate. This caused the 2026-09-27 run to abort at step 2.
The model publishes only the achieved quaternion. The attitude-command
lockstep gate remains enabled; no angular-rate command is required.

After updating, stop and close the old model and regenerate it with
`build_acav_twin`. No ROS message definitions changed. In this configuration
`/stk/.../angular_rate` remains STK's kinematic telemetry, not the twin's
instantaneous integrated rate.

### Ground-target coordinates and moving-target tracking

The 2026-09-28 T1 miss had a geometric cause: a Target's ungrouped fixed
`Cartesian Position` provider returned Earth-fixed coordinates even when the
request specified `ICRF`. The bridge previously ignored that frame request.
The controller converged on a fictitious inertial point: at t=60 s its real
sensor-to-T1 error was about 41.57 degrees despite a small reported error.
The COM adapter now resolves the object's Center through VGT in the requested
central-body system at each requested epoch, failing explicitly if unavailable.

The scheduler caches time-varying ICRF target positions at 10 s intervals and
interpolates them at the current simulation time, rather than freezing the
position at the future image epoch. The twin estimates the desired attitude's
angular rate between steps and uses it as tracking feedforward in NADIR,
SETTLE and IMAGE. The MATLAB implementation and Python reference agree; wheel
torque/momentum limits and capture tolerances are unchanged.

Restart the bridge and scheduler after rebuilding their packages if needed.
Stop the old Simulink model, clear `adcs.simulink_step`, and regenerate/open
the model for the new run. No custom message definition changed. Check both
the twin's pointing error through t=58..60 s and the scheduler's T1 access
result; an `IMAGE` label alone is not evidence of successful capture.

## 8. What is genuinely unverified here, so you know what to look for

- **The manual MATLAB handoff (section 4, step 2-3).** This is the
  piece of the whole pipeline most likely to need a fix on the first
  try -- the specific ack-topic-name/message-format assumptions were
  taken from `stk_bridge`'s own subscriber code
  (`interfaces/ros_subscribers.py`), not observed against a live twin.
- **`bridge_params_acav.yaml`'s `scenario.scenario_name` matching**
  what `build_acav_scenario.py` actually names the scenario in STK. If
  the bridge fails validation immediately with an object-not-found
  style error, check this first.
- **Timing.** `--bridge-connect-timeout-s` (90 s) and
  `--twin-ack-timeout-s` (60 s) are guesses, not measurements -- STK
  Desktop launch time and how long it takes a human to switch to
  MATLAB and press Play are both machine- and person-dependent. Raise
  either if you see a timeout that would have succeeded given a few
  more seconds.

Send the log bundle either way -- a run that failed cleanly with a
clear log is exactly as useful here as one that succeeded.

### Combined attitude and angular-rate read

`sensing.combine_attitude_rate` (true in `bridge_params_acav.yaml`, false in
the generic bridge) lets one `Attitude Segment Quaternions` query per step
serve both the attitude and the angular-rate topics, instead of a separate
`Attitude Quaternions` query. Each run starts with the original separate
reads. At the first epoch where the bridge wrote an inertial attitude
sample, it compares the two quaternions; it switches to the single query
only if they agree within 1e-12 rad (sign-aligned, q and -q equal).
Otherwise it keeps the separate reads. Any structural change (ClearData,
`SetAttitude Standard`, object or scenario lifecycle, a propagator change
or a raw Connect command) returns it to the separate reads until it is
verified again. If the segment provider returns no data at an epoch (it
does at the 720 s terminal epoch), that step's attitude comes from
`Attitude Quaternions` and no rate is published, as before.

It stays off, whatever the YAML says, for E1 (both arms) and for any run
with a forced attitude provider: `run_acav_run.py` writes
`combine_attitude_rate: false` into the run's `bridge_params.yaml`.
Multi-satellite configurations and validation results other than
`Attitude Quaternions` / `Attitude Segment Quaternions` also keep it off.

To check a run:

```powershell
Select-String -Path "$logs\bridge.log" -Pattern 'combined attitude/rate'
(Get-Content "$logs\summary.json" -Raw | ConvertFrom-Json).sensing_mode
```

The first line should say `combined attitude/rate enabled at ...`, with a
sign-aligned angle of at most 1e-12 rad. The run record then names
`Attitude Segment Quaternions` as `attitude_readback_provider`, because
that is the provider queried for the published attitude. `sensing_mode`
in `summary.json` and in the run's `bridge_params.yaml` is the configured
mode, `combined_attitude_rate` or `separate`.

To compare against the original path, run the same E6 configuration
from a clean commit with the run's bridge configuration copy set to
`combine_attitude_rate: false`. Never pool timings from the two modes:
group them by `sensing_mode`. Before relying on the combined read, its
determinism signature must equal the separate-read reference exactly.

At bridge stop, `bridge.log` has one `combined attitude/rate summary` line:
steps in combined and separate mode, verifications (passed, mismatches),
re-verifications after revision resets, and segment NO-DATA fallbacks. In a
normal ACAV run expect one verification, which passes; two separate steps
(t = 0.1 s, before any command, and the verification step); and one
fallback (the terminal epoch).

### Batched HPOP orbit reads

`sensing.orbit_batch_samples` (100 in `bridge_params_acav.yaml`, 1 = off in
the generic bridge) serves lockstep orbit reads from blocks. A block is two
interval queries (Cartesian Position and Velocity) over the next 100 lockstep
epochs. STK's own HPOP rows are used: nothing is interpolated or propagated in
Python. A block is used only if its UTCG millisecond grid, row counts and
values validate, and its first and last rows match instantaneous STK reads
within `orbit_batch_audit_position_m` (1e-6 m) and
`orbit_batch_audit_velocity_mps` (1e-9 m/s). Otherwise its epochs are read
instantaneously and the fallback is logged. That is six orbit queries per
100 steps instead of 200.

It is used only:

- in lockstep;
- for a satellite whose STK propagator is confirmed HPOP or TwoBody;
- when the scenario stop time read from STK matches the scenario window
  (`run_acav_run.py` writes `orbit_batch_expected_window_s` from the
  scenario specification).

Blocks never reach the stop time, so the terminal epoch (720 s) is read
instantaneously. Cached rows are dropped on scenario or object lifecycle
changes, propagation, orbit writes, `SetAttitude Standard` / `ClearData`
(once, at the first command) and raw Connect commands. Attitude samples do
not drop them, because the ACAV force model is attitude-independent
(spherical drag and SRP, confirmed by `inspect_hpop_force_model.py`).

**Disable batching** (`orbit_batch_samples: 1` in the run's bridge
configuration copy) for any run that edits the scenario in STK while the
bridge runs, and for any attitude-dependent force model (for example
N-plate drag or SRP, or a plug-in). Neither can notify the cache. E1 never
batches: `run_acav_run.py` writes `orbit_batch_samples: 1` for both arms.

At bridge stop, `bridge.log` has one `orbit batch summary` line: blocks
loaded, audits passed and failed, validation and query failures, fallback
blocks, invalidations, and epochs served from the cache versus read
instantaneously (plus terminal and unsupported). In a normal ACAV run
expect one invalidation (the first command) and one terminal epoch. Before
relying on batching, the run's determinism signature must equal the
reference exactly. If it changes, stop and analyse; don't accept it.

**Equivalence diagnostic runs** must set both `combine_attitude_rate: false`
and `orbit_batch_samples: 1` in their bridge configuration copy. Otherwise
the diagnostic compares the new paths with themselves.

### COM sensing optimisation and comparison

The COM backend reuses resolved Data Provider handles for the same object,
provider and reference frame. It still executes each query at its requested
epoch and extracts fresh results; telemetry values are never cached. Scenario
replacement, object creation/deletion, propagator changes, arbitrary Connect
commands and disconnect clear the handle cache. A failed query evicts its
handle and propagates the error. Do not edit or replace scenario objects
manually during a run; restart the bridge after external structural edits.

The integration step, controller gains, sample rates and capture criteria are
unchanged. This removes repeated COM object/provider discovery, but the live
speedup must be measured in the next E6 run. Keep old and new E6 records in
separate result directories so the report does not mix software versions.

### Measuring COM provider execution and extraction

The extraction fast path now reads `DataSets.ElementNames` once per result,
using the API exposed by the installed STK 12.6 wrapper. It avoids per-column
`Item`/`ElementName` calls while keeping names fresh on every epoch, including
column reorderings. No schema cache is needed. If that API is unavailable,
the previous metadata walk remains available. Bulk array dimensions must match
the names; otherwise the adapter falls back to actual per-column metadata and
values, rather than silently applying an incompatible mapping.

Benchmark this against the `E6_profile_02` diagnostic using a fresh directory
and run ID (for example `E6_profile_03` / `E6-profile-3`). Rebuild only
`stk_bridge`, keep scenario/controller/timestep unchanged, and collect ten
minutes of ordinary loop activity after geometry preparation. In the detail
logs, `element_names` should replace the repeated `item`/`name` stages for the
bulk path. Compare per-provider total extraction and step/sensing latency;
do not include setup intervals or claim a speedup from unit-test wall time.
Then complete a full run to compare target outcomes, pointing errors and the
determinism signature before accepting this as a validated optimisation.

For detailed extraction profiling, look for `COM extraction detail` in the
same log. Approximately every 60 seconds of extraction activity, these lines
report count, mean/max milliseconds and errors for each provider/frame/stage:
`datasets`, `count`, `item`, `name`, `bulk_fetch`, `python_conversion`,
`output`, and `total`. Fallback paths additionally report `fallback_metadata`
and `fallback_fetch`. Counts for item/name are per column, not per query;
compare accumulated time (mean multiplied by count), not just their means.
Total includes instrumentation overhead; nested stage totals exclude it.
Bulk fetch measures the entire library call, not an isolated COM transport.
The remaining interval is flushed on normal backend disconnect; forced process
termination may lose it. No extra COM reads are made to collect these timings.

A diagnostic run can be stopped after ten minutes of ordinary lockstep
activity, excluding geometry preparation and its overlapping first interval.
Use a new output directory such as `results/E6_profile_02`. This deliberately
incomplete run is profiling evidence, not a completed E6 experiment. Check
that the new detail lines appear before investing in a full run. Rebuild
`stk_bridge` normally; deleting build/install or regenerating MATLAB messages
is unnecessary for this Python-only instrumentation.

The COM backend now emits `COM provider profile (interval)` in `bridge.log`
approximately once per 60 seconds of provider activity. Each provider/frame
has separate `execute` and `extract` counts and mean milliseconds. Execution
measures the provider call; extraction includes column metadata, data retrieval
and Python conversion. Handle lookup, fixed-object `LocateInSystem`, other
sensing work and queue delays are outside these timings. Failed attempts are
included. Interval boundaries may split a query's two phases; do not expect
identical counts in every line.

`extraction paths` reports successful `bulk` conversions, `fallback` attempts
and `bulk_failed` attempts. A bulk failure disables bulk extraction for that
backend instance, retaining the existing fallback behavior. The fallback now
reuses column handles and names within the same result instead of looking
them up twice. No telemetry values or column schema are cached across epochs.
Single-epoch queries also avoid resolving an unused scenario stop time when
no stop was supplied; ordinary sensing already supplies both timestamps, so
that change should not be counted as a major hot-path speedup.

The supplied `E6_run.zip` baseline contains five reportable campaign runs
(plus a separate smoke run). All five captured 5/5 targets and share one
determinism signature. Across the five campaign records, mean step latency
is 658.06 ms and mean sensing latency is 621.45 ms (94.4% of step latency).
They contain no `code_version` in provenance. Preserve the archive unchanged;
these historical measurements are context, not a controlled before/after
benchmark for a different revision.

After rebuilding the bridge, collect a fresh run in a new results directory
with identical scenario, timestep and controller settings. Compare provider
profiles, total sensing/step latency and target outcomes. No live speedup is
claimed by the offline adapter tests. This phase does not precompute orbit
samples, change telemetry rates or parallelise COM calls.

### End-of-run shutdown and errors

The runner stops the bridge immediately after the scheduler exits, **before**
asking you to stop Simulink. Waiting for that keypress with a live lockstep
bridge used to generate a missing-ack timeout after the run had finished.
Actual scheduler failures and bridge failures during a run still fail the run.

Some older Windows ROS logs additionally contain `Failed to remove timer jump
callback` during teardown. That is a distinct ROS cleanup diagnostic, not a
capture result or proof that all earlier errors can be ignored. Keep it in
the log bundle and assess run completion from the scheduler exit, result
record and timestamps. The startup/termination ordering fix does not claim
to repair that ROS runtime diagnostic.

### Interpreting wheel saturation in the paper

`wheel_saturation_events` counts observed false-to-true transitions of the
aggregate `wheels_saturated` flag. Report it as **actuator-limiting episodes**,
not the number of momentum saturations, failed wheels, or SAFE entries.
Torque clipping alone can raise the flag while momentum remains well below
capacity and the spacecraft continues a normal slew. Momentum blocking and
wheel-health restrictions can also raise it. Continuous limiting is one
episode even if several wheels are involved; missing telemetry can hide or
merge episodes. The current boolean cannot separate the causes.

Use `peak_momentum_fraction` per axis for capacity utilisation and
`entered_safe_mode` for the SAFE outcome. Existing E6 event counts need no
numerical rewrite; their interpretation and figure/table captions must use
this definition. Changing the count to mean capacity hits would invalidate
comparisons with old records.

### E1: explicit convention-guard ablation

E1's forced-provider treatment now writes `attitude_mismatch_policy:
e1_readback_ablation` into its run configuration, together with the experiment
and provider identity. MATLAB warns at startup, retains measured attitude in
feedback and allows the intentional command/read-back disagreement. The E1
control and all ordinary runs retain strict endpoint validation. Time-gap
checks and actuator/SAFE limits remain active in both arms. MATLAB rejects the
ablation policy outside an explicitly identified E1 treatment.

Interpret this as a counterfactual **without** the convention guard, not as
evidence that the protected nominal pipeline silently accepts a wrong frame.
Keep the run configuration with the results. Capture degradation is a
hypothesis to measure, not a guaranteed outcome; a SAFE transition is also a
possible treatment response. Regenerate the twin for each run configuration.

### Geometry preparation before lockstep control

The scheduler resolves one target trajectory at a time before acknowledging
the initial step. The bridge replays the original initial telemetry at the
same timestamp while waiting, without adding COM sensing work to the queue.
Preparation has a separate 180-second deadline; missing or invalid geometry
aborts the scheduler rather than starting a run with incomplete targets.
Look for `target geometry ready; releasing initial lockstep barrier` in
`scheduler.log`. Normal command timeouts and capture criteria are unchanged.
