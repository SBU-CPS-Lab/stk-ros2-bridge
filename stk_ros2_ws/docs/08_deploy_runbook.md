# Deploy runbook — running a campaign on the Windows machine, offline

This is for whoever has STK 12.6 on their own machine and is running a
campaign there while the analysis happens somewhere else -- the machine
this project's own analysis runs on has no live STK install at all.
Follow this top to bottom for a first run. It is written to be run by
someone who was not in the room when this pipeline was built, so it
says what to type and what to check after each step, not just what the
final command is.

**Nothing in this file has been run against a live STK install.**
Everything below has been verified against an injected launcher (a
fake process standing in for the real one) and against the twin's own
offline reference, never against real STK, real MATLAB, and a real
Windows ROS 2 install all wired together. Expect the first attempt to
surface something this runbook gets wrong -- that is exactly what
section 6 asks you to capture and send back.

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

Leave the scenario open in STK afterwards -- the ACAV bridge config
(section 3) attaches to whatever is already open rather than building a
new one, same as the base bridge's `attach_to_existing: true`.

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

(`--run-config` does not need to exist yet if you are not overriding
gains -- see `campaign_runner.write_run_config`; for a hand-run smoke
test create an empty one first: `'{"run_id":"E6-smoke-1","gains":{},
"fault":null}' | Out-File -Encoding utf8 results\E6_smoke\E6-smoke-1.runconfig.json`.)

What should happen, in order:

1. The script launches `ros2 launch stk_bridge stk_bridge.launch.py
   config_file:=...bridge_params_acav.yaml` and waits for
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
   300 s (`--twin-ack-timeout-s`). If this times out, the twin is
   running but not actually acking the barrier -- check the Subscribe/
   Publish block topics in the model against `stk/lockstep/ack` and
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
        --run-id {run_id} --experiment {experiment} --scheduler {scheduler} `
        --execution-mode {execution_mode} --seed {seed} `
        --output {output} --scenario {scenario} --backend {backend} `
        --attitude-provider {attitude_provider} --run-config {run_config} `
        --gains-json {gains_json} --fault-json {fault_json} `
        --bridge-config src\acav\config\bridge_params_acav.yaml
```

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

## 6. Sending results back for analysis

Zip and send the whole `results\E6\` directory, not just the JSON
records -- the logs are the point for this first round:

```powershell
Compress-Archive -Path results\E6 -DestinationPath E6_run.zip
```

That includes: the `*.json` records, the `*.runconfig.json` files,
every run's `*.logs\` directory, `campaign.json`, and, if you also ran
`run_acav_campaign report` (see below), the JSON report and
`figures\*.{pdf,svg,png}`.

Before sending, run the report locally -- it needs no STK or MATLAB, so
it is a good first sanity check even on the machine that just ran the
campaign:

```powershell
ros2 run acav run_acav_campaign report --experiment E6 --output-dir results\E6
```

If this prints `"unreportable": N` greater than zero, check those
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
  `--twin-ack-timeout-s` (300 s) are guesses, not measurements -- STK
  Desktop launch time and how long it takes a human to switch to
  MATLAB and press Play are both machine- and person-dependent. Raise
  either if you see a timeout that would have succeeded given a few
  more seconds.

Send the log bundle either way -- a run that failed cleanly with a
clear log is exactly as useful here as one that succeeded.
