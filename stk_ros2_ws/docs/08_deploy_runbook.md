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

(`--run-config` does not need to exist yet if you are not overriding
gains -- see `campaign_runner.write_run_config`; for a hand-run smoke
test create an empty one first: `'{"run_id":"E6-smoke-1","gains":{},
"fault":null}' | Out-File -Encoding utf8 results\E6_smoke\E6-smoke-1.runconfig.json`.)

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

**What to capture on the next run**, in addition to the log bundle: in
separate terminals, from just before pressing Enter until the run ends,

```powershell
ros2 topic echo /stk/status > status.txt
ros2 topic echo /stk/sss_1/attitude > attitude.txt
ros2 topic echo /stk/sss_1/angular_rate > rate.txt
ros2 topic echo /stk/cmd/attitude > cmd.txt
ros2 topic echo /acav/twin_status > twin_status.txt
```

The attitude, rate and command captures answer the next open question:
whether the body rate STK reports back (it derives it from the attitude
samples the twin sends) matches the rate the twin integrated. The twin
restarts every step from STK's reported rate, so a large mismatch there
would show up as slews that never settle.

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
