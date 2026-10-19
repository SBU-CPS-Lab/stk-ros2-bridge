# MATLAB / Simulink setup

The twin runs in MATLAB R2023b with ROS Toolbox and Simulink. Every
recorded run used R2023b Update 6 (`matlab_session.json`,
`matlab_version`). A MATLAB release must match ROS 2 Humble in ROS
Toolbox; check MathWorks' ROS Toolbox system requirements before using
another release.

## 1. Check the toolboxes

```matlab
ver('ros')
ver('simulink')
```

## 2. Visual Studio Build Tools

`ros2genmsg` compiles the custom messages with MSVC. Install Visual
Studio Build Tools with the "Desktop development with C++" workload
first; without it, message generation fails deep inside with a
compiler-not-found error that does not name Visual Studio.

## 3. Generate the message types: `setup_stk_ros`

```matlab
cd <path-to>\stk_ros2_ws\src\stk_matlab
setup_stk_ros()                    % first time
setup_stk_ros('Rebuild', true)     % after any .msg change
```

It adds `+stkros` to the path, runs `ros2genmsg` on `stk_msgs` (output
under `src/matlab_msg_gen`), and checks that the ROS 2 network is
reachable (`setup_stk_ros.m`). **After any `.msg` change** (for
example the `TwinStatus` fields added in d97800c) rebuild with colcon
and run `setup_stk_ros('Rebuild', true)`; the twin's code fingerprint
covers the message definitions, so a model built against stale types
is refused by the twin identity check.

## 4. Two ways to run the twin

The run orchestrator (`run_acav_run.py`) supports both
(`--twin-launch`, `deploy_run_cli.py`).

### Manual (`--twin-launch manual`, the default)

The runner prints the build command and waits:

```matlab
cd('src/stk_matlab'); setup_stk_ros;
build_acav_twin('RunConfigPath', '<this run''s .runconfig.json>')
```

Then open the generated `acav_twin.slx` and press Play. The runner does
not trust the keypress: it waits for the twin's barrier ack and checks
its identity ([00_architecture.md](00_architecture.md) section 9). Each
run needs a fresh build: the model bakes its run-config path in.

### Automated (`--twin-launch matlab-batch`)

The runner starts one fresh MATLAB per run (`acav/twin_launcher.py`):

```
matlab -batch "run_twin_session('<run config>', '<run>.logs\twin.stop')"
       -logfile <run>.logs\matlab.log
```

`run_twin_session.m` builds the model for the run config, starts the
simulation, and polls until the stop file appears; on the stop file or
any error it stops the simulation, closes the model and writes
`<run>.logs\matlab_session.json` (MATLAB version, pid, started, built,
running and stopped times, exit `clean_stop` or `error`, the error
report). Simulink run-time errors are captured from the Diagnostic
Viewer into `simulink_diagnostics.txt` and copied into
`matlab_session.json`. MATLAB's console output goes to
`matlab_console.log`.

The runner waits up to 252.6 s for the first ack
(`AUTOMATED_TWIN_ACK_TIMEOUT_S`: twice the 126.3 s measured for MATLAB
start plus build plus simulation start, at least 180 s). Across the 11
automated runs with a clean stop up to 2026-10-17, launch to first ack
took 103.4-113.2 s, and 152.2 s in one run whose MATLAB was slower in
every phase (results/_archive/20261019_results_cleanup/pilots/E5_P10_free2_3186642). Stopping writes the stop
file and waits up to 120 s; if MATLAB has not exited, the runner kills
the process tree it started, by its own PID only.

**Close the GUI MATLAB during automated runs.** The batch session is a
second MATLAB on a 2-core host, and its load changes free-run results
and every timing. Whether your licence allows two concurrent MATLAB
sessions depends on its type; this was not checked on the deploy host.

## 5. Simulink pacing

`build_acav_twin` sets Simulink pacing to `PollingRateHz` model steps
per wall second (default 100). About 66 steps/s are achieved on the
deploy host at 100 Hz; 200 Hz or more busy-waits. The rate is part of
the twin identity (`... 100Hz`). See [C3_adcs_twin.md](C3_adcs_twin.md).

## 6. Clock

In lockstep the twin integrates one 0.1 s step per bridge step and
acknowledges it; in free-run it integrates as many 0.1 s sub-steps as
ScenarioTime moved ([05_time_synchronization.md](05_time_synchronization.md)).
The twin takes simulation time from the bridge's ScenarioTime, never
from the wall clock.

## 7. The original template and examples

`simulink/build_twin_template.m` and `examples/example_0*.m` are the
framework's generic client examples (telemetry read, a closed attitude
loop, lockstep stepping against the mock backend). The case study does
not use them.
