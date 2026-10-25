# stk_ros2_ws

A co-simulation framework that couples **STK 12.6 (2023 R1)** to a
**MATLAB R2023b / Simulink** digital twin of a satellite's attitude
control system over **ROS 2 Humble**, and a case study built on it,
**ACAV** (Attitude-Coupled Access Verification): does an imaging
schedule that is feasible on geometry alone survive the attitude
dynamics? STK propagates the orbit (HPOP) and computes access at the
attitude the twin actually achieves; the twin integrates the dynamics
and the controller; a bridge node is the clock authority and the only
process that talks to STK. Runs execute in **lockstep** (deterministic,
barrier per 0.1 s step) or **free-run** (wall clock times a real-time
factor), and every result carries the provenance needed to say which
code, configuration, scenario and conditions produced it.

## Layout

```
src/
  stk_msgs/     ROS 2 interfaces (msg/srv/action), incl. TwinStatus, TargetSetpoint
  stk_bridge/   the bridge: STK backends, time, sensing, actuation, validation
  stk_matlab/   the MATLAB/Simulink twin (+adcs, build_acav_twin, run_twin_session)
  acav/         the case study: scenarios, scheduler, run orchestrator, campaigns, analysis
docs/           the documentation (map below); docs/dev/ is engineering history
results/        live run outputs (not in git); results/_archive holds superseded runs
```

## Quick start

```powershell
# Windows, PowerShell, ROS 2 Humble sourced
cd stk_ros2_ws
colcon build --symlink-install
. install\local_setup.ps1
python -m pytest src/acav/test src/stk_bridge/test -q -p no:cacheprovider -rs
```

The offline suite needs no ROS, STK or MATLAB; four tests are skipped
outside a sourced ROS 2 workspace. After any code change, repeat the
`colcon build --symlink-install` and `. install\local_setup.ps1` lines
before the next `ros2 run`. Then
[installation](docs/installation.md) for STK,
[MATLAB client](docs/matlab/client.md) for MATLAB, and
[running campaigns](docs/acav/running_campaigns.md) to run experiments.

## Documentation map

**New to the project?** Read in this order: this page,
[00](docs/bridge/architecture.md), [C1](docs/acav/case_study.md),
[09](docs/09_live_runs_and_results.md).
**Reviewing the method:** 00, 05, C1, C3, C4, C5, C6.
**Running experiments:** 01, 02, 08, then 06 when something fails.
**Extending the bridge:** 00, 03, 04, 07.

| Doc | Covers |
|---|---|
| [architecture](docs/bridge/architecture.md) | The framework as built: components, both execution modes step by step, sensing, actuation, health, the provenance chain, known constraints |
| [installation](docs/installation.md) | Installing STK, ROS 2, `agi.stk12`, the workspace; first checks |
| [MATLAB client](docs/matlab/client.md) | MATLAB, message generation, manual and automated (`matlab -batch`) twin |
| [messages](docs/bridge/messages.md) | Topics, services, the access-deferral prefix, `TwinStatus` fields |
| [frames and units](docs/bridge/frames_and_units.md) | Units, quaternions, frames, the attitude command/read-back pairing |
| [time synchronization](docs/bridge/time_synchronization.md) | Lockstep and free-run as implemented: barrier, start gate, pacer, sub-stepping |
| [STK notes](docs/bridge/stk_notes.md) | Failed run steps and what to do; STK 12.6 findings from bring-up |
| [extending backends](docs/bridge/extending_backends.md) | Adding a backend or capability without breaking the layering |
| [running campaigns](docs/acav/running_campaigns.md) | Operator manual: checklist, scenario build, one section per experiment |
| [09_live_runs_and_results.md](docs/09_live_runs_and_results.md) | The final runs (folders, commits, record counts, checks), then the earlier live evidence as history |
| [case study](docs/acav/case_study.md) | The ACAV claim, vehicle, scenarios, capture criterion, experiments |
| [ADCS twin](docs/matlab/adcs_twin.md) | The twin: reference and MATLAB, modes, how it runs in each mode, tests |
| [scheduler](docs/acav/scheduler.md) | The two scheduling policies, access deferral, scoring |
| [experiment harness](docs/acav/experiment_harness.md) | Provenance, faults, matrices, run orchestrator, campaigns, pooling rules |
| [analysis](docs/acav/analysis.md) | Reports, figures, E6 latency, coupling analysis |
| [commit_map.md](docs/commit_map.md) | Old -> new commit hashes after the 2026-10-19 message cleanup (trees identical); results and docs written earlier cite old hashes |
| [dev/](docs/dev/) | Engineering history and the open backlog |

There is no C2 document: building and verifying the STK scenario is
covered by C1 section 5 and `src/acav/README.md`.
