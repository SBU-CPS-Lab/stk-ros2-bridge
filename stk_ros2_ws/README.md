# stk_ros2_ws

Bidirectional ROS 2 bridge between **AGI/Ansys STK 12.6 (2023 R1)** and
**ROS 2 Humble**, for a satellite ADCS digital twin developed in
MATLAB/Simulink, plus a worked case study built on top of it. Satellite:
**SSS-1**.

See [`docs/00_architecture.md`](docs/00_architecture.md) for the
bridge's full design and phased build plan, and
[`docs/C1_case_study_spec.md`](docs/C1_case_study_spec.md) for the case
study's. This README covers only the workspace layout and the fastest
path to a build.

## Layout

```
src/
  stk_msgs/     ROS 2 interfaces (msg/srv/action) -- ament_cmake
  stk_bridge/   The bridge itself -- ament_python
  stk_matlab/   MATLAB/Simulink-side client code (not a colcon package)
  acav/         Case study built on the bridge -- see src/acav/README.md
docs/           Design docs, one per topic -- see docs/00_architecture.md
                first for the bridge, docs/C1_case_study_spec.md for the
                case study
```

## Build status

**The bridge (phases 1-6) is complete.** ~13,800 lines across `stk_msgs`
and `stk_bridge` (Python, MATLAB, ROS interfaces, YAML, docs):

| Phase | Contents | Status |
|---|---|---|
| 1 | `stk_msgs` complete; `stk_bridge` package plumbing + exception taxonomy | ✅ |
| 2 | `backends/` (com, engine, connect, mock) | ✅ |
| 3 | `core/` conversions, frames, data providers, rate groups, validation | ✅ |
| 4 | `core/` time sync, sensing, actuation, scenario mgmt, health | ✅ |
| 5 | `interfaces/` + `nodes/` -- bridge is runnable | ✅ |
| 6 | config, launch, tests, MATLAB/Simulink side, docs | ✅ |

**The ACAV case study (C1-C6), built on top of the bridge, is also
implemented and offline-tested.** No campaign has yet run against live
STK -- see [`src/acav/README.md`](src/acav/README.md) for phase-by-phase
status and [`docs/08_deploy_runbook.md`](docs/08_deploy_runbook.md) for
running one.

Test suite (bridge + case study, offline, no ROS or STK needed):

```
cd stk_ros2_ws
python -m pytest src/acav/test src/stk_bridge/test -q
```

740 passed, 3 skipped as of the last C6 increment (the skips are
`builtin_interfaces` import guards for running outside a sourced ROS 2
workspace -- see `src/stk_bridge/test/test_conversions.py`).

Several of this project's design decisions were corrected against a
**live STK 12.6 install** partway through development, not assumed from
documentation alone -- see `docs/06_troubleshooting.md` for the specific
findings (the CBFQuat/ECF attitude frame pairing, the RealTime
propagator's ephemeris-discarding behaviour, `SetPosition`'s true
argument order, and others). `core/validation.py`'s startup self-tests
exist specifically because these were real, not hypothetical, failure
modes.

## Quick start

```powershell
# Windows, PowerShell, with ROS 2 Humble already sourced
cd stk_ros2_ws
colcon build --symlink-install
. install\local_setup.ps1
pip install -r src\stk_bridge\requirements.txt

# Verify the environment before touching real STK:
python src\stk_bridge\scripts\check_environment.py --backend mock
python src\stk_bridge\scripts\check_environment.py --backend com   # once STK is running

# First run -- no STK required:
ros2 launch stk_bridge stk_bridge_mock.launch.py
```

In another terminal:

```powershell
ros2 topic echo /stk/status
ros2 topic echo /stk/sss_1/state
```

Then, against real STK 12.6:

```powershell
ros2 launch stk_bridge stk_bridge.launch.py
```

See `docs/01_installation_windows.md` for the full setup (including the
`agi.stk12` wheel install, which does not come from PyPI) and
`docs/02_matlab_setup.md` for pointing MATLAB's ROS Toolbox at this
workspace's custom messages.

## Running the test suite

```bash
# Both the bridge and the case study, no ROS 2 or STK needed:
python3 -m pytest src/acav/test src/stk_bridge/test -q

# Just the bridge:
cd src/stk_bridge
python3 -m pytest test/ --ignore=test/test_flake8.py --ignore=test/test_pep257.py
```

The full `core/` test suite (conversions, frames, Connect protocol, rate
groups, time sync/lockstep barrier, mock-backend regression tests,
actuation guards, and an end-to-end integration test) runs with **no
ROS 2 installation required** -- `core/` has no ROS import at module
scope, by design (see `stk_bridge/core/__init__.py`). The same is true
of `src/acav/test` except for `scheduler_node.py` and
`campaign_runner.py`'s real subprocess behaviour.

## Documentation map

| Doc | Covers |
|---|---|
| `00_architecture.md` | Bridge design, package structure, phased plan (this project's starting spec) |
| `01_installation_windows.md` | Full Windows/STK/ROS 2 setup |
| `02_matlab_setup.md` | MATLAB ROS Toolbox + `+stkros` setup |
| `03_message_reference.md` | Topic/service/action index |
| `04_frames_and_units.md` | SI units, quaternion convention, the CBFQuat/ECF frame trap |
| `05_time_synchronization.md` | Execution modes, lockstep barrier, the threading model |
| `06_troubleshooting.md` | Every STK-12.6-specific finding from this project's bring-up |
| `07_extending_backends.md` | Adding a backend or capability without breaking layering |
| `08_deploy_runbook.md` | Running an ACAV campaign on the Windows STK machine, and what to send back |
| `C1_case_study_spec.md` | The ACAV case study's claim, scenario contract, and honest limitations |
| `C3_adcs_twin.md` | The ADCS twin: Python reference + MATLAB/Simulink model |
| `C4_scheduler.md` | The target scheduler (kinematic vs. dynamics-aware) |
| `C5_experiment_harness.md` | Provenance, seeded faults, run matrices, the campaign CLI |
| `C6_analysis.md` | Turning a campaign directory into the paper's numbers and figures |

(There is no separate `C2_*.md`: building and verifying the STK
scenario is covered by `C1_case_study_spec.md` and
`src/acav/README.md`.)
