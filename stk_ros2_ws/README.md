# stk_ros2_ws

Bidirectional ROS 2 bridge between **AGI/Ansys STK 12.6 (2023 R1)** and
**ROS 2 Humble**, for a satellite ADCS digital twin developed in
MATLAB/Simulink. Worked example: satellite **SSS-1**.

See [`docs/00_architecture.md`](docs/00_architecture.md) for the full
design and the phased build plan. This README covers only the workspace
layout and the fastest path to a build.

## Layout

```
src/
  stk_msgs/     ROS 2 interfaces (msg/srv/action) -- ament_cmake
  stk_bridge/   The bridge itself -- ament_python
  stk_matlab/   MATLAB/Simulink-side client code (not a colcon package)
docs/           Design docs, one per topic -- see docs/00_architecture.md first
```

## Build status

| Phase | Contents | Status |
|---|---|---|
| 1 | `stk_msgs` complete; `stk_bridge` package plumbing + exception taxonomy | ✅ |
| 2 | `backends/` (com, engine, connect, mock) | ✅ |
| 3 | `core/` conversions, frames, data providers, rate groups, validation | ✅ |
| 4 | `core/` time sync, sensing, actuation, scenario mgmt, health | ✅ |
| 5 | `interfaces/` + `nodes/` -- bridge is runnable | ✅ |
| 6 | config, launch, tests, MATLAB/Simulink side, docs | ✅ |

**All phases complete.** ~13,800 lines across the workspace (Python,
MATLAB, ROS interfaces, YAML, docs). Test suite: 105 passed, 1 skipped
(the skip is a `builtin_interfaces` import guard for running outside a
sourced ROS 2 workspace -- see `test/test_conversions.py`).

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
cd src/stk_bridge
python3 -m pytest test/ --ignore=test/test_flake8.py --ignore=test/test_pep257.py
```

The full `core/` test suite (conversions, frames, Connect protocol, rate
groups, time sync/lockstep barrier, mock-backend regression tests,
actuation guards, and an end-to-end integration test) runs with **no
ROS 2 installation required** -- `core/` has no ROS import at module
scope, by design (see `stk_bridge/core/__init__.py`).

## Documentation map

| Doc | Covers |
|---|---|
| `00_architecture.md` | Design, package structure, phased plan (this project's starting spec) |
| `01_installation_windows.md` | Full Windows/STK/ROS 2 setup |
| `02_matlab_setup.md` | MATLAB ROS Toolbox + `+stkros` setup |
| `03_message_reference.md` | Topic/service/action index |
| `04_frames_and_units.md` | SI units, quaternion convention, the CBFQuat/ECF frame trap |
| `05_time_synchronization.md` | Execution modes, lockstep barrier, the threading model |
| `06_troubleshooting.md` | Every STK-12.6-specific finding from this project's bring-up |
| `07_extending_backends.md` | Adding a backend or capability without breaking layering |
