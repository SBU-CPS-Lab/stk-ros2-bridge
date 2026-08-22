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
| 1 | `stk_msgs` complete; `stk_bridge` package plumbing + exception taxonomy | ✅ this delivery |
| 2 | `backends/` (com, engine, connect, mock) | not yet generated |
| 3 | `core/` conversions, frames, data providers, rate groups | not yet generated |
| 4 | `core/` time sync, sensing, actuation | not yet generated |
| 5 | `interfaces/` + `nodes/` -- bridge becomes runnable | not yet generated |
| 6 | config, launch, tests, MATLAB side, docs | not yet generated |

## Quick start (once all phases land)

```powershell
# Windows, PowerShell, with ROS 2 Humble already sourced
cd stk_ros2_ws
colcon build --symlink-install
. install\local_setup.ps1
pip install -r src\stk_bridge\requirements.txt
python src\stk_bridge\scripts\check_environment.py
ros2 launch stk_bridge stk_bridge_mock.launch.py   # no STK required
```

See `docs/01_installation_windows.md` (Phase 6) for the full STK-attached
setup, and `docs/02_matlab_setup.md` for pointing MATLAB's ROS Toolbox at
this workspace's custom messages.

## Right now (Phase 1 only)

The message package builds and is introspectable on its own:

```powershell
colcon build --packages-select stk_msgs
. install\local_setup.ps1
ros2 interface show stk_msgs/msg/AttitudeCommand
ros2 interface show stk_msgs/srv/SetPropagator
```

`stk_bridge` will build (it's valid, empty Python packages plus
`exceptions.py`) but its console-script entry points are not yet
implemented -- `ros2 run stk_bridge bridge_node` will fail with an import
error until Phase 5.
