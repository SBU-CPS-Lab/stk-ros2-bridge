# Installation (Windows, single host)

Everything runs on one Windows machine: STK 12.6 Desktop, ROS 2 Humble,
the bridge, MATLAB R2023b and the case-study nodes (topology T1,
[00_architecture.md](00_architecture.md) section 3). MATLAB is covered
in [02_matlab_setup.md](02_matlab_setup.md); running experiments in
[08_deploy_runbook.md](08_deploy_runbook.md).

## 1. Prerequisites

| Component | Version | Notes |
|---|---|---|
| Windows | 10 or 11 | The recorded runs used Windows 10 Pro on a 2-core Intel Core i5-6300U. |
| STK | 12.6 (2023 R1) Desktop | Integration and real-time propagator capabilities (checked at bridge startup, `core/scenario_manager.py` `_check_capabilities`). |
| ROS 2 | Humble, Windows binaries | Official Windows install. |
| Python | the one ROS 2's tooling uses | `agi.stk12` must be installed into the same interpreter (section 3). |
| MATLAB | R2023b with ROS Toolbox and Simulink | See [02_matlab_setup.md](02_matlab_setup.md). |
| Visual Studio Build Tools | "Desktop development with C++" | Needed by `colcon` and by MATLAB's `ros2genmsg`. |

## 2. ROS 2 Humble

Follow the official Windows install guide, then check:

```powershell
ros2 doctor
```

## 3. The `agi.stk12` Python API

The wheel ships inside the STK install, not on PyPI:

```powershell
pip install "C:\Program Files\AGI\STK 12\bin\AgPythonAPI\agi.stk12-12.6.0-py3-none-any.whl"
pip install -r src\stk_bridge\requirements.txt
```

Use the wheel file name actually present in that folder. Install into
the interpreter ROS 2 uses (see `src/stk_bridge/requirements.txt` for
why not a separate virtual environment).

## 4. Build the workspace

Source ROS 2 first (otherwise `colcon` is not found), build, then source
the workspace:

```powershell
cd <path-to>\stk_ros2_ws
C:\dev\ros2-windows\local_setup.ps1     # your ROS 2 install
colcon build --symlink-install
. install\local_setup.ps1               # this workspace
```

After any code change, rebuild (`colcon build --symlink-install`) and
source `install\local_setup.ps1` again before the next `ros2 run`:
`ros2 run` executes the installed package, so a new module or entry
point is not seen until then. After a change to a `.msg` file, also
regenerate MATLAB's message types
([02_matlab_setup.md](02_matlab_setup.md) section 3).

## 5. Run the offline tests

No ROS, STK or MATLAB needed:

```powershell
$env:PYTHONDONTWRITEBYTECODE = "1"
python -m pytest src/acav/test src/stk_bridge/test -q -p no:cacheprovider -rs
```

Four tests are skipped outside a sourced ROS 2 workspace (rclpy,
builtin_interfaces, ament_flake8, ament_pep257); they are not verified
by this command.

## 6. Check the environment

```powershell
python src\stk_bridge\scripts\check_environment.py --backend mock
python src\stk_bridge\scripts\check_environment.py --backend com   # with STK open
```

The COM check attaches to the STK Desktop that is already running; it
does not start one (since 24b5eeb the COM backend only attaches unless
`backend.com.start_stk` is set). Start STK Desktop yourself first.

**Do not go on past a failing convention-validation check.** The
bridge's startup validation (`core/validation.py`) proves units,
quaternion order and the attitude command/read-back pairing on this
install; see [06_troubleshooting.md](06_troubleshooting.md) for what
each failure means.

## 7. First run without STK: the mock backend

```powershell
ros2 launch stk_bridge stk_bridge_mock.launch.py
```

In another terminal:

```powershell
ros2 topic echo /stk/status
ros2 topic echo /stk/sss_1/state
```

`state: 1` (CONNECTED) on `/stk/status` and periodic `SatelliteState`
messages mean the ROS side works. The mock backend has no STK physics;
nothing it produces is reportable.

## 8. Against STK

Start STK Desktop yourself, then:

```powershell
ros2 launch stk_bridge stk_bridge.launch.py
```

The bridge attaches to that STK, runs convention validation (each phase
is logged with its duration, `core/startup_phases.py`), and publishes
telemetry only once validation has passed. For the case study, use the
run orchestrator instead ([08_deploy_runbook.md](08_deploy_runbook.md)).
