# Installation (Windows, T1 single-host topology)

Covers the default deployment: STK 12.6, the bridge, and ROS 2 Humble
all on one Windows machine. For the split topologies (T2/T3), see
[00_architecture.md](00_architecture.md) section 2 and
[06_troubleshooting.md](06_troubleshooting.md)'s network section.

## 1. Prerequisites

| Component | Version | Notes |
|---|---|---|
| STK | 12.6 (2023 R1) | With the **Integration** capability licensed. |
| Windows | 10 or 11 | ROS 2 Humble's official Windows binaries target this. |
| Python | 3.8+ | Confirmed via `check_environment.py`; matches ROS 2 Humble's Windows build. |
| ROS 2 | Humble | [docs.ros.org](https://docs.ros.org) Windows install instructions. |
| MATLAB | R2023b or R2024a | For ROS 2 Humble support in ROS Toolbox. See [02_matlab_setup.md](02_matlab_setup.md). |
| Visual Studio Build Tools | "Desktop development with C++" workload | Required by `ros2genmsg` on the MATLAB side, and useful generally. |

## 2. Install ROS 2 Humble

Follow the official Windows install guide. After installation, confirm:

```powershell
ros2 doctor
```

## 3. Install the `agi.stk12` Python API

The wheel ships **inside your STK install**, not on PyPI:

```powershell
pip install "C:\Program Files\AGI\STK 12\bin\AgPythonAPI\agi.stk12-12.6.0-py3-none-any.whl"
```

Install into the **same Python environment ROS 2 Humble's tooling uses**
(see `stk_bridge/requirements.txt`'s note on why this must not be a
separate venv). Adjust the wheel filename to whatever is actually
present in that directory on your machine.

Install the rest of the bridge's Python dependencies:

```powershell
pip install -r src\stk_bridge\requirements.txt
```

## 4. Build the workspace

```powershell
cd stk_ros2_ws
colcon build --symlink-install
. install\local_setup.ps1
```

## 5. Verify the environment

```powershell
python src\stk_bridge\scripts\check_environment.py --backend mock
```

This should pass everything except the (harmless, expected) STK-specific
checks if STK isn't running. Then, with STK 12.6 open:

```powershell
python src\stk_bridge\scripts\check_environment.py --backend com
```

**Do not proceed past a failing convention-validation check.** See
[06_troubleshooting.md](06_troubleshooting.md) for what each failure
means; several were found and fixed against a real STK 12.6 install
during this project's development, and the fixes are baked into the
current `core/` modules -- but a *different* 12.6 install (different
patch level, different licensed modules) may still surface something
new. `check_environment.py` exists precisely so that surfaces as an
actionable message here, not as a mysterious failure three phases later.

## 6. First run: the mock backend

Before pointing anything at real STK, confirm the ROS-facing half of the
bridge works:

```powershell
ros2 launch stk_bridge stk_bridge_mock.launch.py
```

In another terminal:

```powershell
ros2 topic echo /stk/status
ros2 topic echo /stk/sss_1/state
```

You should see `state: 1` (CONNECTED) on `/stk/status` and periodic
`SatelliteState` messages. If this doesn't work, the problem is in the
bridge's ROS plumbing, not in STK -- debug here first.

## 7. Run against real STK

```powershell
ros2 launch stk_bridge stk_bridge.launch.py
```

STK Desktop should launch (or be attached to, if already running --
controlled by `backend.com.attach_to_existing` in
`config/bridge_params.yaml`). Watch the log for the convention
validation summary; it must pass before telemetry starts publishing.

## 8. MATLAB side

See [02_matlab_setup.md](02_matlab_setup.md).
