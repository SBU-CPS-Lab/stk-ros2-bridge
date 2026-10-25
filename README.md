# stk-ros2-bridge

A ROS 2 bridge for **AGI/Ansys STK 12.6**. It exposes an STK scenario
(orbit, attitude, access, sensing) as ROS 2 topics, services and
actions, and acts as the simulation clock authority so that external
models can be coupled to STK deterministically (**lockstep**, a barrier
per step) or against the wall clock (**free-run**, with a real-time
factor).

Three parts, usable independently:

| Package | What it is |
|---|---|
| `src/stk_bridge` | The bridge: STK backends (COM, Connect, mock), time, sensing, actuation, startup validation of frames, units and quaternion conventions. |
| `src/stk_msgs` | The ROS 2 message, service and action definitions. |
| `src/stk_matlab` | A MATLAB client (`+stkros`) for the bridge, and a Simulink **ADCS digital twin** of a small satellite built on it (`+adcs`). |
| `src/acav` | **ACAV**, one case study built on all of the above: does an imaging schedule that is feasible on geometry alone survive the attitude dynamics? Scenarios, schedulers, a run orchestrator, campaigns and analysis. |

A mock backend implements the same interface as the STK backends, so
the bridge and the case-study tooling run and are tested without STK,
MATLAB or ROS.

## Quick start

Requirements for the full setup: Windows host with STK 12.6, ROS 2
Humble, Python with `agi.stk12`, MATLAB R2023b with ROS Toolbox and
Simulink (see [installation](docs/installation.md)).

```powershell
colcon build --symlink-install
. install\local_setup.ps1
```

Offline tests (no ROS, STK or MATLAB needed):

```bash
python -m pytest src/acav/test src/stk_bridge/test -q -p no:cacheprovider
```

Run the bridge without STK, on the mock backend:

```bash
ros2 launch stk_bridge stk_bridge_mock.launch.py
```

After a code change, repeat `colcon build --symlink-install` and source
the install before the next `ros2 run`.

## Documentation

The [documentation index](docs/README.md) lists everything. Good first
reads: [architecture](docs/bridge/architecture.md),
[time synchronization](docs/bridge/time_synchronization.md),
[messages](docs/bridge/messages.md), and for the case study the
[ACAV case study](docs/acav/case_study.md). Contributors: the
[developer guide](docs/developer_guide.md).

## Citing

If you use this software in research, please cite the accompanying
paper (reference to be added) and this repository.

## License

MIT, see [LICENSE](LICENSE).
