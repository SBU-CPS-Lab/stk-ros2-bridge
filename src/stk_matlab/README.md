# stk_matlab

MATLAB support for the bridge: a client library and a Simulink ADCS
digital twin. Requires MATLAB R2023b with ROS Toolbox (and Simulink for
the twin); see [MATLAB client](../../docs/matlab/client.md).

| Folder / file | Contents |
|---|---|
| `+stkros/` | Client library: `BridgeClient`, `TelemetrySubscriber`, `CommandPublisher`, `ScenarioService`, `frames`. |
| `+adcs/` | The attitude-control twin: `twin_step`, `simulink_step`, rigid body, wheels, guidance, mission modes, code fingerprint. |
| `simulink/` | Model builders: `build_acav_twin` (the ACAV twin), `build_twin_template` (a generic template), bus definitions. |
| `examples/` | `example_01_read_telemetry`, `example_02_closed_loop_attitude`, `example_03_lockstep_stepping`. |
| `setup_stk_ros.m` | Path setup, `ros2genmsg` for `stk_msgs`, network check. |
| `run_twin_session.m` | Runs the twin as a session (used by the automated `matlab -batch` launch). |
| `test_*.m` | MATLAB tests, run from this folder after `setup_stk_ros`. |

Start with:

```matlab
cd src/stk_matlab
setup_stk_ros
test_acav_twin
```

After any `.msg` change run `setup_stk_ros('Rebuild', true)`. The twin's
design is in [ADCS twin](../../docs/matlab/adcs_twin.md). Known issues:
`build_twin_template` contains a placeholder that must be replaced and
has not been exercised in MATLAB.
