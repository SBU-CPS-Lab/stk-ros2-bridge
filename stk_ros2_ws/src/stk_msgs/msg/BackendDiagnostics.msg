# Lower-level, lower-rate diagnostic detail than BridgeStatus. Published on
# the "slow" rate group and available on-demand for troubleshooting; not
# meant to gate any control logic.

std_msgs/Header header

string backend_type
bool licensed_integration
bool licensed_astrogator
bool licensed_realtime_propagator
bool licensed_engine_on_linux

string python_version
string agi_stk_wheel_version   # parsed from bin/AgPythonAPI, "unknown" if not found

float64 mean_data_provider_latency_ms
float64 mean_command_latency_ms
uint32 com_apartment_thread_id   # diagnostic only -- see core/com_thread.py
