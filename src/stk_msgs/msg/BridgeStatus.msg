# Top-level health/heartbeat message for the bridge node. Published at a
# fixed 1 Hz regardless of execution mode so a watchdog can distinguish
# "bridge is dead" from "sim is paused".

std_msgs/Header header

uint8 STARTING = 0
uint8 CONNECTED = 1
uint8 DEGRADED = 2      # backend reachable but some capability unavailable
uint8 DISCONNECTED = 3
uint8 ERROR = 4
uint8 state

string backend_type              # "com" | "engine" | "connect" | "mock"
string stk_version_string        # as reported by the backend, e.g. "12.6.0"

uint32 dropped_attitude_samples  # see AttitudeCommand.msg monotonicity note
uint32 dropped_position_samples
uint32 command_queue_depth
uint32 command_queue_capacity

float64 last_backend_round_trip_ms
string last_error

# Step-loop latency breakdown: mean of each phase over the window since
# the previous BridgeStatus (core/step_timing.py). step_samples is the
# number of step iterations in that window. A phase mean of 0.0 can mean
# "fast" or "not measured" (free-run has no barrier wait), so consumers
# should check step_samples before using the means.
float64 mean_sense_ms
float64 mean_actuate_ms
float64 mean_barrier_wait_ms
uint32 step_samples

# The attitude read-back provider that sensing uses, and whether it was
# forced by sensing.force_attitude_provider instead of being selected by
# startup validation. Empty until sensing is configured. Consumers that
# record provenance should take the provider from here rather than from
# their own launch parameters.
string attitude_provider
bool attitude_provider_forced
