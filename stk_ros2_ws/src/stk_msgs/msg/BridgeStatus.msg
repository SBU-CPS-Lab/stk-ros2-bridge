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

# E6's latency breakdown, mean over the second since the last publish
# (BridgeStatus is always 1 Hz -- see this message's own top comment).
# step_samples is how many step-loop iterations fed that mean; a
# consumer must check it before trusting a phase's number, because 0.0
# means EITHER "fast" or "never measured this run" -- free-run has no
# barrier wait at all, and a run that has not started stepping yet has
# neither. See stk_bridge/core/step_timing.py for why this is windowed
# rather than a lifetime average.
float64 mean_sense_ms
float64 mean_actuate_ms
float64 mean_barrier_wait_ms
uint32 step_samples
