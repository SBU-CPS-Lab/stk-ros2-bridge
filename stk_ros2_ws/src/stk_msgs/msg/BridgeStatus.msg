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

# The attitude read-back provider sensing actually uses, and whether it
# was forced by sensing.force_attitude_provider (E1's treatment) rather
# than chosen by startup validation. Empty until sensing is configured.
# A consumer recording provenance must take these from here, not from
# its own launch parameters: a forced provider left in a bridge YAML
# would otherwise turn a nominal run into an E1 treatment unrecorded.
string attitude_provider
bool attitude_provider_forced
