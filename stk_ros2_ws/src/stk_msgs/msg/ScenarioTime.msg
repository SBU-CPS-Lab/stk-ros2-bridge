# Published alongside /clock by nodes/clock_node.py -- richer than
# rosgraph_msgs/Clock because it also carries STK scenario epoch context
# and the bridge's current execution mode, so consumers can distinguish
# "simulation is paused" from "simulation is behind".

builtin_interfaces/Time sim_time
builtin_interfaces/Time scenario_epoch     # STK scenario's analysis start

uint8 FREE_RUN = 0
uint8 PACED = 1
uint8 LOCKSTEP = 2
uint8 EXTERNAL = 3
uint8 execution_mode

float64 real_time_factor    # 1.0 == wall-clock real-time; only meaningful
                             # outside LOCKSTEP
uint32 step_index           # monotonically increasing in LOCKSTEP mode
bool paused
