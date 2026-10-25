# Command consumed by the ExecuteManeuver action (Astrogator segment
# mutation + RunMCS). Requires an Astrogator licence; the action server
# rejects the goal with an explicit message when it is not available.

string object_path

uint8 IMPULSIVE = 0
uint8 FINITE = 1
uint8 burn_type

string frame_id
ImpulsiveBurn impulsive     # valid if burn_type == IMPULSIVE
FiniteBurn finite            # valid if burn_type == FINITE

builtin_interfaces/Time execute_at
