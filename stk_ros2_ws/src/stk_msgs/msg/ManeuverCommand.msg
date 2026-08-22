# Command consumed by the ExecuteManeuver action (Astrogator segment
# mutation + RunMCS). Requires the Astrogator capability to be licensed --
# core/scenario_manager.py checks this at startup and the action server
# rejects goals with a clear error if it is not available, rather than
# failing deep inside a Connect call.

string object_path

uint8 IMPULSIVE = 0
uint8 FINITE = 1
uint8 burn_type

string frame_id
ImpulsiveBurn impulsive     # valid if burn_type == IMPULSIVE
FiniteBurn finite            # valid if burn_type == FINITE

builtin_interfaces/Time execute_at
