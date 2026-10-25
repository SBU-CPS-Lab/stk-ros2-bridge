# Guidance target for a closed-loop ADCS twin.
#
# Published by an application-level scheduler (for example the ACAV
# case study's acav_scheduler on /acav/setpoint). It is not part of the
# bridge's command surface and is never sent to STK. The twin turns the
# target position into a commanded attitude with its own guidance law;
# the message carries a position, not an attitude, so schedulers differ
# only in when they switch targets.

std_msgs/Header header

# Scenario target id, e.g. "T3"; "" if has_target is false.
string target_id

# Metres, inertial frame named by header.frame_id (e.g. "icrf").
geometry_msgs/Point target_position_m

# false: hold / point at nadir. true: point the boresight at
# target_position_m.
bool has_target
