# Guidance target for a closed-loop ADCS twin.
#
# Published by an application-level scheduler (e.g. the ACAV case
# study's acav_scheduler node) on an application topic such as
# /acav/setpoint -- this is NOT part of the bridge's own command
# surface. Contrast stk_msgs/PositionCommand, which IS realized via
# STK's SetPosition on a named object; nothing here is ever sent to
# STK directly. The twin subscribes to this to know WHERE to point;
# the guidance law that turns a target position into a commanded body
# attitude quaternion (accounting for the vehicle's own boresight axis
# and a roll reference such as the velocity direction) lives entirely
# in the twin, not here. Keeping this message data-only -- a position,
# not an attitude -- is what lets the scheduler differ from a
# kinematic to a dynamics-aware policy purely by WHEN it switches
# targets, without duplicating the twin's own guidance math.

std_msgs/Header header

# Scenario target id, e.g. "T3"; "" if has_target is false.
string target_id

# Metres, inertial frame named by header.frame_id (e.g. "icrf").
geometry_msgs/Point target_position_m

# false: hold / point at nadir. true: point the boresight at
# target_position_m.
bool has_target
