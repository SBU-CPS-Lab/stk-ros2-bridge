# Live, per-sample position command consumed by core/actuation.py.
#
# Realized via STK's Real-time propagator + the `SetPosition` Connect
# command. Same append-only, strictly-increasing-time constraint as
# AttitudeCommand -- see core/actuation.py for the shared monotonicity
# guard applied before either command is issued.
#
# Satellites on the real-time propagator are excluded from STK Coverage
# analysis. Use EphemerisSegment and the StkExternal propagator when
# coverage must include the vehicle.

std_msgs/Header header

string object_path

geometry_msgs/Point position     # metres, header.frame_id
geometry_msgs/Vector3 velocity   # metres/second, header.frame_id (optional derivative)
bool velocity_valid
