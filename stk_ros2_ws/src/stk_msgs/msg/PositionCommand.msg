# Live, per-sample position command consumed by core/actuation.py.
#
# Realized via STK's Real-time propagator + the `SetPosition` Connect
# command. Same append-only, strictly-increasing-time constraint as
# AttitudeCommand -- see core/actuation.py for the shared monotonicity
# guard applied before either command is issued.
#
# Note: satellites using the Real-time propagator are NOT included in STK
# Coverage analysis (see docs/00_architecture.md Known Gotchas). Use
# EphemerisSegment + StkExternal if coverage must account for this vehicle.

std_msgs/Header header

string object_path

geometry_msgs/Point position     # metres, header.frame_id
geometry_msgs/Vector3 velocity   # metres/second, header.frame_id (optional derivative)
bool velocity_valid
