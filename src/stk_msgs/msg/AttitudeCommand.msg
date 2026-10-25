# Live, per-sample attitude command consumed by core/actuation.py.
#
# Realized in STK through the real-time attitude profile: the bridge
# activates it once per object (`SetAttitude Standard`) and then appends
# one sample per message with `AddAttitude CBFQuat` (Earth-fixed) or
# `AddAttitude Quat` (inertial), selected by use_inertial_frame. Samples
# are appended in place; no ephemeris file is written.
#
# Samples must arrive with strictly increasing stamps per object_path.
# STK rejects a sample that is not newer than the last accepted one;
# core/actuation.py checks this before sending and counts rejected samples
# in BridgeStatus.dropped_attitude_samples.

std_msgs/Header header

string object_path

geometry_msgs/Quaternion orientation   # (x, y, z, w), body <- reference

bool use_inertial_frame
# false (default): orientation is in Central-Body-Fixed axes -> AddAttitude
#   CBFQuat. true: orientation is inertial (ICRF/J2000, per header.frame_id)
#   -> AddAttitude Quat.
