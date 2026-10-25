# Cartesian position/velocity of a vehicle at one epoch.
#
# Units: SI (REP-103), metres and metres/second. The backends set STK's
# unit preferences to SI at connect time, so nothing above the backend
# layer handles kilometres.
#
# header.frame_id names the reference frame using the aliases in
# config/frames.yaml, e.g. "icrf", "fixed" (STK Fixed / ECEF). Consumers
# should always read header.frame_id rather than assume a frame.

std_msgs/Header header

string object_path          # STK object path, e.g. "*/Satellite/SSS-1"

geometry_msgs/Point position        # metres, in header.frame_id
geometry_msgs/Vector3 velocity      # metres/second, in header.frame_id

# True if this sample came from STK's real-time propagator look-ahead
# extrapolation rather than from a propagated or written point
# (see docs/bridge/time_synchronization.md).
bool is_extrapolated
