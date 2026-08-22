# Cartesian position/velocity of a vehicle at one epoch.
#
# Units: SI (REP-103) -- metres, metres/second. STK's native Data Provider
# units are km / km/s; the km->m conversion happens once, inside
# stk_bridge/core/conversions.py, at the backend boundary. Nothing above the
# backend layer should ever see kilometres.
#
# header.frame_id names the STK reference frame this state is expressed in,
# using the aliases defined in config/frames.yaml, e.g. "icrf", "j2000",
# "itrf" (STK "Fixed"/ECEF), "lvlh". Do not assume a fixed frame -- always
# read header.frame_id.

std_msgs/Header header

string object_path          # STK object path, e.g. "*/Satellite/SSS-1"

geometry_msgs/Point position        # metres, in header.frame_id
geometry_msgs/Vector3 velocity      # metres/second, in header.frame_id

# True if this sample came from STK's Real-time propagator look-ahead
# extrapolation rather than a directly propagated/measured point. Mirrors
# the Look-Ahead/Look-Behind semantics described in
# docs/05_time_synchronization.md.
bool is_extrapolated
