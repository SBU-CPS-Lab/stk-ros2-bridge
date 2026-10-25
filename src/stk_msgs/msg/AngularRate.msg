# Body angular velocity of a vehicle.
#
# Units: rad/s (REP-103). STK reports body rates in its own angle-rate
# unit; the backend measures that scale at startup and converts at the
# backend boundary.

std_msgs/Header header

string object_path

geometry_msgs/Vector3 angular_velocity   # rad/s, body frame
