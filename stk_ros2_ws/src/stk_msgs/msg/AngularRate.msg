# Body angular velocity of a vehicle.
#
# Units: rad/s (REP-103). STK's AddAttitude AngVels Connect command and the
# Object Model both natively use degrees/second -- converted once at the
# backend boundary.

std_msgs/Header header

string object_path

geometry_msgs/Vector3 angular_velocity   # rad/s, body frame
