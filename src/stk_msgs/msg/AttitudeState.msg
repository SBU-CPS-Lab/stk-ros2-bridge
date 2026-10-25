# Attitude quaternion of a vehicle body frame relative to a reference frame.
#
# Quaternion component order is (x, y, z, w), matching both
# geometry_msgs/Quaternion (REP-103) and STK's Object Model quaternion
# element order (Q1..Q3 vector part, Q4 scalar part). core/validation.py
# checks this convention at bridge startup.

std_msgs/Header header

string object_path

geometry_msgs/Quaternion orientation   # body <- header.frame_id

bool is_extrapolated
