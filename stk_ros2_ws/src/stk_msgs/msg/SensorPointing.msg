# Boresight pointing of a sensor attached to a vehicle.

std_msgs/Header header

string sensor_path             # e.g. "*/Satellite/SSS-1/Sensor/StarTracker1"

geometry_msgs/Vector3 boresight_unit_vector   # unit vector, header.frame_id
float64 azimuth_rad
float64 elevation_rad
