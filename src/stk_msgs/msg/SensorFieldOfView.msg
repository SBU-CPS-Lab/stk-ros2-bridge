# Static geometric description of a sensor's field of view. Published at
# bridge startup rather than on a rate group, since it rarely changes.

string sensor_path

uint8 SIMPLE_CONIC = 0
uint8 RECTANGULAR = 1
uint8 CUSTOM = 2

uint8 fov_type
float64 cone_half_angle_rad     # valid if fov_type == SIMPLE_CONIC
float64 horizontal_half_angle_rad  # valid if fov_type == RECTANGULAR
float64 vertical_half_angle_rad    # valid if fov_type == RECTANGULAR
