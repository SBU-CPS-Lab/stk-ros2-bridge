# Instantaneous (boolean) access state at the current sim time -- cheaper
# to stream than the full interval list when a consumer only needs "in
# contact right now".

std_msgs/Header header

string from_object_path
string to_object_path

bool in_access
float64 range_m
float64 range_rate_mps
