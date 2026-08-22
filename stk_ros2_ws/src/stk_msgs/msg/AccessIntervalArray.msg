# Full result of an access computation between two objects over the
# requested time span. Returned by the ComputeAccess service and published
# on the "slow" rate group for standing access relationships (e.g. ground
# station contacts).

std_msgs/Header header

string from_object_path
string to_object_path

AccessInterval[] intervals
