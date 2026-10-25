# Column-oriented result matching IAgDrDataSetCollection: one name per
# column and row_count values per column, stored row-major in
# flattened_data (see core/data_providers.py).

std_msgs/Header header

string object_path
string provider_name
string[] column_names
float64[] column_units_scale   # multiply raw STK units by this to reach SI;
                                 # parallel array to column_names
uint32 row_count
float64[] flattened_data        # row-major: row_count x len(column_names)
bool success
string error_message
