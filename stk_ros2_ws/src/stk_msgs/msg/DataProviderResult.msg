# Column-oriented result matching IAgDrDataSetCollection's shape: one name
# per column, one flattened float64 array per column, all the same length
# (row_count). This mirrors ToNumpyArray() (STK 12.6+, see
# core/data_providers.py) more directly than a row-of-structs encoding
# would, and avoids re-deriving column semantics per caller.

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
