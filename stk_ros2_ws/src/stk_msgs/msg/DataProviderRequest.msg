# Generic escape hatch for arbitrary STK Data Provider queries not covered
# by a typed message above. Prefer the typed topics/services wherever they
# exist; use this only for exploratory work or genuinely one-off queries.
# See srv/QueryDataProvider.srv for the request/response pair that wraps
# this message.

string object_path
string provider_name        # e.g. "Cartesian Position", "LLA State"
string group_name            # STK data provider group, if applicable, else ""
string[] element_names        # subset of elements to return; empty = all
builtin_interfaces/Time start_time
builtin_interfaces/Time stop_time
float64 step_s
