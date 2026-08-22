# Describes one object in the current STK scenario. Used by
# scenario_manager.py to publish the object inventory and by
# CreateObject/DeleteObject services.

string object_path
string object_type       # "Satellite" | "Facility" | "Sensor" | "GroundStation" | ...
string propagator_type    # "TwoBody" | "HPOP" | "J4Perturbation" | "RealTime" | "External" | ""
bool exists
