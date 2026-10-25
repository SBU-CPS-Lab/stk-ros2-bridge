# A full trajectory to be written as an STK external ephemeris (.e / CCSDS
# .oem) file and loaded via the StkExternal propagator. Used for batch
# replay of an externally propagated trajectory, not for the live control
# path (use PositionCommand / AttitudeCommand for that).

string object_path
string frame_id                  # must resolve via config/frames.yaml
EphemerisPoint[] points
bool replace_existing             # true: overwrite prior segment for this object
