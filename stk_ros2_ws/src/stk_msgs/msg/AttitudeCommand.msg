# Live, per-sample attitude command consumed by core/actuation.py.
#
# Realized in STK via the Real-Time Standard attitude type: the bridge
# issues `SetAttitude Standard` once to activate the real-time profile (if
# not already active), then streams samples with `AddAttitude CBFQuat` (or
# `AddAttitude Quat` for inertial-referenced input, selected by
# use_inertial_frame below). This is STK's native per-sample append
# mechanism for attitude -- the direct analogue of SetPosition for orbit
# state. There is NO file buffering/rewrite involved.
#
# IMPORTANT constraint inherited from the underlying Connect command:
# samples MUST arrive with strictly increasing stamp values per object_path.
# A sample with stamp <= the last accepted stamp is silently dropped by STK
# (not NACKed) -- core/actuation.py logs and counts these as
# BridgeStatus.dropped_attitude_samples rather than relying on STK to
# report the problem.

std_msgs/Header header

string object_path

geometry_msgs/Quaternion orientation   # (x, y, z, w), body <- reference

bool use_inertial_frame
# false (default): orientation is in Central-Body-Fixed axes -> AddAttitude
#   CBFQuat. true: orientation is inertial (ICRF/J2000, per header.frame_id)
#   -> AddAttitude Quat.
