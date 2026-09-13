# Diagnostic status streamed by a closed-loop ADCS twin back to whatever
# is scheduling it (e.g. the ACAV case study's acav_scheduler node) on
# an application topic such as /acav/twin_status.
#
# boresight_error_rad is the twin's OWN internally-computed angle
# between its achieved boresight direction and the line of sight to
# its current commanded target (see stk_msgs/TargetSetpoint) -- the
# same quantity the twin's mission-mode chart uses for its
# SETTLE -> IMAGE transition. A consumer scoring captures should
# prefer this over re-deriving the angle independently from raw
# telemetry: the twin already has the freshest target + own-state pair
# at the instant it computes this, and a second, independently-written
# implementation of the same angle is exactly the kind of duplication
# that has previously made a result measure a discrepancy between two
# models instead of the effect under study (see acav/metrics.py).

std_msgs/Header header

# Stateflow mode name: NADIR | SLEW | SETTLE | IMAGE | SAFE.
string mode

float64 boresight_error_rad
