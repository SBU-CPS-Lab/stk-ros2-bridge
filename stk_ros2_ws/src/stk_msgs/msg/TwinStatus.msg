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

# Reaction wheel state, for E5's fault campaign and for the run-level
# quantities acav/metrics.py's RunResult reports.
#
# Momentum is published in its physical unit rather than as a fraction
# of capacity: normalising it here would bake a configuration value into
# telemetry, and a consumer that disagreed with the twin about the
# capacity would then silently disagree about how close to saturation
# the vehicle was. The fraction is derived by whoever has the spec.
geometry_msgs/Vector3 wheel_momentum_nms

# True on any step where a wheel clamped, in torque or in momentum.
bool wheels_saturated

# Cumulative integral of |tau . omega| since the run began, joules --
# a proxy for slew cost. Accumulated by the twin rather than integrated
# by a consumer, because the twin is the only party that holds both the
# delivered torque and the body rate at the same instant, and a running
# total survives a dropped telemetry sample where a consumer-side
# integration would silently lose that step's contribution.
float64 wheel_energy_j
