# Status streamed by a closed-loop ADCS twin to its scheduler (for
# example the ACAV case study's acav_scheduler on /acav/twin_status).
#
# boresight_error_rad is the twin's own angle between its achieved
# boresight and the line of sight to the commanded target
# (stk_msgs/TargetSetpoint), the quantity its mode logic uses for the
# SETTLE -> IMAGE transition. Consumers scoring captures use this value
# instead of recomputing the angle, so that one implementation defines it.

std_msgs/Header header

# Mission mode: NADIR | SLEW | SETTLE | IMAGE | SAFE.
string mode

float64 boresight_error_rad

# Reaction wheel state. Momentum is published in N m s rather than as a
# fraction of capacity, so telemetry does not depend on a configuration
# value; consumers derive the fraction from their own specification.
geometry_msgs/Vector3 wheel_momentum_nms

# True on any step where a wheel clamped, in torque or in momentum.
bool wheels_saturated

# Cumulative integral of |tau . omega| since the run began, in joules, as
# a proxy for slew cost. The twin accumulates it because it holds the
# delivered torque and the body rate at the same instant, and a running
# total is unaffected by a dropped telemetry sample.
float64 wheel_energy_j

# Events between two statuses. In free-run the twin may integrate
# several steps per published status, so short events would otherwise be
# missed; in lockstep every step is published and these fields add
# nothing.
#
# Cumulative count of steps at which wheels_saturated rose from false to
# true, counted by the twin at every integration step.
uint32 saturation_events

# The largest boresight_error_rad over every step since the previous
# TwinStatus, this one's included; equals boresight_error_rad when one
# step was integrated. The capture hold window is judged on it.
float64 max_boresight_error_since_last_rad
