# Aggregate, single-message snapshot of a satellite's full observable state.
#
# Published on the "fast" rate group (config/sensing_pipeline.yaml) as a
# convenience for consumers that want one synchronized sample rather than
# correlating OrbitState/AttitudeState/AngularRate by timestamp themselves.
# The individual per-quantity topics remain the source of truth for anyone
# who needs per-field QoS or a different rate.

std_msgs/Header header

string object_path

OrbitState orbit
AttitudeState attitude
AngularRate angular_rate
EclipseState eclipse
