# One sample of a batch ephemeris trajectory, for the StkExternal propagator
# path (core/ephemeris_writer.py). Distinct from OrbitState: this is
# *commanded* data going STK-ward for batch replay, not sensed telemetry.

builtin_interfaces/Time time
geometry_msgs/Point position     # metres, ICRF unless overridden by segment
geometry_msgs/Vector3 velocity   # metres/second
