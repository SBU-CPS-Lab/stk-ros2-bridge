# DT-ADCS

Hybrid digital twin for the ADCS of the SSS-1 CubeSat: a bidirectional
ROS 2 bridge to AGI/Ansys STK, and a worked case study on top of it
showing that mission planning which assumes instantaneous, perfect
pointing over-promises imaging capacity — measurable only in a closed,
time-synchronized loop.

## Where the work is

```
stk_ros2_ws/    The ROS 2 workspace: the STK<->ROS 2 bridge, the ACAV
                case study built on it, and all project documentation.
                Start at stk_ros2_ws/README.md.
Physical Models/  Standalone Simulink sensor models (GPS, gyroscope,
                  magnetometer) for the SSS-1 platform. Predates the
                  bridge/case-study work above and is not currently
                  wired into it.
```

See [`stk_ros2_ws/README.md`](stk_ros2_ws/README.md) for the bridge's
build status, the case study's phase status, how to run the offline
test suite, and the full documentation map.
