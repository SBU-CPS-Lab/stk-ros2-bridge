cmake_minimum_required(VERSION 3.8)
project(stk_msgs)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# MATLAB does not reuse this build: ros2genmsg compiles the same interface
# files against MATLAB's own client libraries (on Windows it needs the MSVC
# "Desktop development with C++" workload; see docs/installation.md). These
# directories are the single source of truth for both toolchains.

set(msg_files
  "msg/OrbitState.msg"
  "msg/AttitudeState.msg"
  "msg/AngularRate.msg"
  "msg/LLAState.msg"
  "msg/SatelliteState.msg"
  "msg/EclipseState.msg"
  "msg/SensorPointing.msg"
  "msg/SensorFieldOfView.msg"
  "msg/AccessInterval.msg"
  "msg/AccessIntervalArray.msg"
  "msg/AccessStatus.msg"
  "msg/LinkBudget.msg"
  "msg/LinkBudgetArray.msg"
  "msg/GroundStationAER.msg"
  "msg/EphemerisPoint.msg"
  "msg/EphemerisSegment.msg"
  "msg/AttitudeCommand.msg"
  "msg/PositionCommand.msg"
  "msg/ManeuverCommand.msg"
  "msg/ImpulsiveBurn.msg"
  "msg/FiniteBurn.msg"
  "msg/ScenarioTime.msg"
  "msg/BridgeStatus.msg"
  "msg/BackendDiagnostics.msg"
  "msg/DataProviderRequest.msg"
  "msg/DataProviderResult.msg"
  "msg/ObjectDescriptor.msg"
  "msg/TargetSetpoint.msg"
  "msg/TwinStatus.msg"
)

set(srv_files
  "srv/LoadScenario.srv"
  "srv/SaveScenario.srv"
  "srv/ResetScenario.srv"
  "srv/CreateObject.srv"
  "srv/DeleteObject.srv"
  "srv/SetPropagator.srv"
  "srv/QueryDataProvider.srv"
  "srv/ComputeAccess.srv"
  "srv/GetLinkBudget.srv"
  "srv/SetAnimationTime.srv"
  "srv/StepSimulation.srv"
  "srv/SetExecutionMode.srv"
  "srv/ExecuteConnect.srv"
)

set(action_files
  "action/RunScenario.action"
  "action/ExecuteManeuver.action"
  "action/ComputeCoverage.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
  ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime std_msgs geometry_msgs builtin_interfaces)
ament_package()
