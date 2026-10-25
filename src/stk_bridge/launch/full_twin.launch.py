"""Full digital-twin launch: bridge + diagnostics, with MATLAB documented
as a separately-started process.

MATLAB/Simulink is intentionally NOT launched from here. ros2 launch has
no first-class way to manage a MATLAB session, and coupling the two would
make ordinary MATLAB debugging (breakpoints, the workspace inspector)
fight the launch system. Start MATLAB separately and run
stk_matlab/examples/example_03_lockstep_stepping.m, or your own
Simulink model built from simulink/build_twin_template.m, once this
launch file reports the bridge is CONNECTED on stk/status.

    ros2 launch stk_bridge full_twin.launch.py
    # in a second terminal:
    ros2 topic echo /stk/status
    # once state == CONNECTED, start MATLAB and run the twin.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("stk_bridge")

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, "launch", "stk_bridge.launch.py")),
    )

    diagnostics = Node(
        package="stk_bridge",
        executable="diagnostics_node",
        name="stk_diagnostics",
        output="screen",
        parameters=[{
            "config_file": os.path.join(share, "config", "bridge_params.yaml"),
        }],
    )

    reminder = LogInfo(msg=(
        "stk_bridge is starting. Once `ros2 topic echo /stk/status` shows "
        "state: 1 (CONNECTED), start MATLAB separately and run "
        "stk_matlab/examples/example_03_lockstep_stepping.m -- see "
        "docs/matlab/client.md."
    ))

    return LaunchDescription([reminder, bridge, diagnostics])
