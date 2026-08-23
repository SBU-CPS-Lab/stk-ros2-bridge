"""Production launch: bridge node against real STK, backend from config.

Usage:
    ros2 launch stk_bridge stk_bridge.launch.py
    ros2 launch stk_bridge stk_bridge.launch.py config_file:=/path/to/custom.yaml

The backend type (com/engine/connect) comes from the config file's
`backend.type`, not from a launch argument -- deployment topology is a
configuration concern (see docs/00_architecture.md section 2), and
having two places to set it (launch arg vs config) invites them to
disagree silently.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_config = os.path.join(
        get_package_share_directory("stk_bridge"),
        "config", "bridge_params.yaml",
    )

    config_file_arg = DeclareLaunchArgument(
        "config_file", default_value=default_config,
        description="Path to the bridge YAML configuration.",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Standard ROS 2 sim-time flag. The bridge is the "
                    "/clock authority in lockstep/paced modes, so this "
                    "should stay true for every node in the graph, "
                    "including the bridge's own logging.",
    )

    bridge_node = Node(
        package="stk_bridge",
        executable="bridge_node",
        name="stk_bridge",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "config_file": LaunchConfiguration("config_file"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        bridge_node,
    ])
