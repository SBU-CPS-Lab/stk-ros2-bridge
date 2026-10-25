"""Launch the bridge against the mock backend -- no STK required.

This is the fastest path to confirming the ROS-facing half of the bridge
works: topics, services, actions, tf, and the lockstep loop, all
exercised against the analytic SSS-1 Kepler model in
backends/mock_backend.py. If this launch file does not produce a healthy
`stk/status` topic, the problem is in the bridge, not in STK -- start
debugging here before pointing anything at a real install.

    ros2 launch stk_bridge stk_bridge_mock.launch.py
    ros2 topic echo /stk/sss_1/state
    ros2 service call /stk/step_simulation stk_msgs/srv/StepSimulation "{}"
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    base_config_path = os.path.join(
        get_package_share_directory("stk_bridge"),
        "config", "bridge_params.yaml",
    )
    with open(base_config_path) as handle:
        config = yaml.safe_load(handle)

    # Override just the backend selection and the validation-fatal set
    # that assumes a real STK's exact behaviour (e.g. STK 12.6 NACKing
    # out-of-order attitude, which the mock also does -- see
    # mock_backend.py -- but future scenario tweaks should not have to
    # remember to keep both in sync).
    config["backend"] = {"type": "mock",
                         "mock": {"satellite_name": "SSS-1"}}
    config["scenario"]["scenario_path"] = ""

    # Write the merged config to a temp file: launch_ros Node parameters
    # take a file path, not an in-memory dict, for the yaml-file form.
    fd, merged_path = tempfile.mkstemp(prefix="stk_bridge_mock_", suffix=".yaml")
    with os.fdopen(fd, "w") as handle:
        yaml.safe_dump(config, handle)

    bridge_node = Node(
        package="stk_bridge",
        executable="bridge_node",
        name="stk_bridge",
        output="screen",
        emulate_tty=True,
        parameters=[{"config_file": merged_path, "use_sim_time": True}],
    )

    return LaunchDescription([bridge_node])
