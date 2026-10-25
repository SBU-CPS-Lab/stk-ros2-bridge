"""Launch the bridge against a REMOTE STK over the Connect socket (T3).

For the cross-OS deployment: STK 12.6 on Windows, bridge and the rest of
the ROS 2 graph on Linux. See docs/bridge/architecture.md section 2 (T3) and
section 1's note that Connect is the ONLY network-transparent interface
in STK 12.6 -- there is no gRPC remoting until 12.8.

    ros2 launch stk_bridge stk_bridge_connect.launch.py stk_host:=192.168.1.50

Before using this in anger: confirm the Connect socket is reachable and
that ConControl AckOn succeeds from this machine (Windows Firewall
blocks inbound connections to port 5001 by default -- add a rule on the
STK host). Rate expectations differ materially from the COM backend; run
scripts/benchmark_rates.py against THIS topology, not the COM numbers.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_node(context, *args, **kwargs):
    import tempfile

    import yaml

    base_config_path = os.path.join(
        get_package_share_directory("stk_bridge"),
        "config", "bridge_params.yaml",
    )
    with open(base_config_path) as handle:
        config = yaml.safe_load(handle)

    config["backend"] = {
        "type": "connect",
        "connect": {
            "host": LaunchConfiguration("stk_host").perform(context),
            "port": int(LaunchConfiguration("stk_port").perform(context)),
            "timeout_s": 10.0,
            "connect_timeout_s": 5.0,
        },
    }

    fd, merged_path = tempfile.mkstemp(prefix="stk_bridge_connect_",
                                       suffix=".yaml")
    with os.fdopen(fd, "w") as handle:
        yaml.safe_dump(config, handle)

    return [Node(
        package="stk_bridge",
        executable="bridge_node",
        name="stk_bridge",
        output="screen",
        emulate_tty=True,
        parameters=[{"config_file": merged_path, "use_sim_time": True}],
    )]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("stk_host", default_value="localhost",
                              description="Hostname/IP of the Windows "
                                          "machine running STK 12.6."),
        DeclareLaunchArgument("stk_port", default_value="5001",
                              description="STK Connect port (Edit > "
                                          "Preferences > Connect)."),
        OpaqueFunction(function=_make_node),
    ])
