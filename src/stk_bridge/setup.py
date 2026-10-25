import os
from glob import glob

from setuptools import find_packages, setup

package_name = "stk_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"),
            glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.org",
    description=(
        "Bidirectional ROS 2 bridge for AGI/Ansys STK 12.6, driving a "
        "MATLAB/Simulink satellite digital twin."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Phase 5 populates nodes/*.py with the classes referenced here.
            "bridge_node = stk_bridge.nodes.bridge_node:main",
            "clock_node = stk_bridge.nodes.clock_node:main",
            "diagnostics_node = stk_bridge.nodes.diagnostics_node:main",
        ],
    },
)
