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
    maintainer="Mohammadamin Alizadeh",
    maintainer_email="mohammada.alizadeh@mail.sbu.ac.ir",
    description=(
        "Bidirectional ROS 2 bridge for AGI/Ansys STK 12.6 with "
        "free-run, paced and lockstep execution."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bridge_node = stk_bridge.nodes.bridge_node:main",
            "clock_node = stk_bridge.nodes.clock_node:main",
            "diagnostics_node = stk_bridge.nodes.diagnostics_node:main",
        ],
    },
)
