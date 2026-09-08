import os
from glob import glob

from setuptools import find_packages, setup

package_name = "acav"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        # Installed for reference/inspection; the runnable form is the
        # console_scripts below, which import acav.build_cli /
        # acav.verify_cli. The files in scripts/ are thin shims so the
        # same tools can be run straight out of a source checkout.
        (os.path.join("share", package_name, "scripts"), glob("scripts/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.org",
    description="Attitude-Coupled Access Verification case study.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # C4 adds the scheduler node; C5 adds the campaign driver.
            "describe_scenario = acav.spec_cli:main",
            "build_acav_scenario = acav.build_cli:main",
            "verify_acav_scenario = acav.verify_cli:main",
        ],
    },
)
