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
        (os.path.join("share", package_name, "config"),
            glob("config/*.yaml") + glob("config/*.json")),
        # Installed for reference/inspection; the runnable form is the
        # console_scripts below, which import acav.build_cli /
        # acav.verify_cli. The files in scripts/ are thin shims so the
        # same tools can be run straight out of a source checkout.
        (os.path.join("share", package_name, "scripts"), glob("scripts/*.py")),
    ],
    install_requires=["setuptools"],
    # matplotlib is deliberately NOT a hard dependency: it is needed only
    # to render C6's figures, on whatever machine does the analysis --
    # not on the deploy machine running acav_scheduler against live STK,
    # which should not need to pull in a plotting stack it never uses.
    # `run_acav_campaign report` imports it lazily and degrades to
    # JSON-only output with a clear message if it is missing.
    extras_require={"figures": ["matplotlib>=3.5"]},
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.org",
    description="Attitude-Coupled Access Verification case study.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "describe_scenario = acav.spec_cli:main",
            "build_acav_scenario = acav.build_cli:main",
            "verify_acav_scenario = acav.verify_cli:main",
            "export_twin_fixtures = acav.fixtures_cli:main",
            "acav_scheduler = acav.scheduler_node:main",
            "run_acav_campaign = acav.campaign_cli:main",
        ],
    },
)
