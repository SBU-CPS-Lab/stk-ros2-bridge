#!/usr/bin/env python3
"""Run ONE campaign run end to end (bridge, twin handoff, scheduler).

This is the command ``CommandLauncher`` invokes per run of a campaign
(see ``acav/campaign_runner.py`` and ``docs/08_deploy_runbook.md``), or
that can be run by hand for a single E6 measurement run.

    python scripts/run_acav_run.py --run-id E6-seed1 --output out.json \\
        --bridge-config config/bridge_params_acav.yaml --scenario ... \\
        --run-config out.runconfig.json

Written in the same spirit as ``check_environment.py``/
``verify_acav_scenario.py``: runnable straight out of a source checkout,
no colcon build and no sourced workspace needed to invoke it (ROS 2
itself, of course, still needs to be sourced for ``ros2 launch``/
``ros2 run`` to work).
"""

import os
import sys


def _bootstrap_source_checkout() -> None:
    scripts = os.path.dirname(os.path.abspath(__file__))
    package_root = os.path.dirname(scripts)          # src/acav
    workspace_src = os.path.dirname(package_root)    # src/
    for candidate in (package_root,
                      os.path.join(workspace_src, "stk_bridge")):
        if os.path.isdir(candidate) and candidate not in sys.path:
            sys.path.insert(0, candidate)


_bootstrap_source_checkout()

from acav.deploy_run_cli import main  # noqa: E402  (after path bootstrap)

if __name__ == "__main__":
    sys.exit(main())
