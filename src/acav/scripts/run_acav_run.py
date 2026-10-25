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
help is available without a build. On Windows an actual run loads the
built workspace's install/setup.bat in a child process and validates
ROS message type support before starting the bridge.
"""

import os
import sys
import subprocess


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
    workspace = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    workspace = os.path.dirname(workspace)
    if (os.name == 'nt' and not any(a in sys.argv for a in ('-h', '--help'))
            and os.environ.get('ACAV_RUN_WORKSPACE') != workspace):
        from acav.ros_environment import run_with_workspace
        try:
            sys.exit(run_with_workspace(__file__, sys.argv[1:], workspace))
        except (RuntimeError, OSError, subprocess.TimeoutExpired) as exc:
            print(f'ROS workspace setup failed: {exc}', file=sys.stderr)
            sys.exit(2)
    sys.exit(main())
