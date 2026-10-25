#!/usr/bin/env python3
"""Build the ACAV STK scenario. Runnable from a source checkout.

    python scripts/build_acav_scenario.py --backend com
    python scripts/build_acav_scenario.py --backend mock --verify

The implementation lives in ``acav.build_cli`` so that setup.py can
expose it as a console_script (``ros2 run acav build_acav_scenario``).
This file exists so the same code can be run the way
``stk_bridge/scripts/check_environment.py`` is run -- straight out of
the source tree, with no colcon build and no sourced workspace.
"""

import os
import sys


def _bootstrap_source_checkout() -> None:
    """Put src/acav and src/stk_bridge on sys.path if they are not already.

    Harmless when the workspace is installed and sourced: the installed
    packages are found first either way.
    """
    scripts = os.path.dirname(os.path.abspath(__file__))
    package_root = os.path.dirname(scripts)          # src/acav
    workspace_src = os.path.dirname(package_root)    # src/
    for candidate in (package_root,
                      os.path.join(workspace_src, "stk_bridge")):
        if os.path.isdir(candidate) and candidate not in sys.path:
            sys.path.insert(0, candidate)


_bootstrap_source_checkout()

from acav.build_cli import main  # noqa: E402  (after the path bootstrap)

if __name__ == "__main__":
    sys.exit(main())
