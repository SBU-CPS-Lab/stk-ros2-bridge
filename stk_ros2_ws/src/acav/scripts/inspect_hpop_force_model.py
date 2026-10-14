#!/usr/bin/env python3
"""Read-only HPOP force-model inspection. Runnable from a source checkout.

    python scripts/inspect_hpop_force_model.py --output force_model.txt

Attaches to the running STK Desktop and prints, without changing anything,
the force model of */Satellite/SSS-1. The implementation and the list of
every Object Model call it makes are in ``acav.hpop_inspection``.
"""

import os
import sys


def _bootstrap_source_checkout() -> None:
    """Put src/acav and src/stk_bridge on sys.path if they are not already."""
    scripts = os.path.dirname(os.path.abspath(__file__))
    package_root = os.path.dirname(scripts)          # src/acav
    workspace_src = os.path.dirname(package_root)    # src/
    for candidate in (package_root,
                      os.path.join(workspace_src, "stk_bridge")):
        if os.path.isdir(candidate) and candidate not in sys.path:
            sys.path.insert(0, candidate)


_bootstrap_source_checkout()

from acav.hpop_inspection import main  # noqa: E402  (after the path bootstrap)

if __name__ == "__main__":
    sys.exit(main())
