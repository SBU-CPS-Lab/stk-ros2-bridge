"""Shared pytest configuration for stk_bridge's test suite.

Ensures the package is importable when tests are run directly with
`pytest` from a source checkout, without requiring a full `colcon build`
first -- useful for the fast inner loop while iterating on core/, which
has no ROS dependency.
"""

import os
import sys

_PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, _PACKAGE_ROOT)
