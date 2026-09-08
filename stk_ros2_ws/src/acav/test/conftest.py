"""Pytest configuration for the acav package.

Makes the package importable when running plain `pytest` from a source
checkout, without requiring a colcon build first -- the C1 contract
modules (spec, slew_model, metrics) have no ROS dependency by design,
so the fast inner loop should not need a sourced workspace.

C2 adds a dependency on `stk_bridge` (declared in package.xml), which
lives in the sibling source directory rather than in this package. It
is put on the path here for the same reason: the C2 builder and
verifier are exercised against the mock backend, which needs no ROS
and no STK, so those tests must stay runnable in the fast loop too.
"""

import os
import sys

_PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
_WORKSPACE_SRC = os.path.dirname(_PACKAGE_ROOT)

for _path in (_PACKAGE_ROOT, os.path.join(_WORKSPACE_SRC, "stk_bridge")):
    if os.path.isdir(_path) and _path not in sys.path:
        sys.path.insert(0, _path)
