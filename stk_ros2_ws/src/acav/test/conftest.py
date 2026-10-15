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

# The test directory itself, so test modules can share helpers such as
# stk126_double.py without turning test/ into an installed package.
_TEST_DIR = os.path.dirname(os.path.abspath(__file__))

for _path in (_TEST_DIR, _PACKAGE_ROOT,
              os.path.join(_WORKSPACE_SRC, "stk_bridge")):
    if os.path.isdir(_path) and _path not in sys.path:
        sys.path.insert(0, _path)

import pytest  # noqa: E402  (after the path bootstrap)


@pytest.fixture(autouse=True)
def _accept_twin_identity(monkeypatch):
    """Offline tests have no twin to publish an identity: accept one.

    test_twin_identity_check.py exercises the real check by injecting
    its own transports; this only replaces the ROS-backed default.
    """
    import acav.deploy_run_cli as deploy_run_cli
    monkeypatch.setattr(deploy_run_cli, "_confirm_twin_identity_ros",
                        lambda run_config, timeout_s: ("v1 test", []))


@pytest.fixture(autouse=True)
def _graphics_off(monkeypatch):
    """Offline tests have no bridge to sample STK: report "off" (the
    default every run requires). test_graphics_mode_runner.py injects
    its own samples."""
    import acav.deploy_run_cli as deploy_run_cli
    entry = {"windows_2d": 0, "windows_3d": 0, "main_window": "minimized",
             "visible": True, "mode": "off", "step": 0}
    data = {"startup": entry, "samples": [], "end": entry}
    monkeypatch.setattr(deploy_run_cli, "_read_graphics_state_file",
                        lambda path, phase, timeout_s: data)
