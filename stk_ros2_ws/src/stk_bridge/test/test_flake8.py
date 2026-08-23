"""ament_flake8 conformance, standard for ament_python packages.

Skips cleanly if ament_flake8 is not installed (e.g. running this
package's tests outside a full ROS 2 workspace via plain pytest).
"""

import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    try:
        from ament_flake8.main import main
    except ImportError:
        pytest.skip("ament_flake8 not available outside a ROS 2 workspace")

    rc = main(argv=[])
    assert rc == 0, "flake8 found style errors; run ament_flake8 for details"
