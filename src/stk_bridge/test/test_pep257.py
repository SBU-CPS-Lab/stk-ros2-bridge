"""ament_pep257 docstring-convention conformance."""

import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    try:
        from ament_pep257.main import main
    except ImportError:
        pytest.skip("ament_pep257 not available outside a ROS 2 workspace")

    rc = main(argv=[".", "test"])
    assert rc == 0, "found code style errors / warnings"
