"""Validation must tolerate artifacts left by an interrupted run."""
from unittest.mock import Mock
import pytest
from stk_bridge.core.validation import ConventionValidator


def test_each_validation_uses_a_distinct_object_and_preserves_existing():
    objects = {"*/Satellite/StkBridgeValidation"}
    backend = Mock()

    def create(kind, name):
        path = f"*/{kind}/{name}"
        assert path not in objects
        objects.add(path)
        return path

    backend.create_object.side_effect = create
    backend.delete_object.side_effect = objects.remove
    first = ConventionValidator(backend)
    second = ConventionValidator(backend)
    path1 = first._setup_object()
    path2 = second._setup_object()
    assert path1 != path2
    first._teardown_object()
    second._teardown_object()
    assert objects == {"*/Satellite/StkBridgeValidation"}


def test_propagation_failure_cleans_up_the_new_object():
    backend = Mock()
    backend.create_object.return_value = "*/Satellite/owned"
    backend.propagate.side_effect = RuntimeError("propagation failed")
    validator = ConventionValidator(backend)
    with pytest.raises(RuntimeError, match="propagation failed"):
        validator._setup_object()
    backend.delete_object.assert_called_once_with("*/Satellite/owned")
