"""Lockstep reliability is opt-in; streaming defaults are preserved."""
import pytest

pytest.importorskip("rclpy")
from rclpy.qos import ReliabilityPolicy
from stk_bridge.interfaces.qos import from_config


def test_lockstep_reliable_override_preserves_streaming_default():
    assert from_config()["telemetry"].reliability == ReliabilityPolicy.BEST_EFFORT
    profiles = from_config({"qos": {"telemetry_reliability": "reliable"}})
    assert profiles["telemetry"].reliability == ReliabilityPolicy.RELIABLE
    assert profiles["telemetry"].depth == 1


def test_invalid_reliability_rejected():
    with pytest.raises(ValueError, match="telemetry_reliability"):
        from_config({"qos": {"telemetry_reliability": "typo"}})
