"""QoS profiles for the bridge's topics.

Different traffic wants different guarantees, and ROS 2's defaults suit
none of the three kinds this bridge carries:

**Telemetry** (position, attitude, rates) defaults to best-effort with
depth 1 for streaming. ACAV lockstep explicitly selects reliable delivery:
its current sample cannot become obsolete until all participants ack it.

**Commands** (position/attitude/rate injection) are reliable with a
small queue. A dropped command is a real loss -- STK's real-time
channels are append-only, so a missing sample leaves a gap in the
injected history that nothing later can fill.

**Status** is transient-local so a late-joining subscriber (an operator
opening a dashboard mid-run) immediately sees current health rather than
waiting for the next heartbeat.

**Clock** follows the ROS convention for ``/clock``: reliable, volatile,
small depth. Every node's time source depends on it, and a missed tick
in lockstep means a participant acknowledges the wrong step.
"""

from __future__ import annotations

from typing import Dict, Optional


def _import_qos():
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    return QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


def telemetry_qos(depth: int = 1):
    """Best-effort, keep-last-1. Newest state wins."""
    QoSProfile, Reliability, Durability, History = _import_qos()
    return QoSProfile(
        history=History.KEEP_LAST,
        depth=depth,
        reliability=Reliability.BEST_EFFORT,
        durability=Durability.VOLATILE,
    )


def command_qos(depth: int = 10):
    """Reliable, keep-last-N. Dropped commands leave real gaps."""
    QoSProfile, Reliability, Durability, History = _import_qos()
    return QoSProfile(
        history=History.KEEP_LAST,
        depth=depth,
        reliability=Reliability.RELIABLE,
        durability=Durability.VOLATILE,
    )


def status_qos(depth: int = 1):
    """Reliable + transient-local, so late joiners see current state."""
    QoSProfile, Reliability, Durability, History = _import_qos()
    return QoSProfile(
        history=History.KEEP_LAST,
        depth=depth,
        reliability=Reliability.RELIABLE,
        durability=Durability.TRANSIENT_LOCAL,
    )


def clock_qos(depth: int = 10):
    """Reliable, volatile. Matches the ROS 2 convention for /clock."""
    QoSProfile, Reliability, Durability, History = _import_qos()
    return QoSProfile(
        history=History.KEEP_LAST,
        depth=depth,
        reliability=Reliability.RELIABLE,
        durability=Durability.VOLATILE,
    )


def from_config(config: Optional[Dict] = None) -> Dict[str, object]:
    """Build the profile set, honouring overrides in ``qos_profiles.yaml``.

    Streaming telemetry defaults to best effort. Lockstep deployments can
    explicitly request reliable telemetry: there is no newer step until
    participants consume this one, so dropping its only sample can deadlock
    the barrier. Subscribers must also request reliable delivery.
    """
    section = (config or {}).get("qos", {}) or {}
    reliability = section.get("telemetry_reliability", "best_effort")
    if reliability not in ("best_effort", "reliable"):
        raise ValueError("qos.telemetry_reliability must be best_effort or reliable")
    telemetry = telemetry_qos(int(section.get("telemetry_depth", 1)))
    if reliability == "reliable":
        _, Reliability, _, _ = _import_qos()
        telemetry.reliability = Reliability.RELIABLE
    return {
        "telemetry": telemetry,
        "command": command_qos(int(section.get("command_depth", 10))),
        "status": status_qos(int(section.get("status_depth", 1))),
        "clock": clock_qos(int(section.get("clock_depth", 10))),
    }
