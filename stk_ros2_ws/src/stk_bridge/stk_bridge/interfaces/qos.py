"""QoS profiles for the bridge's topics.

Different traffic wants different guarantees, and ROS 2's defaults suit
none of the three kinds this bridge carries:

**Telemetry** (position, attitude, rates) is best-effort with depth 1.
A control loop wants the *newest* state, not a replayed queue of stale
ones. Reliable delivery here is actively harmful: on a congested link it
buys retransmission of samples that were obsolete before they arrived.

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

    Only depth is configurable. Reliability and durability are
    deliberately not: a user who makes telemetry reliable to "avoid
    dropping data" will instead build unbounded latency into a control
    loop, and the failure looks like a performance problem rather than a
    configuration mistake. If that trade-off is genuinely wanted, it
    should be an explicit code change with a comment explaining why.
    """
    section = (config or {}).get("qos", {}) or {}
    return {
        "telemetry": telemetry_qos(int(section.get("telemetry_depth", 1))),
        "command": command_qos(int(section.get("command_depth", 10))),
        "status": status_qos(int(section.get("status_depth", 1))),
        "clock": clock_qos(int(section.get("clock_depth", 10))),
    }
