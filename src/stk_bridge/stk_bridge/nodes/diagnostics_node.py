"""Human-readable diagnostics for a running bridge.

Subscribes to ``stk/status`` and ``stk/diagnostics`` and prints a
periodic summary. Deliberately read-only: it holds no STK connection and
issues no commands, so it can be started and stopped freely against a
live run without any risk of perturbing it.

The counters worth watching, and what they mean:

``dropped_attitude_samples`` / ``dropped_position_samples``
    Non-monotonic or STK-rejected commands. A steady nonzero rate means
    the controller is publishing faster than simulation time advances,
    or is re-sending stamps. Not fatal, but the control input is not
    what the controller thinks it is.

``command_queue_depth`` approaching capacity
    STK cannot keep up with the configured rate. Latency is climbing and
    the drop policy is about to start discarding commands.

``last_backend_round_trip_ms``
    The practical ceiling on loop rate. If this is 50 ms, a 20 Hz loop
    is already saturated regardless of what the configuration says.
"""

from __future__ import annotations

import logging
from typing import Optional

LOGGER = logging.getLogger(__name__)

_STATE_NAMES = {0: "STARTING", 1: "CONNECTED", 2: "DEGRADED",
                3: "DISCONNECTED", 4: "ERROR"}


class DiagnosticsNode:
    def __init__(self, node, config: dict):
        from stk_bridge.interfaces import qos as qos_module
        from stk_msgs.msg import BackendDiagnostics, BridgeStatus

        self.node = node
        prefix = str(config.get("topic_prefix", "stk"))
        profiles = qos_module.from_config(config)

        self._status: Optional[object] = None
        self._diagnostics: Optional[object] = None

        node.create_subscription(BridgeStatus, f"{prefix}/status",
                                 self._on_status, profiles["status"])
        node.create_subscription(BackendDiagnostics, f"{prefix}/diagnostics",
                                 self._on_diagnostics, profiles["status"])

        period = float((config.get("diagnostics") or {}).get("period_s", 5.0))
        node.create_timer(period, self._report)

    def _on_status(self, message) -> None:
        self._status = message

    def _on_diagnostics(self, message) -> None:
        self._diagnostics = message

    def _report(self) -> None:
        if self._status is None:
            LOGGER.info("no BridgeStatus received yet -- is the bridge "
                        "running, and does topic_prefix match?")
            return

        status = self._status
        state = _STATE_NAMES.get(status.state, str(status.state))
        saturation = (
            100.0 * status.command_queue_depth / status.command_queue_capacity
            if status.command_queue_capacity else 0.0
        )

        lines = [
            f"state={state} backend={status.backend_type} "
            f"stk={status.stk_version_string}",
            f"  queue {status.command_queue_depth}/"
            f"{status.command_queue_capacity} ({saturation:.0f}%)  "
            f"round-trip {status.last_backend_round_trip_ms:.1f} ms",
            f"  dropped: attitude={status.dropped_attitude_samples} "
            f"position={status.dropped_position_samples}",
        ]
        if status.last_error:
            lines.append(f"  last error: {status.last_error}")

        if self._diagnostics is not None:
            diagnostics = self._diagnostics
            lines.append(
                f"  licensed: integration="
                f"{diagnostics.licensed_integration} "
                f"astrogator={diagnostics.licensed_astrogator} "
                f"realtime={diagnostics.licensed_realtime_propagator}"
            )
            lines.append(
                f"  python={diagnostics.python_version} "
                f"wheel={diagnostics.agi_stk_wheel_version} "
                f"com_thread={diagnostics.com_apartment_thread_id}"
            )

        if saturation > 80.0:
            lines.append("  WARNING: command queue is nearly full; STK is "
                         "not keeping up with the configured rate")

        LOGGER.info("\n".join(lines))


def main(args=None) -> None:
    import rclpy
    from rclpy.node import Node

    from stk_bridge.nodes.bridge_node import load_config

    rclpy.init(args=args)
    node = Node("stk_diagnostics")
    node.declare_parameter("config_file", "")
    config = load_config(node.get_parameter("config_file").value)

    logging.basicConfig(level=logging.INFO, format="%(message)s")

    DiagnosticsNode(node, config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
