"""Standalone /clock authority.

Optional. The bridge node publishes ``/clock`` itself by default, which
is right for the common single-bridge case. This node exists for the two
situations where that is wrong:

* **Multiple bridges.** Several STK instances feeding one ROS graph must
  share a clock; exactly one publisher can own it.
* **Replay and offline analysis.** Driving a recorded run through the
  same controller without STK attached at all.

Running this alongside a bridge that also publishes ``/clock`` produces
two authorities and timestamps that jump backwards, which corrupts every
tf lookup and message filter downstream. So the bridge must be
configured with ``time.publish_clock: false`` when this node is used --
and this node warns loudly if it detects another publisher.
"""

from __future__ import annotations

import logging
import time
from typing import Optional

LOGGER = logging.getLogger(__name__)


class ClockNode:
    """Publishes simulation time on ``/clock`` and ``stk/scenario_time``."""

    def __init__(self, node, config: dict):
        from rosgraph_msgs.msg import Clock
        from stk_bridge.core.time_sync import TimeSync, TimeSyncConfig
        from stk_bridge.interfaces import qos as qos_module
        from stk_msgs.msg import ScenarioTime

        self.node = node
        self.config = config
        prefix = str(config.get("topic_prefix", "stk"))
        profiles = qos_module.from_config(config)

        self._pub_clock = node.create_publisher(Clock, "/clock",
                                                profiles["clock"])
        self._pub_scenario = node.create_publisher(
            ScenarioTime, f"{prefix}/scenario_time", profiles["clock"])

        self.time_sync = TimeSync(TimeSyncConfig.from_config(config))
        self._epoch_s = float((config.get("time") or {}).get(
            "epoch_unix_seconds", time.time()))
        self._rate_hz = float((config.get("time") or {}).get(
            "clock_publish_hz", 50.0))
        self._timer = None

    def start(self) -> None:
        from stk_bridge.backends.base import StkTime

        self.time_sync.start(StkTime(self._epoch_s))
        self._warn_if_contended()
        self._timer = self.node.create_timer(1.0 / self._rate_hz,
                                             self._publish)
        LOGGER.info("clock node publishing /clock at %.1f Hz from epoch %.3f",
                    self._rate_hz, self._epoch_s)

    def _warn_if_contended(self) -> None:
        """Detect another /clock publisher before we add a second one."""
        try:
            count = self.node.count_publishers("/clock")
        except Exception:
            return
        if count > 1:
            LOGGER.error(
                "%d publishers already on /clock. Two clock authorities "
                "produce timestamps that move backwards, which silently "
                "corrupts tf lookups and message filters. Set "
                "time.publish_clock: false on the bridge node.", count,
            )

    def _publish(self) -> None:
        from rosgraph_msgs.msg import Clock
        from stk_bridge.core import conversions as conv
        from stk_msgs.msg import ScenarioTime

        now = self.time_sync.now()

        clock = Clock()
        clock.clock = conv.stk_time_to_ros(now)
        self._pub_clock.publish(clock)

        scenario = ScenarioTime()
        scenario.sim_time = conv.stk_time_to_ros(now)
        scenario.scenario_epoch = conv.stk_time_to_ros(self.time_sync.epoch)
        scenario.execution_mode = int(self.time_sync.mode)
        scenario.real_time_factor = self.time_sync.config.real_time_factor
        scenario.step_index = self.time_sync.step_index
        scenario.paused = self.time_sync.paused
        self._pub_scenario.publish(scenario)


def main(args=None) -> None:
    import rclpy
    from rclpy.node import Node

    from stk_bridge.nodes.bridge_node import load_config

    rclpy.init(args=args)
    node = Node("stk_clock")
    node.declare_parameter("config_file", "")
    config = load_config(node.get_parameter("config_file").value)

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s")

    clock = ClockNode(node, config)
    try:
        clock.start()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
