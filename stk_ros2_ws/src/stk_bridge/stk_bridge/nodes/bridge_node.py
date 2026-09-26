"""The bridge node.

Concurrency model
-----------------
Three kinds of thread, with a strict rule about which touches STK:

1. **The COM apartment thread**, owned by ``BackendWorker``. The *only*
   thread that calls into ``agi.stk12``. Everything else reaches STK
   through ``worker.submit`` (fire-and-forget) or ``worker.call``
   (blocking).
2. **Executor threads**, running ROS callbacks: subscribers, services,
   actions. These never touch STK directly and never block on the step
   loop.
3. **The step loop thread**, created here. Senses, publishes, applies
   commands, advances time, and waits on the lockstep barrier.

The step loop is a dedicated thread rather than an executor timer, and
that is the single most important decision in this file. In lockstep the
loop must *block* waiting for participants to acknowledge a step, and
those acknowledgements arrive as ROS messages. If the loop occupied an
executor thread, it would be waiting for a callback that needs an
executor thread to run -- with a small thread pool that is a deadlock,
and with a large one it is an intermittent deadlock that appears only
under load. Separating them makes the dependency acyclic by
construction.

Callback groups reinforce it: command subscribers and the barrier ack
live in a reentrant group so they remain serviceable while services or
actions are busy.

Startup ordering
----------------
Connect, validate, then publish -- never overlapping. The node does not
create publishers until convention validation has passed, because a
subscriber that receives one message with unverified units or an
unverified quaternion pairing has already been given bad data, and no
later correction reaches it.
"""

from __future__ import annotations

import logging
import os
import threading
import time
from typing import Dict, List, Optional

import yaml

from stk_bridge.exceptions import LockstepBarrierTimeoutError

LOGGER = logging.getLogger(__name__)


class BridgeContext:
    """Shared state handed to service and action handlers."""

    def __init__(self, node: "StkBridgeNode"):
        self._node = node

    @property
    def time_sync(self):
        return self._node.time_sync

    @property
    def sensing(self):
        return self._node.sensing

    @property
    def actuation(self):
        return self._node.actuation

    @property
    def scenario(self):
        return self._node.scenario

    @property
    def inbox(self):
        return self._node.inbox

    def request_step(self, step_s: float = 0.0):
        return self._node.request_step(step_s)

    def on_scenario_changed(self, epoch) -> None:
        self._node.on_scenario_changed(epoch)


class StkBridgeNode:
    """Wires the core modules to ROS 2."""

    def __init__(self, node, config: Dict):
        from stk_bridge.backends import create_backend
        from stk_bridge.core.actuation import ActuationDispatcher
        from stk_bridge.core.com_thread import BackendWorker
        from stk_bridge.core.frames import FrameRegistry
        from stk_bridge.core.health import HealthConfig, HealthMonitor
        from stk_bridge.core.rate_group import RateGroupScheduler
        from stk_bridge.core.scenario_manager import (
            ScenarioConfig,
            ScenarioManager,
        )
        from stk_bridge.core.sensing import SensingConfig
        from stk_bridge.core.step_timing import PhaseTimings
        from stk_bridge.core.time_sync import TimeSync, TimeSyncConfig

        self.node = node
        self.config = config
        self._prefix = str(config.get("topic_prefix", "stk"))

        self.frames = FrameRegistry.from_config(config)
        self.scheduler = RateGroupScheduler.from_config(config)
        self.sensing_config = SensingConfig.from_config(config)
        self.health = HealthMonitor(HealthConfig.from_config(config))

        backend = create_backend(config)
        worker_config = (config.get("worker") or {})
        self.worker = BackendWorker(
            backend,
            queue_size=int(worker_config.get("queue_size", 256)),
            full_policy=str(worker_config.get("full_policy", "drop_oldest")),
            default_timeout_s=float(worker_config.get("timeout_s", 10.0)),
        )

        self.time_sync = TimeSync(
            TimeSyncConfig.from_config(config),
            advance_backend=self._advance_stk_clock,
        )
        self.scenario = ScenarioManager(
            backend, self.frames, ScenarioConfig.from_config(config))
        self.actuation = ActuationDispatcher(
            backend, self.frames,
            default_attitude_inertial=bool(
                (config.get("actuation") or {}).get(
                    "default_attitude_inertial", False)),
        )

        self.sensing = None          # built after validation
        self.inbox = None
        self.publishers: Dict[str, object] = {}
        self.bridge_publishers = None
        self.tf = None
        self._services = None
        self._actions = None

        self._context = BridgeContext(self)
        self._loop_thread: Optional[threading.Thread] = None
        self._running = threading.Event()
        self._step_request = threading.Event()
        self._step_size = 0.0
        self._step_lock = threading.Lock()
        self._loop_iterations = 0
        self._last_status_wall = 0.0
        #: E6's latency breakdown. See core/step_timing.py for why this
        #: is a windowed accumulator rather than a running lifetime mean.
        self._timings = PhaseTimings()

    # -- startup -----------------------------------------------------------

    def start(self) -> None:
        """Connect, validate, then wire up ROS. Order is load-bearing."""
        from stk_bridge.core.sensing import SensingPipeline
        from stk_bridge.interfaces import qos as qos_module
        from stk_bridge.interfaces.ros_subscribers import CommandInbox

        LOGGER.info("starting STK bridge")
        self.worker.start(timeout_s=float(
            (self.config.get("worker") or {}).get("start_timeout_s", 120.0)))

        report = self.worker.call(
            lambda backend: self.scenario.setup(
                run_validation=True,
                validation_config=self.config,
            ),
            "scenario setup + validation", timeout_s=300.0,
        )
        LOGGER.info(
            "validated: attitude=%r rates=%r rate-scale=%r",
            report.attitude_provider, report.angular_rate_provider,
            report.angle_rate_scale.source,
        )

        self.sensing = SensingPipeline(
            self.worker.backend, self.frames, self.sensing_config, report)
        self.inbox = CommandInbox(coalesce=bool(
            (self.config.get("actuation") or {}).get("coalesce", True)))

        qos_profiles = qos_module.from_config(self.config)
        self._build_interfaces(qos_profiles)

        self.time_sync.start(self.scenario.epoch)
        self.health.record_success()

        self._running.set()
        self._loop_thread = threading.Thread(
            target=self._loop, name="stk-bridge-loop", daemon=True)
        self._loop_thread.start()
        LOGGER.info("bridge running in %s mode", self.time_sync.mode.name)

    def _build_interfaces(self, qos_profiles: Dict[str, object]) -> None:
        from rclpy.callback_groups import (
            MutuallyExclusiveCallbackGroup,
            ReentrantCallbackGroup,
        )
        from stk_bridge.interfaces.ros_actions import ActionServers
        from stk_bridge.interfaces.ros_publishers import (
            BridgePublishers,
            SatellitePublishers,
        )
        from stk_bridge.interfaces.ros_services import ServiceServers
        from stk_bridge.interfaces.ros_subscribers import CommandSubscribers
        from stk_bridge.interfaces.tf_broadcaster import TfBroadcaster

        # Reentrant: the barrier ack must remain serviceable while the
        # step loop blocks on it and while services are busy.
        io_group = ReentrantCallbackGroup()
        service_group = ReentrantCallbackGroup()

        for path in self.sensing_config.satellites:
            self.publishers[path] = SatellitePublishers(
                self.node, path, self.frames, qos_profiles, self._prefix)

        self.bridge_publishers = BridgePublishers(
            self.node, qos_profiles, self._prefix,
            publish_clock=self.time_sync.config.publish_clock)

        self._subscribers = CommandSubscribers(
            self.node, self.inbox, qos_profiles, self._prefix,
            callback_group=io_group,
            default_frame=self.sensing_config.position_frame,
            on_barrier_ack=self.time_sync.acknowledge,
        )

        if bool((self.config.get("tf") or {}).get("enabled", True)):
            self.tf = TfBroadcaster(self.node, self.frames)

        self._services = ServiceServers(
            self.node, self.worker, self._context, self._prefix,
            callback_group=service_group)
        self._actions = ActionServers(
            self.node, self.worker, self._context, self._prefix,
            callback_group=service_group)

    # -- shutdown ----------------------------------------------------------

    def stop(self) -> None:
        LOGGER.info("stopping STK bridge")
        self._running.clear()
        self._step_request.set()
        if self._loop_thread is not None:
            self._loop_thread.join(timeout=5.0)
            self._loop_thread = None
        self.worker.stop(timeout_s=10.0)

    # -- clock -------------------------------------------------------------

    def _advance_stk_clock(self, when) -> None:
        """Push simulation time into STK.

        Fire-and-forget: the step loop must not block on a COM call to
        set the animation time, since that would couple step rate to
        COM latency. If the queue is full the sample is dropped and the
        next step's time subsumes it -- animation time is absolute, not
        incremental, so a dropped update is self-correcting.
        """
        self.worker.submit(
            lambda backend: backend.set_current_time(when),
            "set_current_time")

    def on_scenario_changed(self, epoch) -> None:
        self.time_sync.start(epoch)
        self.actuation.reset()
        self.inbox.clear()
        self.sensing.invalidate_propagator_cache()
        self.scheduler.reset()
        LOGGER.info("scenario changed; time and guards reset to epoch")

    def request_step(self, step_s: float = 0.0):
        """Ask the loop for one lockstep step and wait for it.

        Called from service and action handlers on executor threads. The
        step itself still happens on the loop thread, so STK access and
        barrier waiting stay on the one thread that owns them.
        """
        from stk_bridge.core.time_sync import ExecutionMode

        if self.time_sync.mode is not ExecutionMode.LOCKSTEP:
            raise RuntimeError("request_step is only valid in lockstep mode")

        with self._step_lock:
            self._step_size = step_s
            before = self.time_sync.step_index
            self._step_request.set()

        deadline = time.monotonic() + 30.0
        while self.time_sync.step_index == before:
            if time.monotonic() > deadline:
                raise TimeoutError("step did not complete within 30s")
            time.sleep(0.001)
        return self.time_sync.now()

    # -- the loop ----------------------------------------------------------

    def _loop(self) -> None:
        from stk_bridge.core.time_sync import ExecutionMode

        while self._running.is_set():
            try:
                if self.time_sync.mode is ExecutionMode.LOCKSTEP:
                    self._lockstep_iteration()
                else:
                    self._free_running_iteration()
                self._loop_iterations += 1
            except LockstepBarrierTimeoutError as exc:
                LOGGER.error("lockstep aborted; time will not advance: %s", exc)
                self.health.record_error(str(exc), fatal=True)
                self._running.clear()
                self._publish_status(self.time_sync.now())
            except Exception as exc:  # noqa: BLE001 - keep non-barrier handling
                LOGGER.exception("bridge loop iteration failed")
                self.health.record_error(f"{type(exc).__name__}: {exc}")
                time.sleep(0.1)

    def _lockstep_iteration(self) -> None:
        # Wait to be asked. External drivers (RunScenario, the
        # StepSimulation service) set the event; an auto-stepping
        # configuration sets it immediately.
        auto = bool((self.config.get("time") or {}).get("auto_step", True))
        if not auto:
            if not self._step_request.wait(timeout=0.1):
                return
        self._step_request.clear()

        with self._step_lock:
            step_size = self._step_size
            self._step_size = 0.0

        # 1. Apply commands computed for the previous step.
        with self._timings.timed("actuate"):
            self._apply_commands()
        # 2. Advance.
        now = self.time_sync.step(step_size or None)
        # 3. Publish the new state and the clock, so participants have
        #    what they need before being asked to acknowledge.
        self._publish_clock(now)
        with self._timings.timed("sense"):
            self._sense_and_publish(now)
        self._run_rate_groups()
        self._publish_status(now)
        # 4. Wait for acknowledgement. Timed for E6, though this
        #    iteration's wait lands in the NEXT status publish's window
        #    -- see core/step_timing.py's module docstring for why that
        #    one-window lag is immaterial to a multi-minute mean.
        #    A step-1 wait that needed the startup retry is NOT recorded:
        #    it is minutes of launch latency (a human starting MATLAB,
        #    the scheduler's process start), and as one sample in a
        #    ~7200-step run it would add several ms to the reported
        #    mean barrier wait -- the order of the quantity E6 measures.
        started = time.perf_counter()
        waited_for_startup = False
        while self._running.is_set():
            try:
                self.time_sync.await_barrier()
                break
            except LockstepBarrierTimeoutError:
                startup_wait = bool((self.config.get("time") or {}).get(
                    "wait_for_startup_participants", False))
                if not startup_wait or self.time_sync.step_index != 1:
                    raise
                # Republish the SAME initial step for late joiners. Never
                # reset the barrier or advance STK while MATLAB starts.
                waited_for_startup = True
                LOGGER.info("waiting at initial step for lockstep participants")
                self._publish_clock(now)
                self._sense_and_publish(now)
                self._publish_status(now)
        if not waited_for_startup:
            self._timings.record(
                "barrier_wait", (time.perf_counter() - started) * 1000.0)

    def _free_running_iteration(self) -> None:
        # No barrier wait outside lockstep -- see BridgeStatus.msg's own
        # field comment: step_samples tells a consumer that phase was
        # never measured this run rather than implying a suspiciously
        # fast zero.
        now = self.time_sync.now()
        with self._timings.timed("actuate"):
            self._apply_commands()
        self._publish_clock(now)
        with self._timings.timed("sense"):
            self._sense_and_publish(now)
        self._run_rate_groups()
        self._publish_status(now)
        self.time_sync.sleep_until_next(
            self.scheduler.next_due_time(self.time_sync.sim_seconds()))

    # -- loop stages -------------------------------------------------------

    def _apply_commands(self) -> None:
        from stk_bridge.interfaces.ros_subscribers import apply_commands

        commands = self.inbox.drain()
        if not commands:
            return
        try:
            self.worker.call(
                lambda backend: apply_commands(self.actuation, commands),
                "apply_commands")
            self.health.record_success()
        except Exception as exc:
            LOGGER.warning("failed to apply %d command(s): %s",
                           len(commands), exc)
            self.health.record_error(str(exc))

    def _sense_and_publish(self, now) -> None:
        for path, publishers in self.publishers.items():
            try:
                samples = self.worker.call(
                    lambda backend, p=path: self.sensing.read_all(p, now),
                    f"sense({path})")
            except Exception as exc:
                LOGGER.debug("sensing failed on %s: %s", path, exc)
                self.health.record_error(str(exc))
                continue

            if not samples:
                # No state at this epoch. Publishing nothing is
                # deliberate -- see ros_publishers' module docstring.
                continue

            publishers.publish_samples(samples)
            if self.tf is not None:
                self.tf.broadcast_samples(path, samples)
            self.health.record_success()

        self.health.update_from_sensing(self.sensing.statistics())

    def _run_rate_groups(self) -> None:
        self.scheduler.run_due(
            self.time_sync.sim_seconds(),
            on_error=lambda group, exc: self.health.record_error(
                f"rate group {group}: {exc}"),
        )

    def _publish_clock(self, now) -> None:
        if self.bridge_publishers is None:
            return
        self.bridge_publishers.publish_clock(now)
        self.bridge_publishers.publish_scenario_time(
            now, self.scenario.epoch, int(self.time_sync.mode),
            self.time_sync.config.real_time_factor,
            self.time_sync.step_index, self.time_sync.paused,
        )

    def _publish_status(self, now) -> None:
        # Fixed 1 Hz on wall time, not simulation time: a watchdog must
        # be able to tell "bridge is dead" from "simulation is paused",
        # and a status heartbeat tied to sim time stops in both cases.
        wall = time.monotonic()
        if wall - self._last_status_wall < 1.0:
            return
        self._last_status_wall = wall

        self.health.check_heartbeat()
        backend = self.worker.backend
        timings = self._timings.summary()
        self._timings.reset()
        self.bridge_publishers.publish_status(
            state=int(self.health.state),
            backend_type=backend.backend_type.value,
            stk_version=backend.info.stk_version,
            dropped=self.actuation.total_dropped(),
            queue_depth=self.worker.queue_depth,
            queue_capacity=self.worker.queue_capacity,
            latency_ms=self.worker.last_latency_ms,
            last_error=self.health.last_error,
            timings=timings,
            when=now,
        )
        self.bridge_publishers.publish_diagnostics(
            backend, self.scenario.validation_report, self.worker, now)

    # -- introspection -----------------------------------------------------

    def statistics(self) -> Dict:
        return {
            "loop_iterations": self._loop_iterations,
            "worker": self.worker.statistics(),
            "time": self.time_sync.statistics(),
            "health": self.health.statistics(),
            "inbox": self.inbox.statistics() if self.inbox else {},
            "actuation": self.actuation.statistics(),
            "sensing": self.sensing.statistics() if self.sensing else {},
            "rate_groups": self.scheduler.statistics(),
        }


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def load_config(path: Optional[str]) -> Dict:
    if not path:
        return {}
    if not os.path.isfile(path):
        LOGGER.warning("config file %s not found; using defaults", path)
        return {}
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def main(args=None) -> None:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node

    rclpy.init(args=args)
    node = Node("stk_bridge")

    node.declare_parameter("config_file", "")
    # Do NOT declare use_sim_time here. rclpy pre-declares it for every
    # node (confirmed on Humble: declaring it raises
    # ParameterAlreadyDeclaredException and the node never starts), and
    # the bridge has no use for it anyway -- it is the clock *source*,
    # stamping every message from STK time rather than from
    # node.get_clock(). It is the downstream consumers that need
    # use_sim_time, and the launch files set it for them.

    config_path = node.get_parameter("config_file").value
    config = load_config(config_path)

    logging.basicConfig(
        level=getattr(logging, str(config.get("log_level", "INFO")).upper(),
                      logging.INFO),
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
    )

    bridge = StkBridgeNode(node, config)
    # Threads: enough for the service/action group, the io group, and
    # headroom. The step loop does NOT consume one -- it has its own
    # thread precisely so it can block on the barrier without starving
    # the ack callback that releases it.
    executor = MultiThreadedExecutor(num_threads=int(
        (config.get("executor") or {}).get("threads", 6)))
    executor.add_node(node)

    try:
        bridge.start()
        while rclpy.ok() and bridge._running.is_set():
            executor.spin_once(timeout_sec=0.5)
        if rclpy.ok() and not bridge._running.is_set():
            raise RuntimeError("bridge execution aborted; see preceding lockstep error")
    except KeyboardInterrupt:
        LOGGER.info("interrupted")
    except Exception:
        LOGGER.exception("bridge failed to start")
        raise
    finally:
        bridge.stop()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
