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
import math
import os
import sys
import threading
import time
from typing import Dict, List, Optional, Set

import yaml

from stk_bridge.core import conversions as conv
from stk_bridge.core.clock_timing import ClockAdvanceTiming
from stk_bridge.core.graphics_state import GraphicsLog, describe
from stk_bridge.core.startup_phases import StartupPhases
from stk_bridge.exceptions import (
    ConventionValidationError,
    LockstepBarrierTimeoutError,
    LockstepCommandError,
)

LOGGER = logging.getLogger(__name__)

#: How often the initial step is republished while lockstep participants
#: are still starting (wall seconds).
STARTUP_REPUBLISH_S = 2.0
#: Free-run start gate: how often to check for the participants' acks.
FREE_RUN_GATE_POLL_S = 0.05
#: How often the step rate and E6's phase breakdown are logged.
TIMING_LOG_PERIOD_S = 60.0
#: Lockstep steps between "running" sensing summary lines. The final
#: lines are logged at the terminal epoch: run_acav_run.py may stop the
#: bridge before stop() runs (E6-BC-1), so stop() alone is not enough.
SUMMARY_EVERY_STEPS = 1000


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
        #: The scenario's stop time, read once at startup (None if not
        #: available). Orbit batching, the equivalence diagnostic and the
        #: terminal sensing summary use it.
        self._scenario_stop = None
        self._final_summary_logged = False
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
        #: The same breakdown, summed over a longer window and written to
        #: the log, so a run that never produces a record (it aborted, or
        #: the scheduler failed) still leaves its step rate behind.
        self._timing_log = {"wall": time.monotonic(), "step": 0,
                            "sense": 0.0, "actuate": 0.0,
                            "barrier_wait": 0.0, "samples": 0}
        self._warn_counts: Dict[str, int] = {}
        self._commands_late = 0
        self._commands_rejected = 0
        self._last_startup_log = 0.0
        self._advance_ms = (0.0, 0)
        self._clock_timing = ClockAdvanceTiming()
        #: STK's graphics state, a recorded run condition (see
        #: core/graphics_state.py); the runner reads the file.
        self._graphics_log = GraphicsLog(
            str(config.get("graphics_state_file") or ""))
        self._commanded_paths: Set[str] = set()
        self._commands_missing = 0
        self._attitude_mismatches = 0
        #: Each startup phase's duration, written to startup_phases_file
        #: as it happens (core/startup_phases.py); every change publishes
        #: BridgeStatus STARTING.
        self.startup = StartupPhases(
            str(config.get("startup_phases_file") or ""),
            on_change=self._publish_starting)

    # -- startup -----------------------------------------------------------

    def start(self) -> None:
        """Connect, validate, then wire up ROS. Order is load-bearing.

        Only the status publisher exists before validation, so the
        runner sees STARTING while the bridge connects and validates;
        commands, services and telemetry still wait for validation.
        """
        from stk_bridge.interfaces import qos as qos_module

        LOGGER.info("starting STK bridge")
        qos_profiles = qos_module.from_config(self.config)
        self.bridge_publishers = self._make_bridge_publishers(qos_profiles)
        try:
            self._start(qos_profiles)
        except BaseException as exc:
            self.startup.finish("failed", f"{type(exc).__name__}: {exc}")
            raise
        self.startup.finish("connected")

    def _start(self, qos_profiles) -> None:
        from stk_bridge.core.sensing import SensingPipeline
        from stk_bridge.interfaces.ros_subscribers import CommandInbox

        self._publish_starting("")
        with self.startup.phase("connect"):
            self.worker.start(timeout_s=float(
                (self.config.get("worker") or {}).get(
                    "start_timeout_s", 120.0)))

        report = self.worker.call(
            lambda backend: self.scenario.setup(
                run_validation=True,
                validation_config=self.config,
                phases=self.startup,
            ),
            "scenario setup + validation", timeout_s=300.0,
        )
        LOGGER.info(
            "validated: attitude=%r rates=%r rate-scale=%r",
            report.attitude_provider, report.angular_rate_provider,
            report.angle_rate_scale.source,
        )
        with self.startup.phase("sensing"):
            check_attitude_pairing(report, self.config)
            self.sensing = SensingPipeline(
                self.worker.backend, self.frames, self.sensing_config,
                report)
            self.sensing.set_attitude_write_probe(
                self._inertial_sample_written)
            self._apply_free_run_sensing_policy()
            self._scenario_stop = self._read_scenario_stop()
            # Before the interfaces exist, so before CONNECTED is
            # published: the runner checks this sample right after
            # bridge_connected.
            self._sample_graphics("startup")
            if self.sensing_config.orbit_batch_samples > 1:
                self._attach_orbit_batch()
            if self.sensing_config.equivalence_diagnostic:
                self._attach_equivalence_diagnostic()
            self.inbox = CommandInbox(coalesce=bool(
                (self.config.get("actuation") or {}).get("coalesce", True)))

        with self.startup.phase("interfaces"):
            self._build_interfaces(qos_profiles)
            self._arm_startup_gate(self.scenario.epoch)
        self.health.record_success()

        self._running.set()
        self._loop_thread = threading.Thread(
            target=self._loop, name="stk-bridge-loop", daemon=True)
        self._loop_thread.start()
        LOGGER.info("bridge running in %s mode", self.time_sync.mode.name)

    def _inertial_sample_written(self, path, when) -> bool:
        """Did this bridge write an inertial attitude sample for ``when``?

        (B) verifies the segment provider only at such an epoch; same
        1e-6 s match as _check_attitude_landed.
        """
        written = self.actuation.last_attitude(path)
        return (written is not None
                and abs(written[0] - when.epoch_seconds) <= 1e-6
                and bool(written[2]))

    def _read_scenario_stop(self):
        """The scenario stop time from STK, or None (logged)."""
        try:
            return self.worker.call(
                lambda backend: backend.get_scenario_stop_time(),
                "scenario stop time")
        except Exception as exc:            # noqa: BLE001 - optional
            LOGGER.warning("could not read the scenario stop time: %s", exc)
            return None

    def _log_sensing_summaries(self, final: bool) -> None:
        prefix = "" if final else "running "
        LOGGER.info("%s%s", prefix, self.sensing.combine_summary_line())
        LOGGER.info("%s%s", prefix, self.sensing.orbit_batch_summary_line())
        timing = getattr(self, "_clock_timing", None)
        if timing is not None:
            LOGGER.info("%s%s", prefix, timing.summary_line())
        pacer = getattr(self, "_free_pacer", None)
        if pacer is not None:
            LOGGER.info("%s%s", prefix, pacer.summary_line())
            self._write_json_file("free_run_pacing_file", {
                "cycles": pacer.cycles, "overruns": pacer.overruns,
                "period_s": pacer.period_s})
        if final:
            self._final_summary_logged = True

    def _sensing_summary_checkpoint(self, now, index=None) -> None:
        """Final summaries at the terminal epoch, running ones every
        SUMMARY_EVERY_STEPS steps. Lockstep calls it after sensing and
        before the barrier wait, which the terminal step never gets past,
        counting by step_index; free-run passes its own iteration count
        as ``index``, since its step_index stays 0."""
        from stk_bridge.core.orbit_block import epoch_ms

        if index is None:
            index = self.time_sync.step_index

        if (getattr(self, "sensing", None) is None
                or getattr(self, "_final_summary_logged", False)):
            return
        stop = getattr(self, "_scenario_stop", None)
        if stop is not None and epoch_ms(now) >= epoch_ms(stop):
            self._sample_graphics("end", index)
            self._log_sensing_summaries(final=True)
        elif index % SUMMARY_EVERY_STEPS == 0:
            self._sample_graphics("running", index)
            self._log_sensing_summaries(final=False)

    def _sample_graphics(self, phase: str, index=None) -> None:
        """Record STK's graphics state (startup, running or end).

        A read that fails is recorded as "unavailable" with its error,
        which the runner refuses like any other state that is not the
        one the run requires.
        """
        log = getattr(self, "_graphics_log", None)
        if log is None:
            return
        error = ""
        try:
            state = self.worker.call(
                lambda backend: backend.graphics_state(), "graphics state")
        except Exception as exc:            # noqa: BLE001 - recorded
            state, error = None, str(exc)
        step = (getattr(self.time_sync, "step_index", 0)
                if index is None else index)
        entry = log.add(phase, state, step, error)
        LOGGER.info("STK graphics (%s): %s", phase, describe(entry))

    def _attach_orbit_batch(self) -> None:
        """(C): lockstep only, and only with a stop time STK confirms."""
        from stk_bridge.core.orbit_batch import OrbitBatch, stop_time_problem
        from stk_bridge.core.time_sync import ExecutionMode

        config = self.sensing_config
        if self.time_sync.mode is not ExecutionMode.LOCKSTEP:
            LOGGER.warning("sensing.orbit_batch_samples is %d but the bridge "
                           "is in %s mode; orbit batching is lockstep-only "
                           "and stays off", config.orbit_batch_samples,
                           self.time_sync.mode.name)
            return
        stop = self._scenario_stop
        problem = stop_time_problem(self.scenario.epoch, stop,
                                    config.orbit_batch_expected_window_s)
        if problem:
            LOGGER.warning("orbit batching disabled for this run: %s",
                           problem)
            return
        self.sensing.attach_orbit_batch(OrbitBatch(
            samples=config.orbit_batch_samples,
            step_s=self.time_sync.config.step_s, stop_time=stop,
            audit_position_m=config.orbit_batch_audit_position_m,
            audit_velocity_mps=config.orbit_batch_audit_velocity_mps))
        LOGGER.info("orbit batching on: %d-epoch blocks before the stop "
                    "time %s, audited to %.0e m / %.0e m/s",
                    config.orbit_batch_samples, stop,
                    config.orbit_batch_audit_position_m,
                    config.orbit_batch_audit_velocity_mps)
        if config.equivalence_diagnostic:
            LOGGER.warning("the equivalence diagnostic compares orbit block "
                           "rows with themselves while batching is on; run "
                           "it with orbit_batch_samples: 1 and "
                           "combine_attitude_rate: false")

    def _attach_equivalence_diagnostic(self) -> None:
        """Opt-in, observe-only; see core/equivalence_diagnostic.py."""
        from stk_bridge.core.equivalence_diagnostic import (
            EquivalenceDiagnostic,
        )
        from stk_bridge.core.time_sync import ExecutionMode

        if self.time_sync.mode is not ExecutionMode.LOCKSTEP:
            LOGGER.warning("sensing.equivalence_diagnostic is set but the "
                           "bridge is in %s mode; the diagnostic compares "
                           "lockstep epochs only and stays off",
                           self.time_sync.mode.name)
            return
        stop = self._scenario_stop
        self._diagnostic = EquivalenceDiagnostic(
            self.sensing_config.equivalence_diagnostic_dir,
            epoch=self.scenario.epoch, stop_time=stop,
            step_s=self.time_sync.config.step_s)
        self.sensing.attach_diagnostic(self._diagnostic)
        LOGGER.warning(
            "EQUIVALENCE DIAGNOSTIC ON: published values are unchanged, "
            "but extra queries inflate every timing -- this run is not an "
            "E6 measurement. Writing to %s (scenario stop time %s)",
            self.sensing_config.equivalence_diagnostic_dir, stop)

    def _build_interfaces(self, qos_profiles: Dict[str, object]) -> None:
        from rclpy.callback_groups import (
            MutuallyExclusiveCallbackGroup,
            ReentrantCallbackGroup,
        )
        from stk_bridge.interfaces.ros_actions import ActionServers
        from stk_bridge.interfaces.ros_publishers import SatellitePublishers
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

        if self.bridge_publishers is None:
            self.bridge_publishers = self._make_bridge_publishers(
                qos_profiles)

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

    def _make_bridge_publishers(self, qos_profiles):
        from stk_bridge.interfaces.ros_publishers import BridgePublishers

        return BridgePublishers(
            self.node, qos_profiles, self._prefix,
            publish_clock=self.time_sync.config.publish_clock)

    def _publish_starting(self, phase: str) -> None:
        """BridgeStatus STARTING, while startup is still running.

        The message has no field for the phase; it is in
        startup_phases_file, which the runner reads.
        """
        from stk_bridge.core.health import BridgeState

        if (self.bridge_publishers is None or self.startup.finished
                or self.health.state != BridgeState.STARTING):
            return
        self.bridge_publishers.publish_status(
            state=int(BridgeState.STARTING),
            backend_type=self.worker.backend.backend_type.value,
            stk_version="", dropped={}, queue_depth=0,
            queue_capacity=self.worker.queue_capacity, latency_ms=0.0,
            last_error="")

    # -- shutdown ----------------------------------------------------------

    def stop(self) -> None:
        LOGGER.info("stopping STK bridge")
        self._running.clear()
        self._step_request.set()
        if self._loop_thread is not None:
            self._loop_thread.join(timeout=5.0)
            self._loop_thread = None
        self.worker.stop(timeout_s=10.0)
        if (self.sensing is not None
                and not getattr(self, "_final_summary_logged", False)):
            # Only if the terminal epoch did not already log them.
            self._log_sensing_summaries(final=True)
        if getattr(self, "_diagnostic", None) is not None:
            self._diagnostic.close()

    # -- clock -------------------------------------------------------------

    def _advance_stk_clock(self, when) -> None:
        """Push simulation time into STK.

        Fire-and-forget: the step loop must not block on a COM call to
        set the animation time, since that would couple step rate to
        COM latency. If the queue is full the sample is dropped and the
        next step's time subsumes it -- animation time is absolute, not
        incremental, so a dropped update is self-correcting.
        """
        timing = getattr(self, "_clock_timing", None)

        def advance(backend):
            # Timed where it executes: on the worker, ahead of this step's
            # sensing queries -- so inside E6's sense phase. Reporting only.
            started = time.perf_counter()
            try:
                backend.set_current_time(when)
            finally:
                if timing is not None:
                    timing.record((time.perf_counter() - started) * 1000.0)

        self.worker.submit(advance, "set_current_time")

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
            except (LockstepBarrierTimeoutError, LockstepCommandError) as exc:
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

        # A barrier ack can arrive before the command on another DDS topic.
        # The first step has no preceding telemetry/controller output.
        time_config = self.config.get("time") or {}
        required = []
        if (self.time_sync.step_index > 0 and
                time_config.get("require_attitude_command_each_step", False)):
            required = ["attitude"]
            stamp = self.time_sync.now() + (step_size or self.time_sync.config.step_s)
            missing = self.inbox.wait_for_commands(
                required, self.sensing_config.satellites, stamp,
                self.time_sync.config.barrier_timeout_s)
            if missing:
                raise LockstepCommandError(self.time_sync.step_index + 1,
                                           f"missing commands: {missing}")
        # 1. Apply commands computed for the previous step.
        with self._timings.timed("actuate"):
            result = self._apply_commands()
            if required and (not result or any(
                    result["rejected"].get(channel, 0) or
                    result["applied"].get(channel, 0) < len(self.sensing_config.satellites)
                    for channel in required)):
                raise LockstepCommandError(self.time_sync.step_index + 1,
                                           "required command was not accepted by STK")
        # 2. Advance. Not one of E6's three reported phases (it is STK's
        #    own clock update), but timed for the log: it is part of what
        #    a step costs.
        advance_started = time.perf_counter()
        now = self.time_sync.step(step_size or None)
        total, count = self._advance_ms
        self._advance_ms = (
            total + (time.perf_counter() - advance_started) * 1000.0,
            count + 1)
        # 3. Publish the new state and the clock, so participants have
        #    what they need before being asked to acknowledge.
        self._publish_clock(now)
        with self._timings.timed("sense"):
            self._sense_and_publish(now)
        self._run_rate_groups()
        self._publish_status(now)
        self._sensing_summary_checkpoint(now)
        # 4. Wait for acknowledgement. Timed for E6, though this
        #    iteration's wait lands in the NEXT status publish's window
        #    -- see core/step_timing.py's module docstring for why that
        #    one-window lag is immaterial to a multi-minute mean.
        #    A step-1 wait that needed the startup retry is NOT recorded:
        #    it is minutes of launch latency (a human starting MATLAB,
        #    the scheduler's process start), and as one sample in a
        #    ~7200-step run it would add several ms to the reported
        #    mean barrier wait -- the order of the quantity E6 measures.
        #    While waiting at step 1 for late participants, the step is
        #    republished every STARTUP_REPUBLISH_S rather than once per
        #    barrier timeout: a participant that joins late (the
        #    scheduler, started only once the twin acks) sees nothing
        #    until the next republish, and at a 30 s barrier timeout that
        #    was ~30 s of dead time per run.
        started = time.perf_counter()
        waited_for_startup = False
        startup_wait = (
            bool((self.config.get("time") or {}).get(
                "wait_for_startup_participants", False))
            and self.time_sync.step_index == 1)
        while self._running.is_set():
            try:
                self.time_sync.await_barrier(
                    timeout_s=STARTUP_REPUBLISH_S if startup_wait else None)
                break
            except LockstepBarrierTimeoutError as exc:
                if not startup_wait:
                    raise
                # Republish the SAME initial step for late joiners. Never
                # reset the barrier or advance STK while MATLAB starts.
                waited_for_startup = True
                wall = time.monotonic()
                if wall - self._last_startup_log >= 30.0:
                    self._last_startup_log = wall
                    LOGGER.info("waiting at initial step for lockstep "
                                "participants: %s", exc)
                self._publish_clock(now)
                # Same epoch: replay its original samples, without COM work
                # competing with the scheduler's startup geometry queries.
                for path, samples in getattr(self, '_startup_samples', {}).items():
                    self.publishers[path].publish_samples(samples)
                self._publish_status(now)
        # 5. The barrier is released, but the step is not complete until
        #    each commanded object's attitude for the NEXT step is here:
        #    see _await_attitude_commands. Part of the barrier wait, since
        #    it is waiting on a participant.
        self._await_attitude_commands(now)
        if not waited_for_startup:
            self._timings.record(
                "barrier_wait", (time.perf_counter() - started) * 1000.0)

    def _await_attitude_commands(self, now) -> None:
        """Lockstep: wait for each commanded object's next-step attitude.

        A participant publishes its attitude command and its barrier ack
        on two topics, and ROS 2 does not order delivery across topics.
        When the ack wins, the barrier releases, the bridge steps and
        senses before the command lands, and STK -- holding no sample at
        that time -- reports its BASIC attitude (nadir) instead: a jump of
        up to 180 deg in one step. The twin then integrated from that
        jump. On the third live run this happened on 7% of steps and spun
        the vehicle up to ~800 deg/s. Waiting here removes the race
        whatever order the two messages arrive in.

        Only objects that have been commanded this session are waited
        for, so the first step (before any command) and objects nobody
        commands are unaffected. Enabled by
        ``time.await_attitude_commands``.
        """
        section = self.config.get("time") or {}
        if not section.get("await_attitude_commands", False):
            return
        if not self._commanded_paths or not self._running.is_set():
            return
        step_s = self.time_sync.config.step_s
        wanted_s = now.epoch_seconds + 0.5 * step_s
        timeout_s = float(section.get("command_wait_s", 5.0))
        for path in sorted(self._commanded_paths):
            if self.inbox.wait_for("attitude", path, wanted_s, timeout_s):
                continue
            self._commands_missing += 1
            self._warn_limited(
                "command-missing", logging.WARNING,
                "no attitude command for %s stamped for the next step "
                "arrived within %.1f s of the barrier releasing; STK will "
                "report its basic attitude at t=%.3f s instead of the "
                "twin's", path, timeout_s,
                now + step_s - self.time_sync.epoch)

    def _arm_startup_gate(self, epoch) -> None:
        """Start the clock; in free-run, held at t = 0 behind a gate.

        Free-run time follows the wall clock, so without the gate the
        window ran on through the MATLAB handoff and the scheduler's
        target resolution. With time.wait_for_startup_participants and
        declared participants, a free-run clock starts paused at exactly
        t = 0 and _free_gate_iteration opens it once all have acked.
        Lockstep starts exactly as before (its own startup wait is in
        _lockstep_iteration).
        """
        from stk_bridge.core.time_sync import ExecutionMode

        wait = bool((self.config.get("time") or {}).get(
            "wait_for_startup_participants", False))
        participants = self.time_sync.barrier_participants()
        gated = (self.time_sync.mode is ExecutionMode.FREE_RUN and wait
                 and bool(participants))
        self.time_sync.start(epoch, paused=gated)
        if gated:
            self._free_gate = {"started": time.monotonic(),
                               "published": False, "last_publish": 0.0}
            LOGGER.info("free-run start gate: clock held at t = 0 until %s "
                        "have acked", participants)

    def _free_gate_iteration(self, gate) -> None:
        """Hold t = 0: sense once, then republish without COM work until
        every participant has acked, then resume the clock from t = 0."""
        now = self.time_sync.now()
        just_published = not gate["published"]
        if just_published:
            self._publish_clock(now)
            self._free_gate_capturing = True
            try:
                self._sense_and_publish(now)
            finally:
                self._free_gate_capturing = False
            self._publish_status(now)
            gate["published"] = True
            gate["last_publish"] = time.monotonic()
        missing = self.time_sync.missing_acks()
        wall = time.monotonic()
        if not missing:
            waited = wall - gate["started"]
            self.time_sync.resume()
            self._free_gate = None
            participants = self.time_sync.barrier_participants()
            LOGGER.info("free-run start gate opened after %.1f s: %s acked; "
                        "the clock runs from t = 0", waited, participants)
            self._write_startup_gate({
                "opened": True, "wait_s": waited,
                "participants": participants,
                "opened_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                            time.gmtime())})
            return
        if (not just_published
                and wall - gate["last_publish"] >= STARTUP_REPUBLISH_S):
            gate["last_publish"] = wall
            self._publish_clock(now)
            for path, samples in getattr(self, "_startup_samples", {}).items():
                self.publishers[path].publish_samples(samples)
            self._publish_status(now)
        if wall - self._last_startup_log >= 30.0:
            self._last_startup_log = wall
            LOGGER.info("free-run start gate: waiting at t = 0 for %s",
                        missing)
        time.sleep(FREE_RUN_GATE_POLL_S)

    def _apply_free_run_sensing_policy(self) -> None:
        """Free-run: (B) explicitly off, whatever the config says.

        (B) verifies only at an epoch where this bridge wrote an inertial
        sample, which a free-run sense epoch never is, so it would stay
        "pending" all run. Lockstep is untouched.
        """
        from stk_bridge.core.time_sync import ExecutionMode

        if (self.time_sync.mode is ExecutionMode.FREE_RUN
                and self.sensing.combine_state != "off"):
            reason = ("in free-run: it verifies only at an epoch where an "
                      "inertial sample was written, which a free-run sense "
                      "epoch never is; attitude and rate are read separately")
            self.sensing.disable_combine(reason)
            LOGGER.info("combined attitude/rate (B) is off %s", reason)

    def _write_startup_gate(self, record) -> None:
        self._write_json_file("startup_gate_file", record)

    def _write_json_file(self, config_key, record) -> None:
        import json

        path = str(self.config.get(config_key) or "")
        if not path:
            return
        temporary = f"{path}.partial"
        with open(temporary, "w", encoding="utf-8") as handle:
            json.dump(record, handle, indent=2)
            handle.write("\n")
        os.replace(temporary, path)

    def _free_running_iteration(self) -> None:
        gate = getattr(self, "_free_gate", None)
        if gate is not None:
            self._free_gate_iteration(gate)
            return
        # No barrier wait outside lockstep -- see BridgeStatus.msg's own
        # field comment: step_samples tells a consumer that phase was
        # never measured this run rather than implying a suspiciously
        # fast zero.
        pacer = self._free_run_pacer()
        scheduled = (pacer.begin(self.time_sync.sim_seconds())
                     if pacer is not None else None)
        now = self.time_sync.now()
        with self._timings.timed("actuate"):
            self._apply_commands()
        with self._timings.timed("sense"):
            self._sense_and_publish(now)
        # After sensing, with the same `now`: the twin steps only when the
        # telemetry it holds is stamped with the latest ScenarioTime, so a
        # clock published before sensing left it unable to step for a
        # whole sensing time per cycle (E3 at rtf 1.0 on 4b26de0: 31 % of
        # samples answered). Lockstep and the start gate keep their order.
        self._publish_clock(now)
        # Marked at the cycle's scheduled start, so the slower groups keep
        # their configured rates.
        self._run_rate_groups(scheduled)
        self._publish_status(now)
        # Summaries and graphics samples, as in lockstep; counted by
        # iteration because free-run time comes from the wall clock.
        self._free_iterations = getattr(self, "_free_iterations", 0) + 1
        self._sensing_summary_checkpoint(now, index=self._free_iterations)
        if pacer is None:
            self.time_sync.sleep_until_next(
                self.scheduler.next_due_time(self.time_sync.sim_seconds()))
        else:
            # Fixed-rate schedule from the cycle start: period =
            # max(work, period); an overrun starts the next cycle at once.
            self.time_sync.sleep_until_next(
                pacer.end(self.time_sync.sim_seconds()))

    def _free_run_pacer(self):
        """The free-run pacer at the fastest rate group's period, or None
        when no group is enabled (then the old next-due sleep applies)."""
        pacer = getattr(self, "_free_pacer", None)
        if pacer is None:
            from stk_bridge.core.free_run_pacer import FreeRunPacer

            period = self.scheduler.fastest_period_s()
            if period is None:
                return None
            pacer = self._free_pacer = FreeRunPacer(period)
        return pacer

    # -- loop stages -------------------------------------------------------

    def _apply_commands(self):
        from stk_bridge.interfaces.ros_subscribers import apply_commands

        commands = self._screen_command_times(self.inbox.drain())
        if not commands:
            return
        self._commanded_paths.update(
            conv.canonical_object_path(command.object_path)
            for channel, command in commands if channel == "attitude")
        try:
            result = self.worker.call(
                lambda backend: apply_commands(self.actuation, commands),
                "apply_commands")
            self.health.record_success()
            return result
        except Exception as exc:
            LOGGER.warning("failed to apply %d command(s): %s",
                           len(commands), exc)
            self.health.record_error(str(exc))

    def _screen_command_times(self, commands):
        """Lockstep only: refuse commands stamped far from simulation time.

        A command's stamp is the STK sample time it is written at. In
        lockstep every legitimate command is for the next step, so a stamp
        minutes (or years) away is a participant stamping with the wrong
        clock -- and STK accepts such a sample without complaint, so
        applying it would leave the attitude inside the scenario
        untouched for the whole run while everything looked healthy.
        A command stamped at or before the current step is kept, since it
        still fixes STK's attitude history, but counted: it arrived after
        the barrier released, so its step was sensed without it.
        """
        from stk_bridge.core.time_sync import ExecutionMode

        if not commands or self.time_sync.mode is not ExecutionMode.LOCKSTEP:
            return commands
        now_s = self.time_sync.now().epoch_seconds
        step_s = self.time_sync.config.step_s
        limit_s = float((self.config.get("time") or {}).get(
            "max_command_offset_s", 60.0))
        kept = []
        for channel, command in commands:
            offset_s = command.time.epoch_seconds - now_s
            if abs(offset_s) > limit_s:
                self._commands_rejected += 1
                self._warn_limited(
                    "command-offset", logging.ERROR,
                    "rejected %s command for %s stamped %+.1f s from "
                    "simulation time (limit %.0f s). Lockstep commands "
                    "must carry scenario time; a wall-clock stamp is the "
                    "usual cause.", channel, command.object_path,
                    offset_s, limit_s)
                continue
            if offset_s < 0.5 * step_s:
                self._commands_late += 1
                self._warn_limited(
                    "command-late", logging.WARNING,
                    "%s command for %s stamped %+.3f s from the current "
                    "step: it arrived after the barrier released, so the "
                    "step it was for was sensed without it.",
                    channel, command.object_path, offset_s)
            kept.append((channel, command))
        return kept

    def _check_attitude_landed(self, path, now, samples) -> None:
        """Lockstep: is STK's attitude at ``now`` the one written for it?

        Once an object is commanded, STK should report exactly the sample
        written for this step. Anything else -- no command for this step,
        or a read-back that differs from it -- means the controller's
        next step starts from an attitude it never commanded, which is
        silent unless something says so.
        """
        from stk_bridge.core.time_sync import ExecutionMode

        if (self.time_sync.mode is not ExecutionMode.LOCKSTEP
                or path not in self._commanded_paths
                or "attitude" not in samples):
            return
        written = self.actuation.last_attitude(path)
        sample = samples["attitude"]
        t_s = now - self.time_sync.epoch
        if written is None or abs(written[0] - now.epoch_seconds) > 1e-6:
            self._attitude_mismatches += 1
            self._warn_limited(
                "attitude-unwritten", logging.WARNING,
                "no attitude was written to STK for %s at t=%.3f s before "
                "it was sensed; STK reports its basic attitude there, not "
                "the controller's", path, t_s)
            return
        try:
            inertial_read = self.frames.is_inertial_attitude_reference(
                sample.frame)
        except Exception:           # unknown frame: cannot compare
            return
        if inertial_read != written[2]:
            return                  # different frames: not comparable
        error_deg = math.degrees(conv.quaternion_angle_between(
            written[1], sample.quaternion_xyzw))
        if error_deg > 0.1:
            self._attitude_mismatches += 1
            self._warn_limited(
                "attitude-mismatch", logging.WARNING,
                "STK reports an attitude for %s at t=%.3f s %.2f deg away "
                "from the one written for that time", path, t_s, error_deg)

    def _retry_missing_channels(self, path, now, samples):
        """Lockstep only: re-read fast channels that a sensing pass missed.

        A lockstep participant that needs all three fast channels for a
        step (the ACAV twin does -- it waits for attitude, orbit and rate
        stamped with the same step) never acks a step where one of them
        failed to read, and the run aborts 30 s later naming the
        participant. One retry absorbs a transient STK hiccup; a channel
        still missing after it is reported rather than left silent.
        """
        from stk_bridge.core.time_sync import ExecutionMode

        if (not samples
                or self.time_sync.mode is not ExecutionMode.LOCKSTEP):
            return samples
        missing = [c for c in self.sensing.FAST_CHANNELS if c not in samples]
        if not missing:
            return samples
        try:
            retried = self.worker.call(
                lambda backend, p=path, m=missing: self.sensing.read_all(
                    p, now, channels=m),
                f"sense-retry({path})")
        except Exception as exc:
            retried = {}
            LOGGER.debug("sensing retry failed on %s: %s", path, exc)
        samples = {**samples, **(retried or {})}
        still = [c for c in missing if c not in samples]
        if still:
            self._warn_limited(
                "sense-partial", logging.WARNING,
                "no %s for %s at t=%.3f s even after a retry%s",
                "/".join(still), path, now - self.time_sync.epoch,
                self._lockstep_hint())
        return samples

    def _lockstep_hint(self) -> str:
        from stk_bridge.core.time_sync import ExecutionMode

        if self.time_sync.mode is not ExecutionMode.LOCKSTEP:
            return ""
        return ("; nothing was published for this step, so a lockstep "
                "participant waiting on this telemetry will not ack it")

    def _warn_limited(self, key: str, level: int, message: str,
                      *args) -> None:
        """Log the 1st, 10th, 100th, ... occurrence of one kind of problem.

        Enough to see that it happens and how often, without one line
        per step for a two-hour run.
        """
        count = self._warn_counts.get(key, 0) + 1
        self._warn_counts[key] = count
        if str(count).rstrip("0") == "1":
            LOGGER.log(level, message + " [occurrence %d]", *args, count)

    def _log_timing_summary(self, timings) -> None:
        """Every TIMING_LOG_PERIOD_S, log the step rate and E6's breakdown."""
        acc = self._timing_log
        samples = int(timings.get("step_samples", 0))
        for phase in ("sense", "actuate", "barrier_wait"):
            acc[phase] += timings.get(f"mean_{phase}_ms", 0.0) * samples
        acc["samples"] += samples
        wall = time.monotonic()
        elapsed = wall - acc["wall"]
        if elapsed < TIMING_LOG_PERIOD_S:
            return
        step = self.time_sync.step_index
        n = max(acc["samples"], 1)
        channels = getattr(self.sensing, "channel_timings", None) or {}
        per_channel = ", ".join(
            f"{name} {total / max(count, 1):.0f}"
            for name, (total, count) in sorted(channels.items()))
        advance_total, advance_count = self._advance_ms
        execute_total, execute_count = (
            self._clock_timing.take_window()
            if getattr(self, "_clock_timing", None) is not None
            else (0.0, 0))
        LOGGER.info(
            "step %d: %.2f steps/s over the last %.0f s; per step: sense "
            "%.1f ms (per read, ms: %s), actuate %.1f ms, barrier wait "
            "%.1f ms, clock advance submit %.1f ms, clock advance execute "
            "%.1f ms (inside sense); commands late %d, rejected %d, "
            "missing %d; attitude not as written %d (run totals)",
            step, (step - acc["step"]) / elapsed, elapsed,
            acc["sense"] / n, per_channel or "n/a", acc["actuate"] / n,
            acc["barrier_wait"] / n,
            advance_total / max(advance_count, 1),
            execute_total / max(execute_count, 1),
            self._commands_late, self._commands_rejected,
            self._commands_missing, self._attitude_mismatches)
        if channels:
            channels.clear()
        self._advance_ms = (0.0, 0)
        self._timing_log = {"wall": wall, "step": step, "sense": 0.0,
                            "actuate": 0.0, "barrier_wait": 0.0,
                            "samples": 0}

    def _sense_and_publish(self, now) -> None:
        for path, publishers in self.publishers.items():
            try:
                samples = self.worker.call(
                    lambda backend, p=path: self.sensing.read_all(p, now),
                    f"sense({path})")
            except Exception as exc:
                self._warn_limited(
                    "sense-failed", logging.WARNING,
                    "sensing failed on %s at t=%.3f s: %s%s", path,
                    now - self.time_sync.epoch, exc, self._lockstep_hint())
                self.health.record_error(str(exc))
                continue

            samples = self._retry_missing_channels(path, now, samples)
            self._check_attitude_landed(path, now, samples)

            if not samples:
                # No state at this epoch. Publishing nothing is
                # deliberate -- see ros_publishers' module docstring --
                # but in lockstep a participant waiting for this step's
                # telemetry then never acks, and the barrier timeout
                # names the participant rather than the missing data.
                self._warn_limited(
                    "sense-empty", logging.WARNING,
                    "STK returned no state for %s at t=%.3f s (past the "
                    "scenario's analysis period?)%s", path,
                    now - self.time_sync.epoch, self._lockstep_hint())
                continue

            if (self.time_sync.step_index == 1
                    or getattr(self, "_free_gate_capturing", False)):
                if not hasattr(self, '_startup_samples'):
                    self._startup_samples = {}
                self._startup_samples[path] = samples
            publishers.publish_samples(samples)
            if self.tf is not None:
                self.tf.broadcast_samples(path, samples)
            self.health.record_success()

        self.health.update_from_sensing(self.sensing.statistics())

    def _run_rate_groups(self, sim_time_s=None) -> None:
        # Lockstep passes nothing: the step's own simulation time, as
        # before. Free-run passes its cycle's scheduled start.
        self.scheduler.run_due(
            self.time_sync.sim_seconds() if sim_time_s is None
            else sim_time_s,
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
        self._log_timing_summary(timings)
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
            attitude_provider=(self.sensing.attitude_provider
                               if self.sensing is not None else ""),
            attitude_provider_forced=(self.sensing is not None
                                      and self.sensing.attitude_provider_forced),
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


def check_attitude_pairing(report, config: Dict) -> None:
    """Refuse to run commanding in one attitude frame and reading another.

    Sensing reads attitude through the provider validation reports;
    actuation commands in the form ``actuation.default_attitude_inertial``
    configures. Validation prefers the configured form's pairing, so the
    two only disagree when that pairing failed on this install -- and
    running anyway would read back every commanded attitude rotated by
    the Earth-rotation angle, silently.
    """
    if not report.attitude_provider:
        return                      # validation disabled; it already warned
    configured = bool((config.get("actuation") or {}).get(
        "default_attitude_inertial", False))
    if bool(report.attitude_command_inertial) == configured:
        return
    wanted = "AddAttitude Quat (inertial)" if configured \
        else "AddAttitude CBFQuat (Earth-fixed)"
    raise ConventionValidationError(
        f"actuation is configured to command attitude with {wanted}, but "
        f"validation paired the read-back with {report.attitude_provider!r}, "
        "which belongs to the other command form -- the configured "
        "pairing did not round-trip on this install. Commanding and "
        "reading in different frames is wrong by the Earth-rotation angle "
        "with nothing raising, so the bridge will not start. See the "
        "quaternion_ordering result above.")


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

    aborted = False
    try:
        bridge.start()
        while rclpy.ok() and bridge._running.is_set():
            executor.spin_once(timeout_sec=0.5)
        aborted = rclpy.ok() and not bridge._running.is_set()
    except KeyboardInterrupt:
        LOGGER.info("interrupted")
    except Exception:
        LOGGER.exception("bridge stopped with an error")
        raise
    finally:
        bridge.stop()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if aborted:
        LOGGER.error("bridge stopped: the step loop aborted (see the error "
                     "logged above)")
        sys.exit(1)


if __name__ == "__main__":
    main()
