"""The ACAV scheduler: a ROS 2 node driving one closed-loop E2 run.

    ros2 run acav acav_scheduler --ros-args \\
        -p scheduler_type:=dynamics_aware \\
        -p output_json:=/tmp/run_dynamics_aware.json

This node is the "target scheduler" half of the ACAV architecture
(``docs/C1_case_study_spec.md``): it decides WHICH target the vehicle
should be pointed at and WHEN (``acav/scheduling.py``, entirely offline
logic), tells the twin via ``/acav/setpoint``, and scores what actually
happened against ``acav/metrics.py``'s capture criterion using the
twin's own telemetry plus STK's access answer, obtained from the bridge
it is a client of.

It is also a lockstep barrier participant. During startup it withholds
its first acknowledgement until target trajectories have been resolved,
one asynchronous query at a time. The bridge remains at its initial epoch
and replays cached telemetry. Once geometry is ready, normal ticks publish
the setpoint and acknowledge without blocking on STK. Access queries remain
asynchronous during the run.

Design decisions worth being explicit about
---------------------------------------------
**Who computes the pointing error.** The twin does, in
``/acav/twin_status`` -- not this node, independently, from raw
telemetry. The twin already has the freshest (satellite state, target
position) pair at the instant it computes ``boresight_error_rad``, and
a second, independently-written implementation of the same angle here
would risk exactly the failure C1 warns about for the slew model: two
models measuring their own disagreement instead of the effect under
study.

**Who supplies target geometry.** This node queries each attempted
target's Cartesian Position (ICRF) from the bridge's own
``QueryDataProvider`` service, once, at start-of-run -- not by
re-deriving an Earth-rotation model independently. Each target is
static in STK's Fixed frame, so one query per target (at its own image
epoch, since a target's *inertial* position drifts with Earth's
rotation) is enough for the whole run; there is no ongoing per-step
STK traffic this node introduces.

**Who decides "was there access".** STK does, once, shortly after a
target's image epoch has passed and the real-time attitude history up
to it has therefore been committed -- via ``ComputeAccess`` on the
(sensor, target) pair. Querying earlier would ask a question STK
cannot yet answer.
"""

from __future__ import annotations

import json
import logging
import math
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

from stk_bridge.backends.base import StkTime
from stk_bridge.core import conversions as conv
from stk_bridge.core import data_providers as dp
from stk_bridge.core.actuation import ATTITUDE_NOT_COVERED
from stk_bridge.exceptions import StkBridgeError

from acav.capture_tracking import CaptureTracker, LatencyAggregate, \
    RunAggregate, TelemetrySample, access_unverified_result, \
    declined_result, no_access_answer_result, no_telemetry_result
from acav.metrics import CaptureOutcome, RunResult, TargetResult
from acav.scheduling import ScheduleEntry, SchedulePlan, plan
from acav.spec import AcavSpec

LOGGER = logging.getLogger(__name__)

#: Additional seconds of telemetry to keep buffered beyond
#: pointing.settle_hold_s, so the tracker's retention window is not on
#: the hairline against the hold window it has to score.
RETENTION_MARGIN_S = 5.0
TARGET_POSITION_STEP_S = 10.0  # Earth-rotation interpolation error below 0.5 m.


def _between_publishes(message) -> dict:
    """TwinStatus's saturation count and largest error since the previous
    status, or None for each from a message without them."""
    events = getattr(message, "saturation_events", None)
    worst = getattr(message, "max_boresight_error_since_last_rad", None)
    return {"saturation_events": None if events is None else int(events),
            "max_error_since_last_deg": (None if worst is None
                                         else math.degrees(worst))}


def interpolate_target_position(samples, time_s):
    """ICRF ground-target position at the current simulation epoch."""
    u = max(0.0, min(time_s / TARGET_POSITION_STEP_S, len(samples) - 1))
    i = int(u)
    j = min(i + 1, len(samples) - 1)
    return tuple(a + (u-i) * (b-a) for a, b in zip(samples[i], samples[j]))

#: Delay, seconds, after a target's image epoch before this node asks
#: STK whether it had access at that epoch. STK's real-time attitude
#: injection for that instant has already happened by the time the
#: local sim clock reaches it; this margin is for the ScenarioTime tick
#: sequencing, not for STK, and one extra step is generous slack.
DEFAULT_ACCESS_QUERY_DELAY_S = 0.2

#: How long past the window's own end to keep waiting for outstanding
#: access answers before finalising the run anyway (see ``_finish_run``).
#: WALL seconds, not simulation seconds: the STK scenario ends at the
#: window's end, so the bridge cannot step past it, and a grace period
#: counted in simulation time never expires -- the first live run waited
#: for step 7201, never got it, and wrote no record at all.
FINALIZE_GRACE_WALL_S = 15.0

#: A TwinStatus stamped further than this from the scenario's current
#: time is not late, it is on the wrong clock (see ``_on_twin_status``).
TWIN_STAMP_TOLERANCE_S = 5.0


def _code_version() -> str:
    from acav.code_version import describe

    return describe()


def _utc_now() -> str:
    """An ISO-8601 UTC timestamp, for the run's provenance."""
    import datetime

    return datetime.datetime.now(datetime.timezone.utc).isoformat(
        timespec="seconds")


def _execution_mode_name(value: int) -> str:
    """ScenarioTime.execution_mode as a name, via the bridge's own enum.

    Routed through ``core/time_sync.ExecutionMode`` rather than a local
    table so there is one mapping from those constants in the codebase.
    A second copy is exactly how a renamed mode ends up recorded
    correctly in one place and wrongly in another.
    """
    try:
        from stk_bridge.core.time_sync import ExecutionMode

        return ExecutionMode(int(value)).name.lower()
    except (ImportError, ValueError):
        LOGGER.warning("unrecognised execution mode %r", value)
        return "unknown"


def _parse_json_object(text: str, what: str) -> dict:
    value = _parse_json_value(text, what)
    return value if isinstance(value, dict) else {}


def _parse_json_value(text: str, what: str):
    """Parse a JSON argument, or warn and record nothing.

    Warns rather than raises: a malformed ``--fault_json`` should not
    destroy a run that has already executed against live STK. It is
    recorded as absent, and the runner's own check then reports the
    record as not matching the run it asked for -- which is the right
    place for that to surface.
    """
    if not text:
        return None
    try:
        return json.loads(text)
    except ValueError as error:
        LOGGER.warning("ignoring malformed %s argument %r: %s",
                       what, text, error)
        return None


@dataclass
class SchedulerOptions:
    """Everything about a run that is not scenario data.

    Scenario data (targets, tolerances, the barrier participant name)
    comes from ``AcavSpec`` / ``acav_scenario.yaml`` -- these are the
    run-time choices that make one invocation "the kinematic run" and
    another "the dynamics-aware run" of the SAME scenario, which is
    exactly what E2 compares.
    """

    scheduler_type: str = "dynamics_aware"
    prefix: str = "stk"
    participant_name: str = "acav_scheduler"
    setpoint_topic: str = "/acav/setpoint"
    twin_status_topic: str = "/acav/twin_status"
    output_json: str = ""
    access_query_delay_s: float = DEFAULT_ACCESS_QUERY_DELAY_S
    auto_shutdown: bool = True
    run_id: str = "run"
    seed: Optional[int] = None
    #: Which experiment this run belongs to, e.g. "E5". Recorded so a
    #: directory of results says what it is without a spreadsheet.
    experiment: str = ""
    #: Path to the scenario as loaded, hashed into the run's provenance.
    #: Two runs claiming to be the same experiment while reading
    #: different scenarios is a contradiction worth being able to detect.
    scenario_path: str = ""
    #: Controller gains and injected fault in force for this run, as
    #: JSON, supplied by whatever launched it. These are recorded as
    #: CONFIGURED -- unlike the backend, which is observed from the
    #: bridge, because a backend can silently differ from what was asked
    #: for and gains cannot.
    gains_json: str = ""
    fault_json: str = ""
    #: E1's treatment, recorded so a mis-paired run is identifiable in
    #: the results rather than only in the launch command's history.
    attitude_provider: str = ""
    #: The twin identity the run orchestrator accepted, for the record.
    twin_identity: str = ""


class SchedulerNode:
    """ROS-facing wiring around the pure scheduling/capture logic.

    Constructed with an already-created ``node`` (dependency injection,
    matching ``nodes/clock_node.py``'s convention) so the class itself
    never imports ``rclpy``/``stk_msgs`` at module scope -- only lazily,
    inside methods -- and so a future test could hand it a fake node
    without a sourced ROS environment. The substantial logic it calls
    into (``acav.scheduling``, ``acav.capture_tracking``) already is
    tested that way, in ``test_scheduling.py`` / ``test_capture_tracking.py``.
    """

    def __init__(self, node, spec: AcavSpec, options: SchedulerOptions):
        from std_msgs.msg import String
        from stk_msgs.msg import (
            BridgeStatus,
            ScenarioTime,
            TargetSetpoint,
            TwinStatus,
        )
        from stk_msgs.srv import ComputeAccess, QueryDataProvider

        from stk_bridge.interfaces import qos as qos_module

        self._node = node
        self._spec = spec
        self._options = options
        self._plan: SchedulePlan = plan(spec, options.scheduler_type)

        if options.participant_name not in spec.bridge.get(
                "barrier_participants", []):
            LOGGER.warning(
                "this node's participant_name %r is not listed in "
                "acav_scenario.yaml's bridge.barrier_participants %r; "
                "if the bridge runs in lockstep it will time out waiting "
                "for an ack that never claims to be from a declared "
                "participant.", options.participant_name,
                spec.bridge.get("barrier_participants", []))

        self._epoch: Optional[StkTime] = None
        self._target_positions: Dict[str, list] = {}
        self._requested_positions: Set[str] = set()
        self._access_requested: Set[str] = set()
        self._init_access_state()
        self._results: Dict[str, TargetResult] = {}
        self._finalized: Set[str] = set()
        self._window_elapsed_wall: Optional[float] = None
        self._shutdown_requested = False
        self._last_sim_time_s: Optional[float] = None
        self._misstamped_twin_status = 0

        # Observed from the bridge rather than assumed from the launch
        # configuration. C1 section 8 commits the paper to reporting
        # nothing the mock produced, and a run that was configured for
        # COM but fell back to the mock must not be indistinguishable
        # from a real one in the record.
        self._backend_type = "unknown"
        self._stk_version = ""
        self._execution_mode = "unknown"
        # The same for E1's knob: which read-back provider the bridge
        # actually used, and whether it was forced. None until a status
        # carries one (an older bridge never does).
        self._readback_provider = ""
        self._readback_forced: Optional[bool] = None
        self._started_utc = _utc_now()

        self._aggregate = RunAggregate(
            momentum_capacity_Nms=spec.actuators.momentum_capacity_Nms)
        self._latency = LatencyAggregate()

        # Wall-clock interval between scenario ticks, which in lockstep
        # IS the closed-loop step time: the barrier does not release
        # until every participant has acked, so tick-to-tick here spans
        # a whole sense/actuate/barrier cycle. Measured from this node's
        # vantage, so it includes its own scheduling jitter -- E6 should
        # report it as an achieved step rate, not as a bridge-internal
        # latency breakdown, which is a different measurement the bridge
        # would have to publish itself.
        self._last_tick_wall_s: Optional[float] = None
        self._tick_intervals_ms: List[float] = []

        retention_s = spec.pointing.settle_hold_s + RETENTION_MARGIN_S
        self._tracker = CaptureTracker(
            capture_error_deg=spec.pointing.capture_error_deg,
            settle_hold_s=spec.pointing.settle_hold_s,
            retention_s=retention_s)

        # Declined targets need no telemetry at all; their result is
        # known the instant the schedule is built.
        for declined in self._plan.declined:
            self._results[declined.target_id] = declined_result(declined)
            self._finalized.add(declined.target_id)

        profiles = qos_module.from_config({})
        prefix = options.prefix

        self._pub_setpoint = node.create_publisher(
            TargetSetpoint, options.setpoint_topic, profiles["command"])
        self._pub_ack = node.create_publisher(
            String, f"{prefix}/lockstep/ack", profiles["command"])

        node.create_subscription(
            ScenarioTime, f"{prefix}/scenario_time", self._on_scenario_time,
            profiles["clock"])
        node.create_subscription(
            TwinStatus, options.twin_status_topic, self._on_twin_status,
            profiles["telemetry"])
        node.create_subscription(
            BridgeStatus, f"{prefix}/status", self._on_bridge_status,
            profiles["status"])

        self._client_query = node.create_client(
            QueryDataProvider, f"{prefix}/query_data_provider")
        self._client_access = node.create_client(
            ComputeAccess, f"{prefix}/compute_access")

        # Steady (wall) clock on purpose: the grace period must keep
        # running when the bridge has stopped stepping.
        from rclpy.clock import Clock, ClockType
        node.create_timer(1.0, self._maybe_finish_run,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))

        LOGGER.info(
            "acav_scheduler ready: policy=%s, %d attempted, %d declined",
            self._plan.scheduler_type.value, len(self._plan.attempted),
            len(self._plan.declined))

    # -- lockstep step ------------------------------------------------------

    def _observe_attitude_provider(self, message) -> None:
        provider = (getattr(message, "attitude_provider", "") or "").strip()
        if not provider:
            return              # sensing not configured yet, or an old bridge
        forced = bool(getattr(message, "attitude_provider_forced", False))
        if (provider, forced) != (self._readback_provider,
                                  self._readback_forced):
            LOGGER.info("bridge attitude read-back: %s (%s)", provider,
                        "FORCED" if forced else "validated")
            self._readback_provider, self._readback_forced = provider, forced
            observed = provider if forced else ""
            if observed != self._options.attitude_provider:
                LOGGER.error(
                    "the bridge's attitude read-back is %r (%s), but this "
                    "run was launched for forced provider %r. The record "
                    "will carry what the bridge did, and the campaign "
                    "runner will reject it.", provider,
                    "forced" if forced else "validated",
                    self._options.attitude_provider)

    def _recorded_attitude_provider(self) -> str:
        """Provenance's forced provider: observed when the bridge says."""
        if self._readback_forced is None:
            if self._options.attitude_provider:
                LOGGER.warning(
                    "no bridge status reported an attitude read-back "
                    "provider; recording the requested %r unobserved",
                    self._options.attitude_provider)
            return self._options.attitude_provider
        return self._readback_provider if self._readback_forced else ""

    def _on_bridge_status(self, message) -> None:
        """Record which backend is actually running, not which was asked for."""
        backend = (message.backend_type or "").strip().lower()
        if backend and backend != self._backend_type:
            if self._backend_type != "unknown":
                LOGGER.warning(
                    "the bridge's backend changed mid-run: %s -> %s. The "
                    "record will carry the last one seen.",
                    self._backend_type, backend)
            LOGGER.info("bridge backend: %s (STK %s)", backend,
                        message.stk_version_string or "unknown")
        self._backend_type = backend or "unknown"
        self._stk_version = message.stk_version_string or ""
        self._observe_attitude_provider(message)

        # E6's breakdown. Observed unconditionally, on every status
        # message, not just during a designated "E6 run" -- it costs
        # nothing to accumulate and means a campaign never has to be
        # re-run just because someone later wants the latency figure for
        # a run that was launched for a different experiment.
        self._latency.observe(
            message.mean_sense_ms, message.mean_actuate_ms,
            message.mean_barrier_wait_ms, message.step_samples)

    def _on_scenario_time(self, message) -> None:
        import time

        self._execution_mode = _execution_mode_name(message.execution_mode)
        now_wall = time.perf_counter()
        if self._last_tick_wall_s is not None:
            self._tick_intervals_ms.append(
                (now_wall - self._last_tick_wall_s) * 1000.0)
        self._last_tick_wall_s = now_wall
        stamp = conv.ros_time_to_stk(message.sim_time)
        epoch = conv.ros_time_to_stk(message.scenario_epoch)

        if self._epoch is None:
            self._epoch = epoch
            LOGGER.info("scenario epoch resolved: %s", epoch)

        sim_time_s = stamp - self._epoch
        self._last_sim_time_s = sim_time_s

        # Retried every tick, not just once at start: a service that is
        # not yet advertised at t=0 (the bridge node and this one can
        # start in either order) would otherwise leave a target's
        # position permanently unrequested. Each call is a no-op for
        # any target already resolved or already in flight.
        if not self._prepare_target_positions():
            return  # Keep the initial barrier closed until geometry is ready.
        self._publish_setpoint(sim_time_s)
        self._check_access_due(sim_time_s)
        self._check_run_complete(sim_time_s)

        # Ack unconditionally, last: everything above is either O(1) or
        # a fire-and-forget async call. Nothing here can block on STK.
        self._acknowledge_step(int(message.step_index))

    def _acknowledge_step(self, step_index: int) -> None:
        from std_msgs.msg import String

        message = String()
        message.data = f"{self._options.participant_name}:{step_index}"
        self._pub_ack.publish(message)

    def _publish_setpoint(self, sim_time_s: float) -> None:
        from stk_msgs.msg import TargetSetpoint

        active = self._plan.active_entry_at(sim_time_s)
        message = TargetSetpoint()
        message.header.stamp = conv.stk_time_to_ros(self._epoch + sim_time_s)
        message.header.frame_id = "icrf"

        if active is not None and active.target_id in self._target_positions:
            x, y, z = interpolate_target_position(
                self._target_positions[active.target_id], sim_time_s)
            message.target_id = active.target_id
            message.target_position_m.x = x
            message.target_position_m.y = y
            message.target_position_m.z = z
            message.has_target = True
        else:
            # Either genuinely no active target (nadir hold), or one IS
            # active but its position query has not resolved yet -- the
            # latter is possible only in the first fraction of a second
            # of the run, while the start-of-run queries are still in
            # flight. Holding at nadir is the honest answer in both
            # cases: pointing at (0, 0, 0) would be worse than pointing
            # at nothing.
            message.target_id = ""
            message.has_target = False

        self._pub_setpoint.publish(message)

    # -- twin telemetry -------------------------------------------------

    def _on_twin_status(self, message) -> None:
        if self._epoch is None:
            LOGGER.debug("twin_status received before the scenario epoch "
                        "is known; dropping one sample")
            return
        time_s = conv.ros_time_to_stk(message.header.stamp) - self._epoch
        self._check_twin_stamp(time_s)
        try:
            sample = TelemetrySample(
                time_s=time_s,
                error_deg=math.degrees(message.boresight_error_rad),
                mode=str(message.mode),
                wheel_momentum_Nms=(message.wheel_momentum_nms.x,
                                    message.wheel_momentum_nms.y,
                                    message.wheel_momentum_nms.z),
                wheels_saturated=bool(message.wheels_saturated),
                wheel_energy_J=float(message.wheel_energy_j),
                **_between_publishes(message))
            self._tracker.observe(sample)
            # Accumulated even for samples the tracker would discard as
            # outside every hold window: peak momentum and saturation are
            # properties of the run, and a wheel that saturated between
            # two targets saturated.
            self._aggregate.observe(sample)
        except ValueError as exc:
            # Out-of-order telemetry is a harness/transport problem, not
            # something to crash the node over mid-run; log loudly and
            # keep going; that target's window may end up short a
            # sample, which evaluate_capture will notice on its own.
            LOGGER.error("twin_status out of order: %s", exc)

    def _check_twin_stamp(self, time_s: float) -> None:
        """Say so, loudly, when the twin is stamping on the wrong clock.

        Every hold-window lookup uses the TwinStatus stamp as the
        sample's time. A twin stamping with wall-clock time (the first
        live run did) produces samples ~73 million seconds after the
        epoch: none can ever fall in a hold window, every target fails
        to finalise, and each failure message on its own reads like a
        sampling-rate problem rather than a clock problem.
        """
        if self._last_sim_time_s is None:
            return
        offset_s = time_s - self._last_sim_time_s
        if abs(offset_s) <= TWIN_STAMP_TOLERANCE_S:
            return
        self._misstamped_twin_status += 1
        if self._misstamped_twin_status == 1:
            LOGGER.error(
                "twin_status stamped at t=%.1f s while the scenario is at "
                "t=%.1f s (%+.3g s apart): the twin is stamping with a "
                "clock other than the scenario's -- wall-clock time is "
                "the usual cause. No such sample can fall in a hold "
                "window, so no target can be scored.",
                time_s, self._last_sim_time_s, offset_s)

    # -- target geometry --------------------------------------------------

    def _prepare_target_positions(self):
        import time
        pending = [e for e in self._plan.attempted
                   if e.target_id not in self._target_positions]
        if not pending:
            if not getattr(self, '_geometry_ready', False):
                LOGGER.info("target geometry ready; releasing initial lockstep barrier")
                self._geometry_ready = True
                self._last_tick_wall_s = None
            return True
        if not hasattr(self, '_geometry_started'):
            self._geometry_started = time.monotonic()
        if time.monotonic() - self._geometry_started > 180.0:
            raise RuntimeError("Target geometry preparation timed out before control started: "
                               + ', '.join(e.target_id for e in pending))
        self._request_all_target_positions()
        return False

    def _request_all_target_positions(self) -> None:
        # A single COM worker also applies attitude commands. Do not queue
        # every target behind it, and do not release the startup barrier yet.
        for entry in self._plan.attempted:
            if entry.target_id in self._target_positions:
                continue
            self._request_target_position(entry)
            return

    def _request_target_position(self, entry: ScheduleEntry) -> None:
        from stk_msgs.msg import DataProviderRequest
        from stk_msgs.srv import QueryDataProvider

        if (entry.target_id in self._requested_positions
                or entry.target_id in self._target_positions):
            return
        self._requested_positions.add(entry.target_id)

        target_path = target_object_path(entry.target_id)
        # Ground targets move in ICRF: cache their trajectory, not a
        # single position at the future imaging epoch.
        when = self._epoch
        request = QueryDataProvider.Request()
        request.request = DataProviderRequest(
            object_path=target_path, provider_name="Cartesian Position",
            group_name="ICRF", element_names=[],
            start_time=conv.stk_time_to_ros(when),
            stop_time=conv.stk_time_to_ros(when + math.ceil(
                self._spec.window_duration_s / TARGET_POSITION_STEP_S) * TARGET_POSITION_STEP_S),
            step_s=TARGET_POSITION_STEP_S)

        if not self._client_query.wait_for_service(timeout_sec=0.0):
            LOGGER.warning(
                "%s/query_data_provider is not available yet; will retry "
                "on the next scenario_time tick", self._options.prefix)
            self._requested_positions.discard(entry.target_id)
            return

        future = self._client_query.call_async(request)
        future.add_done_callback(
            lambda f, target_id=entry.target_id, path=target_path:
                self._on_target_position_response(target_id, path, f))

    def _on_target_position_response(self, target_id: str, target_path: str,
                                     future) -> None:
        try:
            response = future.result()
        except Exception as exc:                          # noqa: BLE001
            raise RuntimeError(f"Target geometry preparation failed for {target_id}: {exc}") from exc

        result = response.result
        if not result.success or result.row_count == 0:
            raise RuntimeError(f"Target geometry preparation failed for {target_id}: "
                               + (result.error_message or "no rows"))

        try:
            columns = _unflatten(result.column_names, result.flattened_data,
                                 result.row_count)
            data = dp.DataProviderData(
                object_path=target_path, provider_name="Cartesian Position",
                columns=columns, row_count=result.row_count)
            mapping = dp.resolve_columns(dp.CARTESIAN_POSITION, data)
            samples = list(zip(*(tuple(float(v) for v in columns[mapping[c]])
                                 for c in ("x", "y", "z"))))
            expected = math.ceil(self._spec.window_duration_s / TARGET_POSITION_STEP_S) + 1
            if len(samples) != expected or not all(
                    math.isfinite(v) for row in samples for v in row):
                raise ValueError(f"expected {expected} valid time-varying positions, got {len(samples)}")
        except (StkBridgeError, ValueError, KeyError) as exc:
            raise RuntimeError(f"Invalid target geometry for {target_id}: {exc}") from exc

        self._target_positions[target_id] = samples
        LOGGER.info("%s ICRF trajectory resolved: %d positions, %.1f s spacing",
                    target_id, len(samples), TARGET_POSITION_STEP_S)

    # -- access checks ------------------------------------------------------

    def _init_access_state(self) -> None:
        """Bookkeeping for access requests the bridge defers.

        The bridge answers a request whose stop time the satellite's
        written attitude does not reach yet with ATTITUDE_NOT_COVERED
        (stk_bridge.core.actuation.attitude_coverage_gap); such a target
        is asked again on the next tick. (wall, scenario) seconds of each
        target's first and latest request give its recorded wait.
        """
        self._access_first_request: Dict[str, Tuple[float, float]] = {}
        self._access_last_request: Dict[str, Tuple[float, float]] = {}
        self._access_not_covered: Dict[str, str] = {}

    def _check_access_due(self, sim_time_s: float) -> None:
        for entry in self._plan.attempted:
            if entry.target_id in self._access_requested:
                continue
            due_at = entry.t_image_s + self._options.access_query_delay_s
            if sim_time_s >= due_at:
                self._request_access(entry, sim_time_s)

    def _access_waits(self, target_id: str) -> Tuple[float, float]:
        """(scenario s, wall s) from the first request to the latest."""
        first = self._access_first_request.get(target_id)
        last = self._access_last_request.get(target_id)
        if first is None or last is None:
            return float("nan"), float("nan")
        return last[1] - first[1], last[0] - first[0]

    def _request_access(self, entry: ScheduleEntry,
                        sim_time_s: float) -> None:
        from stk_msgs.srv import ComputeAccess

        self._access_requested.add(entry.target_id)
        target_path = target_object_path(entry.target_id)
        sensor_path = self._spec.sensor.object_path

        window_start = self._epoch + max(
            0.0, entry.t_image_s - self._spec.pointing.settle_hold_s)
        window_stop = self._epoch + entry.t_image_s

        if not self._client_access.wait_for_service(timeout_sec=0.0):
            LOGGER.warning(
                "%s/compute_access not available yet for %s; retrying "
                "on the next tick", self._options.prefix, entry.target_id)
            self._access_requested.discard(entry.target_id)
            return

        sent = (time.monotonic(), sim_time_s)
        self._access_first_request.setdefault(entry.target_id, sent)
        self._access_last_request[entry.target_id] = sent

        request = ComputeAccess.Request()
        request.from_object_path = sensor_path
        request.to_object_path = target_path
        request.start_time = conv.stk_time_to_ros(window_start)
        request.stop_time = conv.stk_time_to_ros(window_stop)

        future = self._client_access.call_async(request)
        future.add_done_callback(
            lambda f, e=entry: self._on_access_response(e, f))

    def _on_access_response(self, entry: ScheduleEntry, future) -> None:
        try:
            response = future.result()
        except Exception as exc:                          # noqa: BLE001
            LOGGER.error("ComputeAccess for %s failed: %s", entry.target_id,
                        exc)
            self._access_requested.discard(entry.target_id)
            return

        if not response.success:
            self._access_requested.discard(entry.target_id)
            if response.message.startswith(ATTITUDE_NOT_COVERED):
                if entry.target_id not in self._access_not_covered:
                    LOGGER.info(
                        "access for %s deferred until an attitude sample at "
                        "or after t = %.3f s is written: %s", entry.target_id,
                        entry.t_image_s, response.message)
                self._access_not_covered[entry.target_id] = response.message
                return
            self._access_not_covered.pop(entry.target_id, None)
            LOGGER.error("ComputeAccess for %s reported failure: %s",
                        entry.target_id, response.message)
            return
        self._access_not_covered.pop(entry.target_id, None)

        image_epoch = self._epoch + entry.t_image_s
        has_access = any(
            conv.ros_time_to_stk(interval.start_time) <= image_epoch
            <= conv.ros_time_to_stk(interval.stop_time)
            for interval in response.accesses.intervals)

        try:
            self._results[entry.target_id] = self._tracker.finalize(
                entry, has_access)
        except ValueError as exc:
            # Recorded, never dropped: a target missing from the result
            # would leave the counts and the signature short of it.
            LOGGER.error("could not score %s, recorded as %s: %s",
                         entry.target_id,
                         CaptureOutcome.NO_TELEMETRY.value, exc)
            self._results[entry.target_id] = no_telemetry_result(
                entry, str(exc))
        result = self._results[entry.target_id]
        result.access_wait_s, result.access_wait_wall_s = (
            self._access_waits(entry.target_id))
        self._finalized.add(entry.target_id)
        LOGGER.info("%s finalised: %s (access=%s, waited %.3f s scenario, "
                    "%.3f s wall)", entry.target_id, result.outcome.value,
                    has_access, result.access_wait_s,
                    result.access_wait_wall_s)

    # -- run completion -------------------------------------------------

    def _check_run_complete(self, sim_time_s: float) -> None:
        if self._shutdown_requested:
            return
        if sim_time_s < self._spec.window_duration_s:
            return
        if self._window_elapsed_wall is None:
            self._window_elapsed_wall = time.monotonic()
        self._maybe_finish_run()

    def _maybe_finish_run(self) -> None:
        """Finish once every target is accounted for, or the grace expires.

        Called on every tick after the window's end and from a 1 Hz
        steady-clock timer, since past the scenario's end there may be no
        more ticks at all.
        """
        if self._shutdown_requested or self._window_elapsed_wall is None:
            return
        all_accounted = len(self._finalized) == len(self._spec.targets)
        grace_expired = (time.monotonic() - self._window_elapsed_wall
                         >= FINALIZE_GRACE_WALL_S)
        if all_accounted or grace_expired:
            self._finish_run(forced=len(self._finalized)
                             != len(self._spec.targets))

    def _finish_run(self, forced: bool) -> None:
        self._shutdown_requested = True
        if forced:
            # Only attempted targets can be outstanding: declined ones
            # are finalised when the schedule is built.
            reason = (f"no access answer arrived within "
                      f"{FINALIZE_GRACE_WALL_S:.0f} s of the window's end")
            missing = [entry for entry in self._plan.attempted
                       if entry.target_id not in self._finalized]
            LOGGER.error(
                "finalising the run with %d target(s) never scored, "
                "recorded as %s or %s: %s. The record is not reportable.",
                len(missing), CaptureOutcome.ACCESS_UNVERIFIED.value,
                CaptureOutcome.NO_ACCESS_ANSWER.value,
                ", ".join(entry.target_id for entry in missing))
            for entry in missing:
                deferred = self._access_not_covered.get(entry.target_id)
                if deferred is not None:
                    result = access_unverified_result(
                        entry, f"no attitude sample at or after the image "
                               f"epoch was written by the end of the run "
                               f"({deferred})")
                else:
                    result = no_access_answer_result(entry, reason)
                result.access_wait_s, result.access_wait_wall_s = (
                    self._access_waits(entry.target_id))
                self._results[entry.target_id] = result
                self._finalized.add(entry.target_id)

        result = self._build_run_result()
        self._write_output(result)
        LOGGER.info("run complete: %d/%d captured (scheduler=%s)\n%s",
                   result.captured_count, result.attempted_count,
                   result.scheduler,
                   "\n".join(f"  {t.target_id}: {t.outcome.value}"
                            for t in result.targets))
        if self._options.auto_shutdown:
            self._request_shutdown()

    def _build_run_result(self) -> RunResult:
        ordered_ids = [t.id for t in
                       sorted(self._spec.targets, key=lambda t: t.t_image_s)]
        targets = [self._results[target_id] for target_id in ordered_ids
                  if target_id in self._results]
        result = RunResult(
            run_id=self._options.run_id,
            scheduler=self._plan.scheduler_type.value,
            targets=targets,
            seed=self._options.seed,
        )
        self._aggregate.apply_to(result)
        self._latency.apply_to(result)
        if self._tick_intervals_ms:
            result.mean_step_latency_ms = (
                sum(self._tick_intervals_ms) / len(self._tick_intervals_ms))
        return result

    def _write_output(self, result: RunResult) -> None:
        """Write the run as a C5 record: the result plus its provenance.

        The provenance is what decides whether the numbers may be
        published at all, so it is assembled here, in the process that
        watched the run happen, rather than reconstructed afterwards
        from launch arguments and memory.
        """
        if not self._options.output_json:
            return

        from acav.run_record import (
            Provenance,
            RunRecord,
            write_record,
        )

        scenario = {"scenario_sha256": ""}
        # The file the spec was actually loaded from: the default one when
        # no path was given -- which is how every campaign so far ran, and
        # why their records carry no scenario fingerprint at all.
        scenario_file = self._options.scenario_path or "(default)"
        try:
            from acav.scenarios import scenario_provenance

            scenario_file = AcavSpec.locate(self._options.scenario_path or None)
            scenario = scenario_provenance(scenario_file)
        except (OSError, FileNotFoundError, ValueError) as error:
            LOGGER.warning("could not hash the scenario file %s: %s",
                           scenario_file, error)

        record = RunRecord(
            result=result,
            experiment=self._options.experiment,
            provenance=Provenance(
                backend_type=self._backend_type,
                stk_version=self._stk_version,
                execution_mode=self._execution_mode,
                **scenario,
                gains=_parse_json_object(self._options.gains_json, "gains"),
                attitude_provider=self._recorded_attitude_provider(),
                attitude_readback_provider=self._readback_provider,
                code_version=_code_version(),
                twin_identity=getattr(self._options, "twin_identity", ""),
                fault=_parse_json_value(self._options.fault_json, "fault"),
                started_utc=self._started_utc,
                finished_utc=_utc_now()))

        write_record(self._options.output_json, record)
        LOGGER.info("wrote %s (backend=%s, mode=%s, reportable=%s)",
                    self._options.output_json, self._backend_type,
                    self._execution_mode, record.reportable)
        if not record.reportable:
            LOGGER.warning(
                "this run is NOT reportable: %s. No number it produced may "
                "appear in the paper; it is fine for development.",
                "; ".join(record.unreportable_reasons))

    def _request_shutdown(self) -> None:
        import rclpy

        if rclpy.ok():
            self._node.create_timer(0.5, self._shutdown_timer_callback)

    def _shutdown_timer_callback(self) -> None:
        import rclpy

        if rclpy.ok():
            rclpy.shutdown()


def target_object_path(target_id: str) -> str:
    """The STK object path a scenario target id resolves to.

    Matches the convention ``scenario_builder.py`` creates targets
    under (``Target`` objects named by their scenario id) -- this is
    the ONE place that convention is assumed on the scheduler side, so
    a future change to it needs changing in one place, not wherever a
    target path gets built.
    """
    return f"*/Target/{target_id}"


def _unflatten(column_names: List[str], flattened: List[float],
              row_count: int) -> Dict[str, List[float]]:
    """Row-major ``flattened_data`` back into per-column lists.

    Mirrors ``DataProviderResult.msg``'s own documented layout
    (``row_count x len(column_names)``, row-major) exactly, so this must
    not drift from that comment if the wire format ever changes.
    """
    columns: Dict[str, List[float]] = {name: [] for name in column_names}
    width = len(column_names)
    for row in range(row_count):
        offset = row * width
        for index, name in enumerate(column_names):
            columns[name].append(flattened[offset + index])
    return columns


def load_spec(spec_path: str) -> AcavSpec:
    return AcavSpec.load(spec_path or None)


def main(args=None) -> None:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node

    rclpy.init(args=args)
    node = Node("acav_scheduler")

    node.declare_parameter("scenario_config", "")
    node.declare_parameter("scheduler_type", "dynamics_aware")
    node.declare_parameter("prefix", "stk")
    node.declare_parameter("participant_name", "acav_scheduler")
    node.declare_parameter("setpoint_topic", "/acav/setpoint")
    node.declare_parameter("twin_status_topic", "/acav/twin_status")
    node.declare_parameter("output_json", "")
    node.declare_parameter("access_query_delay_s",
                           DEFAULT_ACCESS_QUERY_DELAY_S)
    node.declare_parameter("auto_shutdown", True)
    node.declare_parameter("run_id", "run")
    node.declare_parameter("seed", -1)
    node.declare_parameter("experiment", "")
    node.declare_parameter("gains_json", "")
    node.declare_parameter("fault_json", "")
    node.declare_parameter("attitude_provider", "")
    node.declare_parameter("twin_identity", "")

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s")

    spec = load_spec(node.get_parameter("scenario_config").value)
    seed_value = int(node.get_parameter("seed").value)
    options = SchedulerOptions(
        scheduler_type=str(node.get_parameter("scheduler_type").value),
        prefix=str(node.get_parameter("prefix").value),
        participant_name=str(node.get_parameter("participant_name").value),
        setpoint_topic=str(node.get_parameter("setpoint_topic").value),
        twin_status_topic=str(node.get_parameter("twin_status_topic").value),
        output_json=str(node.get_parameter("output_json").value),
        access_query_delay_s=float(
            node.get_parameter("access_query_delay_s").value),
        auto_shutdown=bool(node.get_parameter("auto_shutdown").value),
        run_id=str(node.get_parameter("run_id").value),
        seed=None if seed_value < 0 else seed_value,
        experiment=str(node.get_parameter("experiment").value),
        scenario_path=str(node.get_parameter("scenario_config").value),
        gains_json=str(node.get_parameter("gains_json").value),
        fault_json=str(node.get_parameter("fault_json").value),
        attitude_provider=str(
            node.get_parameter("attitude_provider").value),
        twin_identity=str(node.get_parameter("twin_identity").value),
    )

    scheduler = SchedulerNode(node, spec, options)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        LOGGER.info("interrupted")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    del scheduler


if __name__ == "__main__":
    main()
