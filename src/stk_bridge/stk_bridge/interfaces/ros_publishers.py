"""Telemetry publishers.

Converts the typed samples produced by ``core/sensing.py`` into
``stk_msgs`` and publishes them. One publisher set per satellite, under
a per-object namespace derived from the STK object name, so a
multi-satellite scenario needs no configuration beyond listing the
objects.

Absence is information
----------------------
Nothing is published for a channel that returned no sample. The
alternative, republishing the last value with a fresh timestamp, would
make a frozen state indistinguishable from a held attitude. A consumer
that sees no message knows it has no state.

The ``is_extrapolated`` flag carries the weaker version of the same
warning: the sample is real but came from STK's real-time propagator
look-ahead rather than an injected measurement.
"""

from __future__ import annotations

import logging
from typing import Dict, List, Optional

from stk_bridge.backends.base import (
    AngularRateSample,
    AttitudeSample,
    EclipseSample,
    LLASample,
    StateSample,
    StkTime,
)
from stk_bridge.core import conversions as conv
from stk_bridge.core.frames import FrameRegistry

LOGGER = logging.getLogger(__name__)


def topic_namespace(object_path: str) -> str:
    """``*/Satellite/SSS-1`` -> ``sss_1``.

    Lower-cased with non-alphanumerics collapsed to underscores, because
    ROS topic names must match ``[a-zA-Z0-9_]`` and STK object names
    routinely contain hyphens and spaces.
    """
    name = conv.object_name(object_path)
    cleaned = "".join(c if c.isalnum() else "_" for c in name).strip("_")
    while "__" in cleaned:
        cleaned = cleaned.replace("__", "_")
    return cleaned.lower() or "unnamed"


class SatellitePublishers:
    """Per-satellite telemetry publishers."""

    def __init__(self, node, object_path: str, frames: FrameRegistry,
                 qos_profiles: Dict[str, object], prefix: str = "stk",
                 mismatch: str = ""):
        from stk_msgs.msg import (
            AngularRate,
            AttitudeState,
            EclipseState,
            LLAState,
            OrbitState,
            SatelliteState,
        )

        self._node = node
        self._path = conv.canonical_object_path(object_path)
        self._frames = frames
        self._namespace = topic_namespace(self._path)
        base = f"{prefix}/{self._namespace}"
        telemetry = qos_profiles["telemetry"]

        self._pub_orbit = node.create_publisher(
            OrbitState, f"{base}/orbit", telemetry)
        self._pub_attitude = node.create_publisher(
            AttitudeState, f"{base}/attitude", telemetry)
        self._pub_rate = node.create_publisher(
            AngularRate, f"{base}/angular_rate", telemetry)
        self._pub_lla = node.create_publisher(
            LLAState, f"{base}/lla", telemetry)
        self._pub_eclipse = node.create_publisher(
            EclipseState, f"{base}/eclipse", telemetry)
        self._pub_state = node.create_publisher(
            SatelliteState, f"{base}/state", telemetry)

        self._published = 0
        #: Optional interface mismatch applied to everything publish_samples
        #: sends (core/mismatch_injection.py); empty by default.
        from stk_bridge.core.mismatch_injection import check
        self._mismatch = check(mismatch)
        if self._mismatch:
            LOGGER.warning(
                "INTERFACE MISMATCH INJECTED into %s telemetry: %r. "
                "Consumers will read this interface with the wrong "
                "convention. This is a fault-injection setting; if you did "
                "not set it deliberately, clear sensing.inject_mismatch.",
                self._path, self._mismatch)
        LOGGER.info("publishers created for %s under %s/", self._path, base)

    @property
    def object_path(self) -> str:
        return self._path

    @property
    def namespace(self) -> str:
        return self._namespace

    @property
    def published_count(self) -> int:
        return self._published

    # -- helpers -----------------------------------------------------------

    def _header(self, when: StkTime, frame_id: str):
        from std_msgs.msg import Header

        header = Header()
        header.stamp = conv.stk_time_to_ros(when)
        header.frame_id = frame_id
        return header

    # -- individual channels ----------------------------------------------

    def publish_orbit(self, sample: StateSample) -> None:
        from stk_msgs.msg import OrbitState

        message = OrbitState()
        message.header = self._header(sample.time, sample.frame)
        message.object_path = self._path
        message.position.x, message.position.y, message.position.z = \
            sample.position_m
        message.velocity.x, message.velocity.y, message.velocity.z = \
            sample.velocity_mps
        message.is_extrapolated = sample.is_extrapolated
        self._pub_orbit.publish(message)
        self._published += 1

    def publish_attitude(self, sample: AttitudeSample) -> None:
        from stk_msgs.msg import AttitudeState

        message = AttitudeState()
        message.header = self._header(sample.time, sample.frame)
        message.object_path = self._path
        qx, qy, qz, qw = sample.quaternion_xyzw
        message.orientation.x = qx
        message.orientation.y = qy
        message.orientation.z = qz
        message.orientation.w = qw
        message.is_extrapolated = sample.is_extrapolated
        self._pub_attitude.publish(message)
        self._published += 1

    def publish_angular_rate(self, sample: AngularRateSample) -> None:
        from stk_msgs.msg import AngularRate

        message = AngularRate()
        message.header = self._header(sample.time,
                                      self._frames.vehicle_frame_id(
                                          self._path, "body"))
        message.object_path = self._path
        wx, wy, wz = sample.angular_velocity_radps
        message.angular_velocity.x = wx
        message.angular_velocity.y = wy
        message.angular_velocity.z = wz
        self._pub_rate.publish(message)
        self._published += 1

    def publish_lla(self, sample: LLASample) -> None:
        from stk_msgs.msg import LLAState

        message = LLAState()
        message.header = self._header(sample.time,
                                      self._frames.ros_id("fixed"))
        message.object_path = self._path
        message.latitude = sample.latitude_rad
        message.longitude = sample.longitude_rad
        message.altitude_m = sample.altitude_m
        self._pub_lla.publish(message)
        self._published += 1

    def publish_eclipse(self, sample: EclipseSample) -> None:
        from stk_msgs.msg import EclipseState

        message = EclipseState()
        message.header = self._header(sample.time,
                                      self._frames.ros_id("icrf"))
        message.object_path = self._path
        message.state = {
            "Umbra": EclipseState.UMBRA,
            "Penumbra": EclipseState.PENUMBRA,
            "Direct Sun": EclipseState.SUNLIT,
        }.get(sample.condition.value, EclipseState.SUNLIT)
        self._pub_eclipse.publish(message)
        self._published += 1

    # -- aggregate ---------------------------------------------------------

    def publish_state(self, samples: Dict[str, object]) -> bool:
        """Publish the composite ``SatelliteState``.

        Requires orbit and attitude to both be present. A composite
        message with a default-constructed attitude would claim identity
        orientation, which is a specific and wrong claim rather than an
        absent one -- so if either is missing, nothing is published and
        consumers fall back to the per-channel topics.
        """
        from stk_msgs.msg import SatelliteState

        orbit = samples.get("orbit")
        attitude = samples.get("attitude")
        if orbit is None or attitude is None:
            return False

        message = SatelliteState()
        message.header = self._header(orbit.time, orbit.frame)
        message.object_path = self._path

        message.orbit.header = self._header(orbit.time, orbit.frame)
        message.orbit.object_path = self._path
        (message.orbit.position.x, message.orbit.position.y,
         message.orbit.position.z) = orbit.position_m
        (message.orbit.velocity.x, message.orbit.velocity.y,
         message.orbit.velocity.z) = orbit.velocity_mps
        message.orbit.is_extrapolated = orbit.is_extrapolated

        message.attitude.header = self._header(attitude.time, attitude.frame)
        message.attitude.object_path = self._path
        qx, qy, qz, qw = attitude.quaternion_xyzw
        message.attitude.orientation.x = qx
        message.attitude.orientation.y = qy
        message.attitude.orientation.z = qz
        message.attitude.orientation.w = qw
        message.attitude.is_extrapolated = attitude.is_extrapolated

        rate = samples.get("angular_rate")
        if rate is not None:
            message.angular_rate.header = self._header(
                rate.time, self._frames.vehicle_frame_id(self._path, "body"))
            message.angular_rate.object_path = self._path
            wx, wy, wz = rate.angular_velocity_radps
            message.angular_rate.angular_velocity.x = wx
            message.angular_rate.angular_velocity.y = wy
            message.angular_rate.angular_velocity.z = wz

        eclipse = samples.get("eclipse")
        if eclipse is not None:
            self._fill_eclipse(message.eclipse, eclipse)

        self._pub_state.publish(message)
        self._published += 1
        return True

    def _fill_eclipse(self, target, sample: EclipseSample) -> None:
        from stk_msgs.msg import EclipseState

        target.header = self._header(sample.time, self._frames.ros_id("icrf"))
        target.object_path = self._path
        target.state = {
            "Umbra": EclipseState.UMBRA,
            "Penumbra": EclipseState.PENUMBRA,
            "Direct Sun": EclipseState.SUNLIT,
        }.get(sample.condition.value, EclipseState.SUNLIT)

    def publish_samples(self, samples: Dict[str, object]) -> int:
        """Publish every channel present in ``samples``. Returns count."""
        if self._mismatch:
            from stk_bridge.core.mismatch_injection import inject
            samples = inject(self._mismatch, samples)
        count = 0
        if "orbit" in samples:
            self.publish_orbit(samples["orbit"])
            count += 1
        if "attitude" in samples:
            self.publish_attitude(samples["attitude"])
            count += 1
        if "angular_rate" in samples:
            self.publish_angular_rate(samples["angular_rate"])
            count += 1
        if "lla" in samples:
            self.publish_lla(samples["lla"])
            count += 1
        if "eclipse" in samples:
            self.publish_eclipse(samples["eclipse"])
            count += 1
        if self.publish_state(samples):
            count += 1
        return count


class BridgePublishers:
    """Bridge-wide publishers: status, diagnostics, clock, scenario time."""

    def __init__(self, node, qos_profiles: Dict[str, object],
                 prefix: str = "stk", publish_clock: bool = True):
        from rosgraph_msgs.msg import Clock
        from stk_msgs.msg import (
            BackendDiagnostics,
            BridgeStatus,
            ScenarioTime,
        )

        self._node = node
        self._publish_clock = publish_clock

        self._pub_status = node.create_publisher(
            BridgeStatus, f"{prefix}/status", qos_profiles["status"])
        self._pub_diagnostics = node.create_publisher(
            BackendDiagnostics, f"{prefix}/diagnostics",
            qos_profiles["status"])
        self._pub_scenario_time = node.create_publisher(
            ScenarioTime, f"{prefix}/scenario_time", qos_profiles["clock"])
        self._pub_clock = (
            node.create_publisher(Clock, "/clock", qos_profiles["clock"])
            if publish_clock else None
        )

    def publish_clock(self, when: StkTime) -> None:
        if self._pub_clock is None:
            return
        from rosgraph_msgs.msg import Clock

        message = Clock()
        message.clock = conv.stk_time_to_ros(when)
        self._pub_clock.publish(message)

    def publish_scenario_time(self, sim_time: StkTime, epoch: StkTime,
                              mode: int, real_time_factor: float,
                              step_index: int, paused: bool) -> None:
        from stk_msgs.msg import ScenarioTime

        message = ScenarioTime()
        message.sim_time = conv.stk_time_to_ros(sim_time)
        message.scenario_epoch = conv.stk_time_to_ros(epoch)
        message.execution_mode = int(mode)
        message.real_time_factor = float(real_time_factor)
        message.step_index = int(step_index)
        message.paused = bool(paused)
        self._pub_scenario_time.publish(message)

    def publish_status(self, state: int, backend_type: str,
                       stk_version: str, dropped: Dict[str, int],
                       queue_depth: int, queue_capacity: int,
                       latency_ms: float, last_error: str,
                       timings: Optional[Dict[str, float]] = None,
                       when: Optional[StkTime] = None,
                       attitude_provider: str = "",
                       attitude_provider_forced: bool = False) -> None:
        from std_msgs.msg import Header
        from stk_msgs.msg import BridgeStatus

        message = BridgeStatus()
        header = Header()
        if when is not None:
            header.stamp = conv.stk_time_to_ros(when)
        header.frame_id = "stk_bridge"
        message.header = header

        message.state = int(state)
        message.backend_type = backend_type
        message.stk_version_string = stk_version
        message.dropped_attitude_samples = int(dropped.get("attitude", 0))
        message.dropped_position_samples = int(dropped.get("position", 0))
        message.command_queue_depth = int(queue_depth)
        message.command_queue_capacity = int(queue_capacity)
        message.last_backend_round_trip_ms = float(latency_ms)
        message.last_error = last_error[:1000]
        timings = timings or {}
        message.mean_sense_ms = float(timings.get("mean_sense_ms", 0.0))
        message.mean_actuate_ms = float(timings.get("mean_actuate_ms", 0.0))
        message.mean_barrier_wait_ms = float(
            timings.get("mean_barrier_wait_ms", 0.0))
        message.step_samples = int(timings.get("step_samples", 0))
        message.attitude_provider = attitude_provider
        message.attitude_provider_forced = bool(attitude_provider_forced)
        self._pub_status.publish(message)

    def publish_diagnostics(self, backend, report, worker,
                            when: Optional[StkTime] = None) -> None:
        from std_msgs.msg import Header
        from stk_msgs.msg import BackendDiagnostics

        message = BackendDiagnostics()
        header = Header()
        if when is not None:
            header.stamp = conv.stk_time_to_ros(when)
        header.frame_id = "stk_bridge"
        message.header = header

        info = backend.info
        capabilities = backend.capabilities
        message.backend_type = info.backend_type.value
        message.licensed_integration = capabilities.integration
        message.licensed_astrogator = capabilities.astrogator
        message.licensed_realtime_propagator = capabilities.realtime_propagator
        message.licensed_engine_on_linux = False
        message.python_version = info.python_version
        message.agi_stk_wheel_version = info.agi_stk_wheel_version
        message.mean_command_latency_ms = worker.last_latency_ms
        # Masked to 32 bits: the field is uint32 and diagnostic only,
        # and threading.get_ident() can exceed 32 bits on 64-bit Linux.
        message.com_apartment_thread_id = worker.thread_id & 0xFFFFFFFF
        self._pub_diagnostics.publish(message)
