"""Reads STK state into typed samples.

Consumes the :class:`~stk_bridge.core.validation.ValidationReport` rather
than hardcoding provider names, so the pipeline uses whatever the live
install actually supports. On the STK 12.6 install this was developed
against, validation resolves:

* attitude   -> ``ECF Attitude Quaternions`` (pairs with ``AddAttitude
  CBFQuat``; the inertial pair ``Attitude Quaternions`` + ``AddAttitude
  Quat`` also round-trips and can be selected instead)
* body rates -> ``Attitude Segment Quaternions`` (there is no
  ``Body Axes Angular Velocity`` provider on 12.6), reported in rad/s

Staleness vs failure
--------------------
An empty data-provider result is normally an error -- see
``data_providers.execute`` -- but there is one case where it is expected:
a vehicle on the **RealTime propagator** has ephemeris only within
Look-Ahead/Look-Behind of the samples injected so far, and STK 12.6
exposes no settable Look-Ahead property on that propagator, so the
window is whatever STK defaults to. Querying just past the newest
injected sample legitimately returns nothing.

The pipeline therefore distinguishes the two: for a real-time object an
empty result marks the channel **stale** and increments a counter; for
any other propagator it is still an error. Publishing the last-known
sample with a fresh timestamp would be the dangerous alternative -- a
frozen state that looks live.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

from stk_bridge.backends.base import (
    AngularRateSample,
    AttitudeSample,
    EclipseCondition,
    EclipseSample,
    LLASample,
    PropagatorType,
    StateSample,
    StkBackend,
    StkTime,
)
from stk_bridge.core import conversions as conv
from stk_bridge.core import data_providers as dp
from stk_bridge.core.frames import FrameRegistry
from stk_bridge.core.validation import ValidationReport
from stk_bridge.exceptions import ConventionValidationError, QueryError

LOGGER = logging.getLogger(__name__)


@dataclass
class ChannelHealth:
    """Freshness bookkeeping for one telemetry channel."""

    name: str
    samples: int = 0
    stale_reads: int = 0
    errors: int = 0
    last_sim_time_s: Optional[float] = None
    last_error: str = ""

    @property
    def is_stale(self) -> bool:
        return self.samples == 0 or self.stale_reads > 0


@dataclass
class SensingConfig:
    """Which objects and quantities to sample, and in which frames."""

    satellites: List[str] = field(default_factory=lambda: ["*/Satellite/SSS-1"])
    position_frame: str = "icrf"
    velocity_frame: str = "icrf"
    lla_frame: str = "fixed"
    #: Ground stations / facilities to compute access against.
    access_pairs: List[Tuple[str, str]] = field(default_factory=list)
    enable_lla: bool = True
    enable_lighting: bool = True
    enable_access: bool = False
    enable_link_budget: bool = False
    #: Force a specific attitude read-back provider, overriding the one
    #: validation proved correct. Empty (the default) means "use the
    #: validated one", which is the only setting a real run should use.
    #:
    #: This exists to make the ACAV case study's E1 runnable: it injects
    #: the convention bug found during the bridge's own bring-up -- an
    #: inertial read-back provider paired against a central-body-fixed
    #: command form -- so the study can measure what that costs. The
    #: mismatch differs by the Earth-rotation angle and nothing errors,
    #: which is precisely why it is worth an experiment and precisely
    #: why it must be impossible to switch on by accident. See
    #: docs/C1_case_study_spec.md section 9.
    force_attitude_provider: str = ""

    @classmethod
    def from_config(cls, config: Optional[Dict] = None) -> "SensingConfig":
        section = (config or {}).get("sensing", {}) or {}
        pairs = [
            (str(p["from"]), str(p["to"]))
            for p in (section.get("access_pairs") or [])
            if isinstance(p, dict) and "from" in p and "to" in p
        ]
        return cls(
            satellites=[conv.canonical_object_path(s) for s in
                        (section.get("satellites") or ["*/Satellite/SSS-1"])],
            position_frame=str(section.get("position_frame", "icrf")),
            velocity_frame=str(section.get("velocity_frame", "icrf")),
            lla_frame=str(section.get("lla_frame", "fixed")),
            access_pairs=pairs,
            enable_lla=bool(section.get("enable_lla", True)),
            enable_lighting=bool(section.get("enable_lighting", True)),
            enable_access=bool(section.get("enable_access", False)),
            enable_link_budget=bool(section.get("enable_link_budget", False)),
            force_attitude_provider=str(
                section.get("force_attitude_provider", "") or ""),
        )


#: Attitude read-back providers, in the order they are preferred. ECF
#: first: it is the counterpart of the ``CBFQuat`` command form the
#: actuation path uses by default.
_ATTITUDE_SPECS = (dp.ECF_ATTITUDE_QUATERNION, dp.ATTITUDE_QUATERNION,
                   dp.ATTITUDE_SEGMENT)


def _match_attitude_spec(name: str):
    """The provider spec ``name`` refers to, or None.

    Matches the canonical name or any of a spec's known alternatives,
    case-insensitively, because STK's provider naming drifts by version
    exactly like everything else in this project.
    """
    wanted = (name or "").strip().lower()
    if not wanted:
        return None
    for spec in _ATTITUDE_SPECS:
        candidates = {spec.provider_name.lower()}
        candidates.update(a.lower() for a in spec.provider_alternatives)
        if wanted in candidates:
            return spec
    return None


class SensingPipeline:
    """Turns STK data providers into typed samples.

    Stateless with respect to simulation time: every read takes the
    epoch it should sample at. That keeps the pipeline usable from both
    the lockstep loop (which passes the exact step time) and a free
    running loop (which passes wall-derived time) without branching.
    """

    def __init__(self, backend: StkBackend, frames: FrameRegistry,
                 config: SensingConfig,
                 report: Optional[ValidationReport] = None):
        self._backend = backend
        self._frames = frames
        self._config = config
        self._report = report or ValidationReport()
        self._health: Dict[str, ChannelHealth] = {}
        self._realtime_objects: Dict[str, bool] = {}
        self._read_failures = 0
        #: channel -> (total ms, reads), for the bridge's timing log.
        self.channel_timings: Dict[str, Tuple[float, int]] = {}

        self._attitude_spec = self._select_attitude_spec()
        self._rate_spec = self._select_rate_spec()

    # -- provider selection ------------------------------------------------

    def _select_attitude_spec(self) -> dp.ProviderSpec:
        """Pick the attitude provider validation proved works.

        Falls back to the ECF spec, which is the correct counterpart to
        the ``CBFQuat`` command form the actuation path uses by default.
        Choosing arbitrarily would risk the ECI/ECF mismatch that reads
        back a plausible-but-wrong rotation.

        ``force_attitude_provider`` overrides all of this, for E1 only --
        see the field's own documentation.
        """
        forced = (self._config.force_attitude_provider or "").strip()
        if forced:
            spec = _match_attitude_spec(forced)
            if spec is None:
                # Loudly, rather than falling back to the validated
                # provider: a typo here would produce a run that looks
                # like an E1 treatment, behaves like the nominal control,
                # and reports that mis-pairing the provider costs
                # nothing. A wrong finding is worse than a failed start.
                raise ConventionValidationError(
                    f"force_attitude_provider={forced!r} does not name a "
                    f"known attitude read-back provider. Expected one of: "
                    f"{[s.provider_name for s in _ATTITUDE_SPECS]}.")
            LOGGER.warning(
                "ATTITUDE READ-BACK PROVIDER FORCED to %r, overriding "
                "validation. Attitude will be read back through a "
                "provider that may not pair with the command form in "
                "use, and the error is the Earth-rotation angle with "
                "nothing raising. This is an E1 fault-injection setting: "
                "if you did not set it deliberately, clear "
                "sensing.force_attitude_provider.",
                spec.provider_name)
            return spec

        name = (self._report.attitude_provider or "").strip().lower()
        spec = _match_attitude_spec(name)
        if spec is not None:
            LOGGER.info("attitude provider (validated): %s",
                        self._report.attitude_provider)
            return spec
        LOGGER.warning(
            "no validated attitude provider on the report; defaulting to "
            "%r. Run core/validation.py to confirm the command/read-back "
            "pairing -- a mismatch differs by the Earth-rotation angle and "
            "is not otherwise detectable.",
            dp.ECF_ATTITUDE_QUATERNION.provider_name,
        )
        return dp.ECF_ATTITUDE_QUATERNION

    def _select_rate_spec(self) -> dp.ProviderSpec:
        name = (self._report.angular_rate_provider or "").strip().lower()
        if name and name == dp.ATTITUDE_SEGMENT.provider_name.lower():
            return dp.ATTITUDE_SEGMENT
        if name and name == dp.ANGULAR_VELOCITY.provider_name.lower():
            return dp.ANGULAR_VELOCITY
        # STK 12.6 has no "Body Axes Angular Velocity"; the segment
        # provider carries rates alongside the quaternion.
        return dp.ATTITUDE_SEGMENT

    @property
    def angle_rate_scale(self) -> conv.AngleRateScale:
        return self._report.angle_rate_scale

    # -- health ------------------------------------------------------------

    def _channel(self, name: str) -> ChannelHealth:
        if name not in self._health:
            self._health[name] = ChannelHealth(name=name)
        return self._health[name]

    def health(self) -> Dict[str, ChannelHealth]:
        return dict(self._health)

    #: How long a cached "is this object real-time?" answer is trusted
    #: before being re-checked. A permanent cache is unsafe: the only
    #: enforced invalidation point today is the SetPropagator service
    #: handler, and any other path that switches a propagator (direct
    #: script use of ActuationDispatcher.prepare_object, a future
    #: service, ScenarioManager called outside the normal startup
    #: sequence) would leave a stale answer with no invalidation call at
    #: all. The dangerous direction is caching True when the truth is
    #: now False: that would silently reclassify a genuine query error
    #: as mere staleness (returns None) instead of raising -- exactly
    #: the kind of masked failure this whole module exists to prevent.
    #: A bounded TTL caps the exposure instead of requiring every write
    #: path to remember to invalidate.
    _REALTIME_CACHE_TTL_S = 5.0

    def _is_realtime(self, object_path: str) -> bool:
        """Is this object on the RealTime propagator?

        Cached with a TTL (see ``_REALTIME_CACHE_TTL_S``): re-querying on
        every read would double the query count on the fast rate group,
        but caching forever risks a stale answer surviving an
        out-of-band propagator change. ``invalidate_propagator_cache``
        remains the fast, immediate path for callers that know a change
        happened (the SetPropagator service uses it); the TTL is the
        safety net for callers that don't.
        """
        import time as _time

        cached = self._realtime_objects.get(object_path)
        now = _time.monotonic()
        if cached is not None and (now - cached[1]) < self._REALTIME_CACHE_TTL_S:
            return cached[0]

        realtime = False
        try:
            for info in self._backend.list_objects():
                if conv.canonical_object_path(info.object_path) == object_path:
                    realtime = (info.propagator_type or "").lower() in (
                        "realtime", PropagatorType.REALTIME.value.lower()
                    )
                    break
        except Exception:
            LOGGER.debug("could not determine propagator for %s",
                         object_path, exc_info=True)
        self._realtime_objects[object_path] = (realtime, now)
        return realtime

    def invalidate_propagator_cache(self, object_path: Optional[str] = None
                                    ) -> None:
        if object_path is None:
            self._realtime_objects.clear()
        else:
            self._realtime_objects.pop(
                conv.canonical_object_path(object_path), None)

    def _read(self, channel: str, spec: dp.ProviderSpec, object_path: str,
              when: StkTime, frame: str = "ICRF"):
        """Execute a spec, classifying emptiness as stale or as failure."""
        health = self._channel(channel)
        realtime = self._is_realtime(object_path)
        try:
            data, mapping = dp.execute(
                self._backend, spec, object_path, frame=frame,
                start=when, stop=when, allow_empty=realtime,
            )
        except QueryError as exc:
            health.errors += 1
            health.last_error = str(exc)
            raise

        if data.row_count == 0:
            health.stale_reads += 1
            health.last_error = (
                "no data at this epoch; the object is on the RealTime "
                "propagator and this time is outside its Look-Ahead/"
                "Look-Behind window"
            )
            return None, None
        return data, mapping

    # -- typed reads -------------------------------------------------------

    def read_orbit(self, object_path: str, when: StkTime,
                   frame: Optional[str] = None) -> Optional[StateSample]:
        """Position and velocity in one sample.

        Two provider calls, deliberately: STK's Cartesian Position and
        Cartesian Velocity are separate providers, and combining them
        into a single "Cartesian State" query is not available on 12.6.
        """
        frame_id = frame or self._config.position_frame
        stk_frame = self._frames.stk_name(frame_id)

        data, mapping = self._read("orbit_position", dp.CARTESIAN_POSITION,
                                   object_path, when, stk_frame)
        if data is None:
            return None
        position = (
            dp.first_value(data, dp.CARTESIAN_POSITION, mapping, "x"),
            dp.first_value(data, dp.CARTESIAN_POSITION, mapping, "y"),
            dp.first_value(data, dp.CARTESIAN_POSITION, mapping, "z"),
        )

        velocity = (0.0, 0.0, 0.0)
        vdata, vmapping = self._read("orbit_velocity", dp.CARTESIAN_VELOCITY,
                                     object_path, when, stk_frame)
        if vdata is not None:
            velocity = (
                dp.first_value(vdata, dp.CARTESIAN_VELOCITY, vmapping, "vx"),
                dp.first_value(vdata, dp.CARTESIAN_VELOCITY, vmapping, "vy"),
                dp.first_value(vdata, dp.CARTESIAN_VELOCITY, vmapping, "vz"),
            )

        health = self._channel("orbit_position")
        health.samples += 1
        health.last_sim_time_s = when.epoch_seconds

        return StateSample(
            time=when, frame=self._frames.ros_id(frame_id),
            position_m=position, velocity_mps=velocity,
            is_extrapolated=self._is_realtime(object_path),
        )

    def read_attitude(self, object_path: str, when: StkTime
                      ) -> Optional[AttitudeSample]:
        """Body attitude, in the frame the validated provider reports.

        The frame is a property of the provider, not a free parameter:
        ``ECF Attitude Quaternions`` is always central-body-fixed and
        ``Attitude Quaternions`` is always inertial, and both are
        ungrouped on 12.6 so passing a frame would be silently ignored.
        """
        spec = self._attitude_spec
        data, mapping = self._read("attitude", spec, object_path, when)
        if data is None:
            return None

        quaternion = conv.quaternion_from_stk_elements(
            dp.first_value(data, spec, mapping, "q1"),
            dp.first_value(data, spec, mapping, "q2"),
            dp.first_value(data, spec, mapping, "q3"),
            dp.first_value(data, spec, mapping, "q4"),
        )
        if not conv.is_unit_quaternion(quaternion, tolerance=1e-3):
            quaternion = conv.normalize_quaternion(quaternion)

        is_ecf = spec is dp.ECF_ATTITUDE_QUATERNION
        frame_id = self._frames.ros_id("fixed" if is_ecf else "icrf")

        health = self._channel("attitude")
        health.samples += 1
        health.last_sim_time_s = when.epoch_seconds

        return AttitudeSample(
            time=when, frame=frame_id, quaternion_xyzw=quaternion,
            is_extrapolated=self._is_realtime(object_path),
        )

    def read_angular_rate(self, object_path: str, when: StkTime
                          ) -> Optional[AngularRateSample]:
        """Body angular velocity, scaled to rad/s by measured factor."""
        spec = self._rate_spec
        data, mapping = self._read("angular_rate", spec, object_path, when)
        if data is None:
            return None

        scale = self.angle_rate_scale
        try:
            rate = (
                dp.first_value(data, spec, mapping, "wx",
                               angle_rate_scale=scale),
                dp.first_value(data, spec, mapping, "wy",
                               angle_rate_scale=scale),
                dp.first_value(data, spec, mapping, "wz",
                               angle_rate_scale=scale),
            )
        except QueryError:
            # The segment provider carries rates as optional fields; an
            # attitude profile that does not define them yields no
            # columns. Zero is the honest answer here (no commanded
            # rate), not an error.
            self._channel("angular_rate").stale_reads += 1
            return None

        health = self._channel("angular_rate")
        health.samples += 1
        health.last_sim_time_s = when.epoch_seconds
        return AngularRateSample(time=when, angular_velocity_radps=rate)

    def read_lla(self, object_path: str, when: StkTime
                 ) -> Optional[LLASample]:
        stk_frame = self._frames.stk_name(self._config.lla_frame)
        data, mapping = self._read("lla", dp.LLA_STATE, object_path, when,
                                   stk_frame)
        if data is None:
            return None
        health = self._channel("lla")
        health.samples += 1
        health.last_sim_time_s = when.epoch_seconds
        return LLASample(
            time=when,
            latitude_rad=dp.first_value(data, dp.LLA_STATE, mapping,
                                        "latitude"),
            longitude_rad=dp.first_value(data, dp.LLA_STATE, mapping,
                                         "longitude"),
            altitude_m=dp.first_value(data, dp.LLA_STATE, mapping, "altitude"),
        )

    def read_eclipse(self, object_path: str, when: StkTime
                     ) -> Optional[EclipseSample]:
        """Instantaneous lighting condition.

        On STK 12.6 ``Lighting Times`` is *grouped by condition* --
        members ``Sunlight``, ``Penumbra``, ``Umbra`` -- each returning
        intervals, rather than a single provider with a ``Condition``
        column. So the condition at an epoch is determined by testing
        interval membership, cheapest-likely-first.
        """
        health = self._channel("eclipse")
        for condition, enum_value in (
            ("Sunlight", EclipseCondition.SUNLIT),
            ("Penumbra", EclipseCondition.PENUMBRA),
            ("Umbra", EclipseCondition.UMBRA),
        ):
            try:
                data = self._backend.query_data_provider(
                    dp.DataProviderQuery(
                        object_path=object_path,
                        provider_name=dp.LIGHTING.provider_name,
                        group_element=condition,
                        start=when, stop=when,
                    )
                )
            except QueryError:
                continue
            if data.row_count > 0:
                health.samples += 1
                health.last_sim_time_s = when.epoch_seconds
                return EclipseSample(time=when, condition=enum_value)

        health.stale_reads += 1
        return None

    # -- aggregate ---------------------------------------------------------

    #: The fast-group channels ``read_all`` reads, in order.
    FAST_CHANNELS = ("orbit", "attitude", "angular_rate")

    def read_all(self, object_path: str, when: StkTime,
                 channels=None) -> Dict[str, object]:
        """Every fast-group quantity for one object.

        Returns whatever succeeded. A missing key means that channel was
        stale or errored at this epoch -- callers publish only the keys
        present rather than substituting defaults, so a consumer can
        tell "no attitude yet" from "attitude is identity".

        ``channels`` limits the read to a subset, so a caller can retry
        just the ones that failed. Wall time per channel is accumulated
        in ``channel_timings`` (reset by the caller that reports it).
        """
        readers = {"orbit": self.read_orbit,
                   "attitude": self.read_attitude,
                   "angular_rate": self.read_angular_rate}
        result: Dict[str, object] = {}
        for key in self.FAST_CHANNELS:
            if channels is not None and key not in channels:
                continue
            started = time.perf_counter()
            try:
                sample = readers[key](object_path, when)
            except QueryError as exc:
                # Not DEBUG: in lockstep a participant waiting for this
                # channel's telemetry never acks, and the barrier timeout
                # names the participant rather than the failed read.
                self._read_failures += 1
                if str(self._read_failures).rstrip("0") == "1":
                    LOGGER.warning("%s read failed on %s: %s [failure %d]",
                                   key, object_path, exc,
                                   self._read_failures)
                continue
            finally:
                total, count = self.channel_timings.get(key, (0.0, 0))
                self.channel_timings[key] = (
                    total + (time.perf_counter() - started) * 1000.0,
                    count + 1)
            if sample is not None:
                result[key] = sample
        return result

    def statistics(self) -> Dict[str, Dict]:
        return {
            name: {
                "samples": health.samples,
                "stale_reads": health.stale_reads,
                "errors": health.errors,
                "last_sim_time_s": health.last_sim_time_s,
                "last_error": health.last_error,
            }
            for name, health in self._health.items()
        }
