"""Pushes commands from ROS into STK.

Three real-time channels, all append-only and all sharing one hard
constraint: **timestamps must be strictly increasing per object**.

============================  ==========================================
ROS input                     STK mechanism
============================  ==========================================
``/stk/cmd/position``         ``SetPosition`` (RealTime propagator)
``/stk/cmd/attitude``         ``AddAttitude CBFQuat`` / ``Quat``
``/stk/cmd/angular_velocity``  ``AddAttitude AngVels``
============================  ==========================================

Why the monotonicity guard stays
--------------------------------
Validation against a live STK 12.6 install confirmed that out-of-order
attitude samples are **NACKed**, not silently dropped -- better than
originally assumed, and recorded on the validation report as
``monotonicity_is_observable``. The guard is kept anyway, for three
reasons that survive that finding:

1. It converts a round trip into a local rejection. At 10 Hz with a
   controller that occasionally re-sends, the saved COM calls matter.
2. It gives a *counter* (``dropped_*``) rather than a log line, so
   ``BridgeStatus`` can report the rate of loss instead of an operator
   having to grep.
3. It is uniform across backends. The Connect backend's behaviour was
   not separately confirmed, and the ``SetPosition`` path was not tested
   for NACK-vs-silent at all, so relying on STK to complain would make
   correctness backend-dependent.

Ordering constraint on angular velocity
---------------------------------------
``AddAttitude AngVels`` requires at least one prior attitude sample for
the object, or STK rejects it. The dispatcher enforces this ordering
itself rather than letting the rejection surface as a mysterious
NACK several layers up.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

from stk_bridge.backends.base import (
    PropagatorType,
    StkBackend,
    StkTime,
)
from stk_bridge.core import conversions as conv
from stk_bridge.core.frames import FrameRegistry
from stk_bridge.exceptions import (
    CommandError,
    MonotonicityViolationError,
    StkBridgeError,
)

LOGGER = logging.getLogger(__name__)


@dataclass
class ChannelState:
    """Per-object, per-channel bookkeeping."""

    last_stamp_s: Optional[float] = None
    accepted: int = 0
    dropped_non_monotonic: int = 0
    rejected_by_stk: int = 0
    last_error: str = ""


@dataclass
class ObjectState:
    position: ChannelState = field(default_factory=ChannelState)
    attitude: ChannelState = field(default_factory=ChannelState)
    angular_rate: ChannelState = field(default_factory=ChannelState)

    realtime_attitude_enabled: bool = False
    realtime_propagator_set: bool = False
    has_attitude_sample: bool = False


class ActuationDispatcher:
    """Validates and issues real-time commands to STK."""

    def __init__(self, backend: StkBackend, frames: FrameRegistry,
                 default_attitude_inertial: bool = False,
                 auto_enable_realtime: bool = True,
                 min_step_s: float = 0.0):
        self._backend = backend
        self._frames = frames
        self._default_inertial = default_attitude_inertial
        self._auto_enable = auto_enable_realtime
        #: Reject samples closer together than this. 0.0 disables. Useful
        #: when a publisher bursts faster than STK can usefully ingest.
        self._min_step_s = min_step_s
        self._objects: Dict[str, ObjectState] = {}

    # -- state -------------------------------------------------------------

    def _state(self, object_path: str) -> ObjectState:
        key = conv.canonical_object_path(object_path)
        if key not in self._objects:
            self._objects[key] = ObjectState()
        return self._objects[key]

    def reset(self, object_path: Optional[str] = None) -> None:
        """Forget this dispatcher's own last-stamp bookkeeping.

        This clears LOCAL state only. It does not touch STK -- callers
        must call ``backend.clear_attitude_data()`` (or
        ``ScenarioManager.reset()``, which does that for every real-time
        object) BEFORE this, not after. Calling this alone leaves the
        guard willing to accept an earlier timestamp while STK itself
        still remembers the newer one and will reject it -- a real
        failure mode, not a hypothetical: see
        test_actuation.py::test_reset_alone_does_not_clear_stk_side_data,
        added after this exact gap surfaced in the test suite.

        The correct order, and the one ``interfaces/ros_services.py``'s
        ``ResetScenario`` handler follows: clear STK-side data first,
        then call this to match the guard's memory to it.
        """
        if object_path is None:
            self._objects.clear()
            return
        self._objects.pop(conv.canonical_object_path(object_path), None)

    # -- guards ------------------------------------------------------------

    def _check_monotonic(self, object_path: str, channel: ChannelState,
                         when: StkTime, channel_name: str) -> None:
        stamp = when.epoch_seconds
        last = channel.last_stamp_s
        if last is None:
            return
        if stamp <= last:
            channel.dropped_non_monotonic += 1
            raise MonotonicityViolationError(object_path, stamp, last)
        if self._min_step_s > 0.0 and (stamp - last) < self._min_step_s:
            channel.dropped_non_monotonic += 1
            raise MonotonicityViolationError(object_path, stamp,
                                             last + self._min_step_s)

    def _ensure_realtime_attitude(self, object_path: str,
                                  state: ObjectState) -> None:
        """Activate the real-time attitude profile once, lazily.

        Lazy rather than at setup because the bridge should not mutate a
        scenario's attitude configuration for an object nobody ever
        commands. Idempotent in STK, but tracked locally to avoid a
        redundant COM call on every sample.
        """
        if state.realtime_attitude_enabled:
            return
        if not self._auto_enable:
            raise CommandError(
                "AddAttitude", object_path,
                "real-time attitude is not enabled and auto_enable_realtime "
                "is off; call SetPropagator or enable it explicitly first",
            )
        self._backend.enable_realtime_attitude(object_path)
        state.realtime_attitude_enabled = True
        LOGGER.info("real-time attitude enabled on %s", object_path)

    # -- commands ----------------------------------------------------------

    def set_position(self, object_path: str, when: StkTime, frame_id: str,
                     position_m: Tuple[float, float, float],
                     velocity_mps: Optional[Tuple[float, float, float]] = None
                     ) -> bool:
        """Inject one real-time position sample. True if accepted."""
        path = conv.canonical_object_path(object_path)
        state = self._state(path)
        frame = self._frames.require_position_reference(frame_id)

        try:
            self._check_monotonic(path, state.position, when, "position")
        except MonotonicityViolationError as exc:
            state.position.last_error = str(exc)
            LOGGER.debug("position sample rejected locally: %s", exc)
            return False

        try:
            self._backend.set_position(
                path, when, frame.stk_name, position_m, velocity_mps
            )
        except StkBridgeError as exc:
            state.position.rejected_by_stk += 1
            state.position.last_error = str(exc)
            LOGGER.warning("SetPosition rejected on %s: %s", path, exc)
            return False

        state.position.last_stamp_s = when.epoch_seconds
        state.position.accepted += 1
        return True

    def set_attitude(self, object_path: str, when: StkTime,
                     quaternion_xyzw: Tuple[float, float, float, float],
                     frame_id: Optional[str] = None,
                     inertial: Optional[bool] = None) -> bool:
        """Inject one real-time attitude sample. True if accepted.

        ``frame_id`` and ``inertial`` are two ways of saying the same
        thing; ``frame_id`` wins if both are given, because a named
        frame is checkable and a bare boolean is not. If neither is
        given the dispatcher's default applies -- ``CBFQuat`` unless
        configured otherwise, matching the pairing validation confirmed
        against this install.
        """
        path = conv.canonical_object_path(object_path)
        state = self._state(path)

        if frame_id:
            use_inertial = self._frames.is_inertial_attitude_reference(frame_id)
        elif inertial is not None:
            use_inertial = bool(inertial)
        else:
            use_inertial = self._default_inertial

        try:
            normalized = conv.normalize_quaternion(quaternion_xyzw)
        except ValueError as exc:
            state.attitude.rejected_by_stk += 1
            state.attitude.last_error = str(exc)
            LOGGER.warning("attitude command on %s is invalid: %s", path, exc)
            return False

        try:
            self._check_monotonic(path, state.attitude, when, "attitude")
        except MonotonicityViolationError as exc:
            state.attitude.last_error = str(exc)
            LOGGER.debug("attitude sample rejected locally: %s", exc)
            return False

        try:
            self._ensure_realtime_attitude(path, state)
            self._backend.add_attitude_quaternion(
                path, when, normalized, inertial=use_inertial
            )
        except StkBridgeError as exc:
            state.attitude.rejected_by_stk += 1
            state.attitude.last_error = str(exc)
            LOGGER.warning("AddAttitude rejected on %s: %s", path, exc)
            return False

        state.attitude.last_stamp_s = when.epoch_seconds
        state.attitude.accepted += 1
        state.has_attitude_sample = True
        return True

    def set_angular_velocity(self, object_path: str, when: StkTime,
                             angular_velocity_radps: Tuple[float, float, float]
                             ) -> bool:
        """Inject one real-time body-rate sample. True if accepted.

        Refuses if no attitude sample exists yet for this object, since
        ``AddAttitude AngVels`` requires one. Enforcing the ordering here
        produces an actionable message instead of STK's generic failure.
        """
        path = conv.canonical_object_path(object_path)
        state = self._state(path)

        if not state.has_attitude_sample:
            state.angular_rate.rejected_by_stk += 1
            state.angular_rate.last_error = (
                "no prior attitude sample; AddAttitude AngVels requires at "
                "least one AddAttitude Quat/CBFQuat sample first"
            )
            LOGGER.warning("angular-velocity command on %s before any "
                           "attitude sample; dropping", path)
            return False

        try:
            self._check_monotonic(path, state.angular_rate, when, "rate")
        except MonotonicityViolationError as exc:
            state.angular_rate.last_error = str(exc)
            return False

        try:
            self._backend.add_attitude_angular_velocity(
                path, when, angular_velocity_radps
            )
        except StkBridgeError as exc:
            state.angular_rate.rejected_by_stk += 1
            state.angular_rate.last_error = str(exc)
            LOGGER.warning("AddAttitude AngVels rejected on %s: %s", path, exc)
            return False

        state.angular_rate.last_stamp_s = when.epoch_seconds
        state.angular_rate.accepted += 1
        return True

    def clear_attitude(self, object_path: str) -> None:
        path = conv.canonical_object_path(object_path)
        self._backend.clear_attitude_data(path)
        state = self._state(path)
        state.attitude = ChannelState()
        state.angular_rate = ChannelState()
        state.has_attitude_sample = False
        LOGGER.info("cleared real-time attitude data on %s", path)

    def prepare_object(self, object_path: str,
                       look_ahead_s: float = 3600.0,
                       look_behind_s: float = 3600.0) -> None:
        """Put an object into the state the live control path needs.

        Sets the RealTime orbit propagator and activates real-time
        attitude. Note that this **discards the object's analytic
        ephemeris** -- after this call it has state only where samples
        have been injected. That is inherent to the RealTime propagator,
        not a bridge choice, and is why sensing treats empty reads on
        such objects as staleness rather than error.
        """
        path = conv.canonical_object_path(object_path)
        state = self._state(path)

        self._backend.set_propagator(
            path, PropagatorType.REALTIME,
            look_ahead_s=look_ahead_s, look_behind_s=look_behind_s,
        )
        state.realtime_propagator_set = True

        self._backend.enable_realtime_attitude(path)
        state.realtime_attitude_enabled = True

        LOGGER.info(
            "%s prepared for real-time control (analytic ephemeris is now "
            "discarded; state exists only where samples are injected)", path,
        )

    # -- introspection -----------------------------------------------------

    def statistics(self) -> Dict[str, Dict]:
        out: Dict[str, Dict] = {}
        for path, state in self._objects.items():
            out[path] = {
                channel_name: {
                    "accepted": channel.accepted,
                    "dropped_non_monotonic": channel.dropped_non_monotonic,
                    "rejected_by_stk": channel.rejected_by_stk,
                    "last_stamp_s": channel.last_stamp_s,
                    "last_error": channel.last_error,
                }
                for channel_name, channel in (
                    ("position", state.position),
                    ("attitude", state.attitude),
                    ("angular_rate", state.angular_rate),
                )
            }
        return out

    def total_dropped(self) -> Dict[str, int]:
        """Aggregate counters for ``BridgeStatus``."""
        totals = {"position": 0, "attitude": 0, "angular_rate": 0}
        for state in self._objects.values():
            totals["position"] += (state.position.dropped_non_monotonic
                                   + state.position.rejected_by_stk)
            totals["attitude"] += (state.attitude.dropped_non_monotonic
                                   + state.attitude.rejected_by_stk)
            totals["angular_rate"] += (
                state.angular_rate.dropped_non_monotonic
                + state.angular_rate.rejected_by_stk)
        return totals
