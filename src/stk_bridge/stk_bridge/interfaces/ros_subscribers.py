"""Command subscribers and the command inbox.

Subscriber callbacks run on executor threads; STK access runs on the COM
apartment thread; the step loop runs on its own thread. Rather than have
callbacks reach across all three, they deposit commands into a
:class:`CommandInbox` and return immediately. The step loop drains it at
a well-defined point in the cycle.

Why buffer rather than dispatch directly
----------------------------------------
Two reasons, one per execution mode.

In **lockstep**, a command must be applied to the step it was computed
for. Dispatching on arrival would race the step boundary: a command
published for step N could land after the bridge had already advanced to
N+1, silently shifting the control input by one sample time. Draining at
the top of each step makes the association explicit.

In **free-running**, dispatching directly from a callback would let a
fast publisher queue unbounded work onto the COM thread, and because
STK's real-time channels are append-only that backlog turns into
mounting latency rather than dropped work. The inbox applies
newest-wins coalescing per object and channel, so a burst collapses to
the sample that actually matters.
"""

from __future__ import annotations

import logging
import threading
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

from stk_bridge.backends.base import StkTime
from stk_bridge.core import conversions as conv

LOGGER = logging.getLogger(__name__)


@dataclass
class PositionCommand:
    object_path: str
    time: StkTime
    frame_id: str
    position_m: Tuple[float, float, float]
    velocity_mps: Optional[Tuple[float, float, float]] = None


@dataclass
class AttitudeCommand:
    object_path: str
    time: StkTime
    quaternion_xyzw: Tuple[float, float, float, float]
    frame_id: str = ""
    inertial: Optional[bool] = None


@dataclass
class AngularRateCommand:
    object_path: str
    time: StkTime
    angular_velocity_radps: Tuple[float, float, float]


@dataclass
class InboxStatistics:
    received: int = 0
    coalesced: int = 0
    drained: int = 0


class CommandInbox:
    """Thread-safe, newest-wins command buffer.

    Keyed by ``(channel, object_path)``. A second command for the same
    key before a drain replaces the first and increments ``coalesced``.

    Coalescing is correct here specifically because these are *state*
    commands, not incremental ones: the newest attitude fully supersedes
    the previous one. It would be wrong for anything cumulative, which
    is why maneuvers go through an action rather than this path.
    """

    def __init__(self, coalesce: bool = True):
        self._lock = threading.Lock()
        self._changed = threading.Condition(self._lock)
        self._pending: Dict[Tuple[str, str], object] = {}
        self._order: List[Tuple[str, str]] = []
        self._coalesce = coalesce
        self._stats = InboxStatistics()

    def put(self, channel: str, command) -> None:
        key = (channel, conv.canonical_object_path(command.object_path))
        with self._lock:
            self._stats.received += 1
            if key in self._pending:
                if self._coalesce:
                    self._stats.coalesced += 1
                    self._pending[key] = command
                    self._changed.notify_all()
                    return
                # Not coalescing: the first command for this key is kept
                # until the next drain and later ones are discarded.
                return
            self._pending[key] = command
            self._order.append(key)
            self._changed.notify_all()

    def wait_for(self, channel, object_path, min_time_s, timeout_s):
        """Compatibility gate for configurations using await_attitude_commands."""
        key = (channel, conv.canonical_object_path(object_path))
        with self._changed:
            return self._changed.wait_for(
                lambda: key in self._pending and
                self._pending[key].time.epoch_seconds >= min_time_s,
                timeout=timeout_s)

    def wait_for_commands(self, channels, paths, stamp, timeout_s):
        """Wait for exact next-step commands, independently of barrier acks."""
        expected = [(channel, conv.canonical_object_path(path))
                    for channel in channels for path in paths]

        def missing():
            return [f"{channel} for {path}" for channel, path in expected
                    if (channel, path) not in self._pending or
                    abs(self._pending[(channel, path)].time.epoch_seconds
                        - stamp.epoch_seconds) > 1e-4]

        with self._changed:
            self._changed.wait_for(lambda: not missing(), timeout=timeout_s)
            return missing()

    def drain(self) -> List[Tuple[str, object]]:
        """Remove and return all pending commands, in arrival order."""
        with self._lock:
            items = [(key[0], self._pending[key]) for key in self._order
                     if key in self._pending]
            self._pending.clear()
            self._order.clear()
            self._stats.drained += len(items)
            return items

    def clear(self) -> None:
        with self._lock:
            self._pending.clear()
            self._order.clear()

    @property
    def depth(self) -> int:
        with self._lock:
            return len(self._pending)

    def statistics(self) -> Dict[str, int]:
        with self._lock:
            return {
                "received": self._stats.received,
                "coalesced": self._stats.coalesced,
                "drained": self._stats.drained,
                "depth": len(self._pending),
            }


class CommandSubscribers:
    """Creates the command topics and routes them into the inbox."""

    def __init__(self, node, inbox: CommandInbox,
                 qos_profiles: Dict[str, object],
                 prefix: str = "stk",
                 callback_group=None,
                 default_frame: str = "icrf",
                 on_barrier_ack: Optional[Callable[[str, int], None]] = None):
        from std_msgs.msg import String
        from stk_msgs.msg import (
            AttitudeCommand as AttitudeCommandMsg,
            AngularRate,
            PositionCommand as PositionCommandMsg,
        )

        self._node = node
        self._inbox = inbox
        self._default_frame = default_frame
        self._on_barrier_ack = on_barrier_ack
        command_qos = qos_profiles["command"]

        self._sub_position = node.create_subscription(
            PositionCommandMsg, f"{prefix}/cmd/position",
            self._on_position, command_qos, callback_group=callback_group)
        self._sub_attitude = node.create_subscription(
            AttitudeCommandMsg, f"{prefix}/cmd/attitude",
            self._on_attitude, command_qos, callback_group=callback_group)
        self._sub_rate = node.create_subscription(
            AngularRate, f"{prefix}/cmd/angular_velocity",
            self._on_angular_rate, command_qos, callback_group=callback_group)

        # The lockstep acknowledgement. Deliberately a plain String on a
        # separate topic with its own callback group: it must be
        # serviceable while the step loop is blocked waiting for it, and
        # sharing a mutually-exclusive group with the command topics
        # would let a burst of commands delay the ack that releases the
        # barrier.
        self._sub_ack = node.create_subscription(
            String, f"{prefix}/lockstep/ack",
            self._on_ack, command_qos, callback_group=callback_group)

        LOGGER.info("command subscribers created under %s/cmd/", prefix)

    # -- callbacks ---------------------------------------------------------

    def _on_position(self, message) -> None:
        try:
            command = PositionCommand(
                object_path=message.object_path,
                time=conv.ros_time_to_stk(message.header.stamp),
                frame_id=(message.header.frame_id or self._default_frame),
                position_m=(message.position.x, message.position.y,
                            message.position.z),
                velocity_mps=(
                    (message.velocity.x, message.velocity.y,
                     message.velocity.z)
                    if message.velocity_valid else None
                ),
            )
        except Exception:
            LOGGER.exception("malformed PositionCommand; dropping")
            return
        self._inbox.put("position", command)

    def _on_attitude(self, message) -> None:
        try:
            command = AttitudeCommand(
                object_path=message.object_path,
                time=conv.ros_time_to_stk(message.header.stamp),
                quaternion_xyzw=(
                    message.orientation.x, message.orientation.y,
                    message.orientation.z, message.orientation.w,
                ),
                frame_id=message.header.frame_id or "",
                inertial=bool(message.use_inertial_frame),
            )
        except Exception:
            LOGGER.exception("malformed AttitudeCommand; dropping")
            return
        self._inbox.put("attitude", command)

    def _on_angular_rate(self, message) -> None:
        try:
            command = AngularRateCommand(
                object_path=message.object_path,
                time=conv.ros_time_to_stk(message.header.stamp),
                angular_velocity_radps=(
                    message.angular_velocity.x,
                    message.angular_velocity.y,
                    message.angular_velocity.z,
                ),
            )
        except Exception:
            LOGGER.exception("malformed AngularRate command; dropping")
            return
        self._inbox.put("angular_rate", command)

    def _on_ack(self, message) -> None:
        """Parse ``"<participant>:<step_index>"``.

        A tiny ad-hoc encoding rather than a custom message, because the
        ack must be trivially publishable from MATLAB, a shell, or a test
        harness without regenerating interfaces. Malformed acks are
        logged and ignored rather than raising -- a broken ack should
        cause a barrier timeout with a clear cause, not kill the
        subscriber.
        """
        if self._on_barrier_ack is None:
            return
        text = str(message.data).strip()
        if ":" not in text:
            LOGGER.warning("malformed lockstep ack %r; expected "
                           "'<participant>:<step_index>'", text)
            return
        participant, _, index_text = text.partition(":")
        try:
            step_index = int(index_text)
        except ValueError:
            LOGGER.warning("lockstep ack %r has a non-integer step index",
                           text)
            return
        self._on_barrier_ack(participant.strip(), step_index)


def apply_commands(dispatcher, commands: List[Tuple[str, object]]
                   ) -> Dict[str, int]:
    """Apply drained commands via the actuation dispatcher.

    Attitude is applied before angular rate, unconditionally: STK's
    ``AddAttitude AngVels`` requires a prior attitude sample, and a
    controller that publishes both for the same step has no way to
    guarantee subscriber callback ordering across topics.
    """
    priority = {"position": 0, "attitude": 1, "angular_rate": 2}
    ordered = sorted(commands, key=lambda item: priority.get(item[0], 9))

    applied = {"position": 0, "attitude": 0, "angular_rate": 0}
    rejected = {"position": 0, "attitude": 0, "angular_rate": 0}

    for channel, command in ordered:
        if channel == "position":
            ok = dispatcher.set_position(
                command.object_path, command.time, command.frame_id,
                command.position_m, command.velocity_mps)
        elif channel == "attitude":
            ok = dispatcher.set_attitude(
                command.object_path, command.time, command.quaternion_xyzw,
                frame_id=command.frame_id or None,
                inertial=command.inertial)
        elif channel == "angular_rate":
            ok = dispatcher.set_angular_velocity(
                command.object_path, command.time,
                command.angular_velocity_radps)
        else:
            continue
        if ok:
            applied[channel] += 1
        else:
            rejected[channel] += 1

    return {"applied": applied, "rejected": rejected}
