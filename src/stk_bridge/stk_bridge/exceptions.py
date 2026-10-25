"""Exception taxonomy for stk_bridge.

Every exception a backend can raise is declared here, so that ``core/``
and ``interfaces/`` can catch by category without importing anything
backend-specific (``agi.stk12``, ``win32com`` and ``pythoncom`` stay
confined to ``backends/``; see docs/bridge/architecture.md).

Design intent
-------------
- StkBridgeError is the common root; catching it at the top of bridge_node's
  executor callback is always safe and never masks a programming error,
  because programming errors should be plain Python exceptions (TypeError,
  KeyError, ...), not subclasses of this hierarchy.
- Exceptions carry enough structured context (object_path, command, etc.)
  for BridgeStatus.last_error and log messages to be actionable without
  the caller needing to parse a string.
- BackendUnavailableError and LicenseError are deliberately distinct from
  CommandError/QueryError: the former mean "don't retry, escalate", the
  latter mean "this one call failed, the backend is still fine".
"""

from __future__ import annotations

from typing import Optional


class StkBridgeError(Exception):
    """Root of the stk_bridge exception hierarchy."""


# ---------------------------------------------------------------------------
# Backend lifecycle
# ---------------------------------------------------------------------------

class BackendError(StkBridgeError):
    """Base class for all backend-layer failures."""


class BackendUnavailableError(BackendError):
    """The backend could not be reached or initialized at all.

    Raised by StkBackend.connect() implementations. Distinguished from
    ConnectionLostError because it means "never connected", not
    "was connected, now isn't" -- callers generally retry the latter with
    backoff and fail fast on the former.
    """

    def __init__(self, backend_type: str, reason: str):
        self.backend_type = backend_type
        self.reason = reason
        super().__init__(f"[{backend_type}] backend unavailable: {reason}")


class ConnectionLostError(BackendError):
    """A previously-working backend connection has dropped.

    core/health.py treats this as the trigger to transition BridgeStatus
    into DISCONNECTED and to stop draining the command queue until
    reconnection succeeds (see core/com_thread.py's reconnect policy).
    """

    def __init__(self, backend_type: str, reason: str):
        self.backend_type = backend_type
        self.reason = reason
        super().__init__(f"[{backend_type}] connection lost: {reason}")


class LicenseError(BackendError):
    """A required STK capability is not licensed.

    Raised eagerly at startup (scenario_manager.py's capability probe) and
    also at the point of use for capabilities that can't be cheaply probed
    up front (e.g. Astrogator segment types). Action servers
    (ExecuteManeuver in particular) catch this specifically to reject the
    goal immediately rather than letting it fail deep inside a Connect call.
    """

    def __init__(self, capability: str, detail: Optional[str] = None):
        self.capability = capability
        self.detail = detail
        msg = f"required STK capability not licensed: {capability}"
        if detail:
            msg += f" ({detail})"
        super().__init__(msg)


# ---------------------------------------------------------------------------
# Per-call failures
# ---------------------------------------------------------------------------

class CommandError(BackendError):
    """An actuation command (SetPosition, AddAttitude, ExecuteCommand, ...)
    was rejected or NACKed by STK.

    Distinct from ConnectionLostError: the backend is fine, this specific
    command was invalid or rejected. core/actuation.py counts it as a
    dropped sample rather than tearing down the connection.
    """

    def __init__(self, command: str, object_path: str, reason: str):
        self.command = command
        self.object_path = object_path
        self.reason = reason
        super().__init__(
            f"command '{command}' on {object_path!r} failed: {reason}"
        )


class MonotonicityViolationError(CommandError):
    """A SetPosition/AddAttitude sample arrived with a timestamp <= the
    last accepted timestamp for this object.

    Raised by core/actuation.py before the command reaches the backend, so
    an out-of-order sample is counted locally
    (``BridgeStatus.dropped_attitude_samples`` /
    ``dropped_position_samples``) independently of how a backend reports
    it.
    """

    def __init__(self, object_path: str, rejected_stamp: float,
                 last_accepted_stamp: float):
        self.rejected_stamp = rejected_stamp
        self.last_accepted_stamp = last_accepted_stamp
        super().__init__(
            command="AddAttitude/SetPosition",
            object_path=object_path,
            reason=(
                f"non-monotonic sample: stamp={rejected_stamp} <= "
                f"last_accepted={last_accepted_stamp}"
            ),
        )


class QueryError(BackendError):
    """A data provider or Connect report query failed."""

    def __init__(self, provider_name: str, object_path: str, reason: str):
        self.provider_name = provider_name
        self.object_path = object_path
        self.reason = reason
        super().__init__(
            f"query '{provider_name}' on {object_path!r} failed: {reason}"
        )


class ObjectNotFoundError(BackendError):
    """The object path does not name an object in the current scenario."""

    def __init__(self, object_path: str):
        self.object_path = object_path
        super().__init__(f"STK object not found: {object_path!r}")


# ---------------------------------------------------------------------------
# Connect-protocol-specific (raised only inside backends/connect_backend.py
# and backends/connect_protocol.py, but declared here so core/ can catch
# ConnectProtocolError without importing the connect backend module)
# ---------------------------------------------------------------------------

class ConnectProtocolError(BackendError):
    """Malformed or unparseable Connect response, ACK/NACK framing error,
    or socket-level timeout while a response was expected.
    """


class ConnectNackError(ConnectProtocolError):
    """STK returned an explicit NACK for a Connect command."""

    def __init__(self, command: str, nack_text: str):
        self.command = command
        self.nack_text = nack_text
        super().__init__(f"NACK for {command!r}: {nack_text}")


# ---------------------------------------------------------------------------
# Conversion / validation (core layer, but STK-adjacent enough to live here
# alongside the rest of the taxonomy rather than scattered in core/*)
# ---------------------------------------------------------------------------

class ConventionValidationError(StkBridgeError):
    """Raised by core/validation.py's startup self-test when a frame,
    quaternion-order, or unit assumption does not hold against the live
    backend. bridge_node.py treats it as fatal at startup rather than
    continuing with an unverified convention (see
    docs/bridge/design_decisions.md).
    """


class FrameResolutionError(StkBridgeError):
    """A frame_id string could not be resolved via config/frames.yaml."""

    def __init__(self, frame_id: str):
        self.frame_id = frame_id
        super().__init__(
            f"unknown frame_id {frame_id!r}; check config/frames.yaml"
        )


# ---------------------------------------------------------------------------
# Time synchronization
# ---------------------------------------------------------------------------

class LockstepCommandError(StkBridgeError):
    """A required lockstep command was absent or rejected before advance."""

    def __init__(self, step_index: int, reason: str):
        super().__init__(f"lockstep command failed at step {step_index}: {reason}")


class LockstepBarrierTimeoutError(StkBridgeError):
    """A lockstep step did not receive acknowledgement from all configured
    barrier participants within the configured timeout. Raised by
    core/time_sync.py when the ``time.on_barrier_timeout`` policy is
    ``abort``; the bridge node then stops the step loop.
    """

    def __init__(self, step_index: int, missing_participants: list):
        self.step_index = step_index
        self.missing_participants = list(missing_participants)
        super().__init__(
            f"lockstep barrier timeout at step {step_index}: "
            f"missing acks from {self.missing_participants}"
        )
