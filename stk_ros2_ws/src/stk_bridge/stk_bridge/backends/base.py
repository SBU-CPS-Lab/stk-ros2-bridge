"""Abstract backend interface and shared data types for stk_bridge.

This module defines the *only* contract between STK and the rest of the
bridge. Everything in ``core/``, ``interfaces/``, and ``nodes/`` is written
against ``StkBackend`` and the dataclasses declared here; none of them may
import ``agi.stk12``, ``win32com``, ``pythoncom``, or ``socket``-level
Connect machinery. See docs/00_architecture.md section 1.1.

Units contract
--------------
**Backends return SI.** Every backend calls :meth:`StkBackend.configure_units`
during ``connect()``, which forces STK's own UnitPreferences to metres,
metres/second, radians, and seconds. STK then reports SI natively and there
is no unit conversion left to get wrong downstream.

This supersedes the note in the Phase 1 ``.msg`` docstrings that said the
km->m conversion happens in ``core/conversions.py``. It does not, and
should not: converting inside the bridge means every new data provider is
a fresh opportunity to forget a factor of 1000, whereas configuring STK
once is a single point of control that also makes raw Connect output
(e.g. from ``ExecuteConnect``) consistent with typed output. Those msg
comments are corrected in Phase 6.

Two documented exceptions, both handled *inside* the backends and never
visible to callers:

* ``AddAttitude AngVels`` takes degrees/second regardless of unit
  preferences (fixed by the Connect command definition). Backends accept
  rad/s and convert on the way out.
* ``AddAttitude Euler``/``YPR`` take degrees. Not used by the live control
  path (which is quaternion-based), but noted for completeness.

Time contract
-------------
All times crossing this interface are :class:`StkTime` -- a thin wrapper
around a UTC Unix epoch float. Backends are responsible for formatting to
STK's UTCG strings ("1 Jun 2024 12:00:00.000") and parsing them back.
ROS ``builtin_interfaces/Time`` conversion is Phase 3's job
(``core/conversions.py``), deliberately kept out of the backend layer so
backends stay testable without a ROS installation.

Quaternion contract
-------------------
All quaternions crossing this interface are ``(x, y, z, w)`` -- vector
part first, scalar last -- matching both ``geometry_msgs/Quaternion`` and
STK's documented Q1..Q4 ordering. Backends are responsible for reordering
if a specific API they wrap disagrees. ``core/validation.py`` (Phase 3)
verifies this against the live backend at startup and raises
``ConventionValidationError`` rather than letting an unverified assumption
propagate into control output.
"""

from __future__ import annotations

import abc
import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional, Sequence, Tuple

from stk_bridge.exceptions import CommandError


# ---------------------------------------------------------------------------
# Enumerations
# ---------------------------------------------------------------------------

class BackendType(str, Enum):
    """Identifies a concrete backend implementation.

    String-valued so it serializes directly into
    ``BridgeStatus.backend_type`` without a lookup table.
    """

    COM = "com"
    ENGINE = "engine"
    CONNECT = "connect"
    MOCK = "mock"


class PropagatorType(str, Enum):
    """Satellite propagator selection.

    Mirrors ``stk_msgs/srv/SetPropagator.srv``'s constants. REALTIME is the
    one that matters for the live control loop: it is what makes
    ``SetPosition`` meaningful. EXTERNAL is the batch-replay path.
    """

    TWO_BODY = "TwoBody"
    J4_PERTURBATION = "J4Perturbation"
    HPOP = "HPOP"
    REALTIME = "RealTime"
    EXTERNAL = "StkExternal"


class EclipseCondition(str, Enum):
    UMBRA = "Umbra"
    PENUMBRA = "Penumbra"
    SUNLIT = "Direct Sun"


# ---------------------------------------------------------------------------
# Time
# ---------------------------------------------------------------------------

@dataclass(frozen=True, order=True)
class StkTime:
    """A UTC instant, as Unix epoch seconds.

    Chosen over ``datetime`` because the bridge's hot path compares and
    differences timestamps constantly (monotonicity guards, rate groups,
    lockstep barriers) and a float is both cheaper and free of timezone
    ambiguity. Formatting to STK's UTCG representation happens in the
    backends; see ``connect_protocol.format_utcg`` / ``parse_utcg``.

    Sub-millisecond precision is preserved: STK's UTCG format carries
    three decimal places by default, and backends request more where the
    underlying API allows it.
    """

    epoch_seconds: float

    def __add__(self, seconds: float) -> "StkTime":
        return StkTime(self.epoch_seconds + seconds)

    def __sub__(self, other: "StkTime | float") -> "float | StkTime":
        if isinstance(other, StkTime):
            return self.epoch_seconds - other.epoch_seconds
        return StkTime(self.epoch_seconds - other)


# ---------------------------------------------------------------------------
# Backend description
# ---------------------------------------------------------------------------

@dataclass
class BackendCapabilities:
    """What a connected backend can actually do.

    Populated during ``connect()`` by probing the live STK instance rather
    than being hardcoded per backend class, because two installs of the
    same STK version can differ by license. ``core/scenario_manager.py``
    consults this before attempting anything optional, so that an
    unlicensed capability produces a clear ``LicenseError`` at startup
    instead of an opaque NACK twenty minutes into a run.
    """

    integration: bool = False
    astrogator: bool = False
    realtime_propagator: bool = False
    coverage: bool = False
    communications: bool = False

    graphics: bool = False
    async_connect: bool = False
    numpy_bulk_extraction: bool = False

    unlicensed_detail: Dict[str, str] = field(default_factory=dict)

    def require(self, capability: str) -> None:
        """Raise :class:`LicenseError` unless ``capability`` is available."""
        from stk_bridge.exceptions import LicenseError

        if not getattr(self, capability, False):
            raise LicenseError(
                capability,
                self.unlicensed_detail.get(capability),
            )


@dataclass
class BackendInfo:
    backend_type: BackendType
    stk_version: str = "unknown"
    python_version: str = "unknown"
    agi_stk_wheel_version: str = "unknown"
    host: str = "localhost"
    port: Optional[int] = None


# ---------------------------------------------------------------------------
# Sensed data
# ---------------------------------------------------------------------------

@dataclass
class StateSample:
    """Cartesian position/velocity at one instant. SI units, always."""

    time: StkTime
    frame: str
    position_m: Tuple[float, float, float]
    velocity_mps: Tuple[float, float, float]
    is_extrapolated: bool = False


@dataclass
class AttitudeSample:
    """Body attitude at one instant. Quaternion is (x, y, z, w)."""

    time: StkTime
    frame: str
    quaternion_xyzw: Tuple[float, float, float, float]
    is_extrapolated: bool = False


@dataclass
class AngularRateSample:
    """Body angular velocity, rad/s."""

    time: StkTime
    angular_velocity_radps: Tuple[float, float, float]


@dataclass
class LLASample:
    time: StkTime
    latitude_rad: float
    longitude_rad: float
    altitude_m: float


@dataclass
class EclipseSample:
    time: StkTime
    condition: EclipseCondition


@dataclass
class AccessIntervalData:
    start: StkTime
    stop: StkTime

    @property
    def duration_s(self) -> float:
        return self.stop.epoch_seconds - self.start.epoch_seconds


@dataclass
class LinkBudgetData:
    carrier_to_noise_db: float = 0.0
    eb_no_db: float = 0.0
    margin_db: float = 0.0
    received_power_dbw: float = 0.0
    free_space_loss_db: float = 0.0
    link_closed: bool = False


@dataclass
class ObjectInfo:
    object_path: str
    object_type: str
    propagator_type: str = ""


# ---------------------------------------------------------------------------
# Generic data provider I/O
# ---------------------------------------------------------------------------

@dataclass
class DataProviderQuery:
    """A request for one STK data provider over a time span.

    ``group_element`` selects the sub-provider for grouped providers --
    e.g. provider "Cartesian Position" is a *group* whose members are
    reference frames ("ICRF", "Fixed", "J2000"). Getting this wrong is one
    of the most common STK scripting errors, so the two are separate
    fields here rather than a single dotted string that callers assemble
    (and mis-assemble) themselves.
    """

    object_path: str
    provider_name: str
    group_element: Optional[str] = None
    elements: Sequence[str] = field(default_factory=tuple)
    start: Optional[StkTime] = None
    stop: Optional[StkTime] = None
    step_s: float = 1.0

    @property
    def is_instantaneous(self) -> bool:
        """True if this is a single-epoch query (start == stop or no span)."""
        if self.start is None or self.stop is None:
            return True
        return self.stop.epoch_seconds <= self.start.epoch_seconds


@dataclass
class DataProviderData:
    """Column-oriented data provider result.

    ``columns`` maps element name -> list of values, all lists the same
    length. Column orientation matches STK's own
    ``IAgDrDataSetCollection`` shape and ``ToNumpyArray()`` output, so the
    COM backend can hand data across with minimal reshaping.

    Values are ``float`` for numeric elements and ``str`` for time or
    categorical elements (e.g. the "Condition" element of Lighting Times);
    callers should not assume numeric.
    """

    object_path: str
    provider_name: str
    columns: Dict[str, List]
    row_count: int = 0

    def column(self, name: str) -> List:
        from stk_bridge.exceptions import QueryError

        if name not in self.columns:
            raise QueryError(
                self.provider_name,
                self.object_path,
                f"element {name!r} not in result; got {list(self.columns)}",
            )
        return self.columns[name]

    def first(self, name: str):
        col = self.column(name)
        if not col:
            from stk_bridge.exceptions import QueryError

            raise QueryError(
                self.provider_name, self.object_path,
                f"element {name!r} returned no rows",
            )
        return col[0]


@dataclass
class ConnectResponse:
    """Result of a raw Connect command."""

    command: str
    ack: bool
    messages: List[str] = field(default_factory=list)
    raw: str = ""

    @property
    def single(self) -> str:
        return self.messages[0] if self.messages else ""


# ---------------------------------------------------------------------------
# The interface
# ---------------------------------------------------------------------------

class StkBackend(abc.ABC):
    """Abstract STK access layer.

    Implementations must be usable as context managers and must be safe to
    call from a single thread only. Thread confinement is the caller's
    responsibility -- ``core/com_thread.py`` (Phase 3) owns the STA thread
    that serializes access for the COM/Engine backends. Backends do not
    take locks internally, because doing so would silently mask the
    apartment-threading violations we specifically want to surface during
    development.
    """

    def __init__(self, backend_type: BackendType):
        self._backend_type = backend_type
        self._connected = False
        self._capabilities = BackendCapabilities()
        self._info = BackendInfo(backend_type=backend_type)

    # -- lifecycle ---------------------------------------------------------

    @property
    def backend_type(self) -> BackendType:
        return self._backend_type

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def capabilities(self) -> BackendCapabilities:
        return self._capabilities

    @property
    def info(self) -> BackendInfo:
        return self._info

    @abc.abstractmethod
    def connect(self) -> None:
        """Establish the connection and populate info/capabilities.

        Raises:
            BackendUnavailableError: could not connect at all.
            LicenseError: connected, but the Integration capability that
                every backend requires is absent.
        """

    @abc.abstractmethod
    def disconnect(self) -> None:
        """Tear down cleanly. Must be idempotent and must not raise."""

    @abc.abstractmethod
    def configure_units(self) -> None:
        """Force STK's unit preferences to SI. Called by ``connect()``.

        See this module's docstring for why this lives here rather than in
        ``core/conversions.py``.
        """

    def __enter__(self) -> "StkBackend":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.disconnect()

    # -- scenario ----------------------------------------------------------

    def get_current_scenario_name(self) -> Optional[str]:
        """Return the open scenario name, or None, without changing it."""
        from stk_bridge.exceptions import QueryError
        raise QueryError("CurrentScenario", "/",
                         "this backend does not support reusing an open scenario")

    @abc.abstractmethod
    def new_scenario(self, name: str) -> None: ...

    @abc.abstractmethod
    def load_scenario(self, path: str) -> None: ...

    @abc.abstractmethod
    def save_scenario(self, path: Optional[str] = None) -> None: ...

    @abc.abstractmethod
    def close_scenario(self) -> None: ...

    @abc.abstractmethod
    def get_scenario_epoch(self) -> StkTime: ...

    @property
    def revision(self) -> Optional[int]:
        """A counter that changes on every structural STK change.

        Scenario and object lifecycle, propagator changes, real-time
        attitude enable/ClearData and arbitrary Connect commands change
        it; attitude sample writes do not. Anything a caller proved or
        cached about the scenario is valid only while it is unchanged.
        None means this backend does not track it, and callers must not
        rely on anything they proved earlier.
        """
        return None

    def get_propagator_type(self, object_path: str) -> Optional[str]:
        """The object's propagator type name, or None if not known.

        On COM this is the AgEVePropagatorType enum name, e.g.
        'ePropagatorHPOP'. Orbit batching runs only for a confirmed HPOP
        or TwoBody propagator, so None means "do not batch".
        """
        return None

    def get_scenario_stop_time(self) -> Optional[StkTime]:
        """The open scenario's analysis stop time, or None if unknown.

        Not abstract: only the COM backend (and Engine, which inherits
        it) reads it. A caller that needs it -- the orbit-block reader,
        which must never request an epoch at or after it -- treats None
        as "do not batch".
        """
        return None

    def graphics_state(self) -> Optional[Dict[str, Any]]:
        """STK Desktop's graphics windows and main window, or None.

        Not abstract: only STK Desktop has windows (see
        core/graphics_state.py for the format and why it is recorded).
        """
        return None

    def set_graphics_off(self) -> None:
        """Close STK Desktop's 2D/3D Graphics windows and minimise it.

        The write-side counterpart of :meth:`graphics_state`, called only
        by the scenario builder; a no-op where there are no windows.
        """

    @abc.abstractmethod
    def list_objects(self) -> List[ObjectInfo]: ...

    @abc.abstractmethod
    def object_exists(self, object_path: str) -> bool: ...

    @abc.abstractmethod
    def create_object(self, object_type: str, name: str,
                      parent_path: str = "") -> str:
        """Create an object and return its full STK path."""

    @abc.abstractmethod
    def delete_object(self, object_path: str) -> None: ...

    # -- time --------------------------------------------------------------

    @abc.abstractmethod
    def get_current_time(self) -> StkTime:
        """Current animation/scenario time."""

    @abc.abstractmethod
    def set_current_time(self, when: StkTime) -> None: ...

    # -- sensing -----------------------------------------------------------

    @abc.abstractmethod
    def query_data_provider(self, query: DataProviderQuery) -> DataProviderData:
        """Execute one data provider query.

        This is the single primitive on which all typed sensing in
        ``core/sensing.py`` is built. Backends implement it once; they do
        not implement per-quantity getters. That keeps the surface small
        enough that the Connect and COM implementations can be held to the
        same observable behaviour, which ``test_mock_backend.py`` and the
        Phase 6 conformance tests rely on.
        """

    def list_data_providers(self, object_path: str) -> Dict[str, List[str]]:
        """Enumerate the data providers available on an object.

        Returns ``{provider_name: [group_member, ...]}`` with an empty
        list for ungrouped providers.

        Guessing provider and element names across STK versions does not
        scale -- every wrong guess costs a round trip to a live install
        to discover. Enumeration turns that into a single query whose
        answer is authoritative for *this* install, which is what
        ``core/validation.py`` uses to report real options when a spec
        fails to match, and what ``data_providers.discover()`` uses to
        resolve a provider by keyword rather than exact spelling.

        Default implementation returns ``{}`` (unsupported); backends
        that can enumerate override it.
        """
        return {}

    @abc.abstractmethod
    def compute_access(self, from_path: str, to_path: str,
                       start: StkTime, stop: StkTime
                       ) -> List[AccessIntervalData]: ...

    @abc.abstractmethod
    def get_link_budget(self, tx_path: str, rx_path: str,
                        when: StkTime) -> LinkBudgetData: ...

    # -- actuation ---------------------------------------------------------

    @abc.abstractmethod
    def set_propagator(self, object_path: str, propagator: PropagatorType,
                       look_ahead_s: float = 3600.0,
                       look_behind_s: float = 3600.0,
                       external_file: str = "") -> None: ...

    @abc.abstractmethod
    def set_position(self, object_path: str, when: StkTime, frame: str,
                     position_m: Tuple[float, float, float],
                     velocity_mps: Optional[Tuple[float, float, float]] = None
                     ) -> None:
        """Append one real-time position sample (``SetPosition``).

        Append-only: STK silently ignores samples whose time is not
        strictly greater than the last accepted sample for this object.
        Backends do NOT guard against this -- ``core/actuation.py`` owns
        the monotonicity check so that the rejection is counted and
        logged rather than lost. See ``MonotonicityViolationError``.
        """

    @abc.abstractmethod
    def enable_realtime_attitude(self, object_path: str) -> None:
        """Switch an object's attitude profile to Real-Time.

        Issues ``SetAttitude <path> Standard`` (or the Object Model
        equivalent). Idempotent: STK treats a redundant call as a no-op.
        Must be called before :meth:`add_attitude_quaternion` for a given
        object; ``core/actuation.py`` does this lazily on first command.
        """

    # -- sensors -----------------------------------------------------------
    #
    # Concrete, not abstract: every backend can at least issue the
    # Connect form, and a backend with the Object Model available
    # overrides with the typed path. Sensor geometry lives here rather
    # than in a caller because reading a cone angle back requires the
    # Object Model, which only backends/ may touch.

    def set_sensor_cone(self, object_path: str, half_angle_rad: float,
                        angular_resolution_rad: float = 0.017453) -> str:
        """Make a sensor a simple cone of the given half-angle.

        Returns a label describing how it was applied, for the caller's
        report. Raises :class:`CommandError` if nothing worked, with
        every attempt's reason in the message -- a bare failure here
        leaves the sensor at STK's default half-angle, which is wide
        enough that pointing error stops mattering.
        """
        reasons = []
        for label, command in (
            ("Connect Define SimpleConic (degrees)",
             f"Define {object_path} SimpleConic "
             f"{math.degrees(half_angle_rad)!r} "
             f"{math.degrees(angular_resolution_rad)!r}"),
            ("Connect Define SimpleConic (radians)",
             f"Define {object_path} SimpleConic {half_angle_rad!r} "
             f"{angular_resolution_rad!r}"),
        ):
            try:
                if self.execute_connect(command).ack:
                    return label
                reasons.append(f"{label}: NACK")
            except Exception as exc:                      # noqa: BLE001
                reasons.append(f"{label}: {type(exc).__name__}: {exc}")
        raise CommandError("Define SimpleConic", object_path,
                           "; ".join(reasons))

    def set_az_el_mask(self, object_path: str, mode: str,
                       value: float = 0.0) -> str:
        """Apply an azimuth/elevation mask to a static object.

        ``mode`` is ``"terrain"``, ``"none"`` or ``"file"``. Those are
        the only kinds STK has: a live 12.6 install reports
        ``AgEAzElMaskType`` members ``eAzElMaskNone``, ``eMaskFile`` and
        ``eTerrainData``, and nothing else.

        In particular there is **no constant-elevation mask**. A minimum
        elevation is an access *constraint*, a separate mechanism
        reached through ``SetConstraint``, and conflating the two costs
        a live run: the verifier spent one trying to apply a constant
        mask as an experimental control, failing, and then having
        nothing to compare terrain against.

        Returns a label describing how it was applied; raises
        :class:`CommandError` with every attempt's reason if nothing
        worked.
        """
        spelling = {"terrain": "TerrainData", "none": "None",
                    "file": "File"}.get(mode)
        if spelling is None:
            raise CommandError("SetAzElMask", object_path,
                               f"unknown mask mode {mode!r}; STK has "
                               f"terrain, none and file")

        reasons = []
        for label, command in (
            (f"Connect SetAzElMask {spelling}",
             f"SetAzElMask {object_path} {spelling}"),
            (f"Connect SetAzElMask {spelling} (with range)",
             f"SetAzElMask {object_path} {spelling} 1 {value!r}"),
        ):
            try:
                if self.execute_connect(command).ack:
                    return label
                reasons.append(f"{label}: NACK")
            except Exception as exc:                      # noqa: BLE001
                reasons.append(f"{label}: {type(exc).__name__}: {exc}")
        raise CommandError("SetAzElMask", object_path, "; ".join(reasons))

    def get_az_el_mask(self, object_path: str) -> Optional[str]:
        """The mask type STK currently holds, or None if unreadable.

        Only backends with the Object Model can answer. Worth having:
        it separates "the command did not apply" from "the mask applied
        and made no difference", which are different problems with the
        same symptom.
        """
        return None

    def get_sensor_cone_half_angle_rad(self, object_path: str
                                       ) -> Optional[float]:
        """The sensor's configured cone half-angle, or None if unreadable.

        Only backends with the Object Model can answer this. It matters
        because it separates "the command did not apply" from "the
        command applied in the wrong unit" without needing to infer the
        cone from access geometry.
        """
        return None

    @abc.abstractmethod
    def clear_attitude_data(self, object_path: str) -> None:
        """``SetAttitude <path> ClearData`` -- drop accumulated real-time
        attitude samples. Used by the ResetScenario service."""

    @abc.abstractmethod
    def add_attitude_quaternion(self, object_path: str, when: StkTime,
                                quaternion_xyzw: Tuple[float, float, float, float],
                                inertial: bool = False) -> None:
        """Append one real-time attitude sample.

        Maps to ``AddAttitude <path> CBFQuat`` when ``inertial`` is False
        (central-body-fixed reference axes) or ``AddAttitude <path> Quat``
        when True. Same append-only/monotonicity semantics as
        :meth:`set_position`.

        The quaternion is ``(x, y, z, w)`` on this interface; backends
        emit STK's ``<Q1> <Q2> <Q3> <Q4>`` in that same order, Q4 being
        the scalar part.
        """

    @abc.abstractmethod
    def add_attitude_angular_velocity(
        self, object_path: str, when: StkTime,
        angular_velocity_radps: Tuple[float, float, float],
    ) -> None:
        """Append one real-time body angular-velocity sample.

        Maps to ``AddAttitude <path> AngVels``, which requires at least
        one prior attitude data point or it NACKs -- ``core/actuation.py``
        enforces the ordering. Takes rad/s here; backends convert to the
        degrees/second the Connect command requires.
        """

    @abc.abstractmethod
    def load_external_ephemeris(self, object_path: str,
                                file_path: str) -> None: ...

    # -- escape hatch ------------------------------------------------------

    @abc.abstractmethod
    def execute_connect(self, command: str,
                        timeout_s: Optional[float] = None) -> ConnectResponse:
        """Send a raw Connect command.

        Every backend supports this, including COM/Engine (via
        ``IAgStkObjectRoot.ExecuteCommand``) -- with one documented
        exception: asynchronous Connect commands are not supported over
        COM, only over the TCP socket. The Connect backend advertises
        ``capabilities.async_connect = True``; COM/Engine do not.
        """

    # -- convenience (concrete, shared) ------------------------------------

    def require_connected(self) -> None:
        from stk_bridge.exceptions import BackendUnavailableError

        if not self._connected:
            raise BackendUnavailableError(
                self._backend_type.value, "not connected"
            )

    def __repr__(self) -> str:
        state = "connected" if self._connected else "disconnected"
        return f"<{type(self).__name__} {self._backend_type.value} {state}>"
