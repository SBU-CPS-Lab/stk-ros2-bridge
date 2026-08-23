"""STK Connect backend: raw TCP socket to STK's Connect port.

The only backend in STK 12.6 that works across a machine or OS boundary.
STK 12.6 has no gRPC remoting (that arrives in 12.8 / Ansys 2024 R1), and
the Object Model is in-process COM, so if the bridge must run on Linux
while STK runs on Windows, this is the path.

Trade-offs versus :mod:`stk_bridge.backends.com_backend`:

* Everything is text. Data providers are reached through ``Report_RM``,
  which returns formatted strings with no type or unit metadata, so
  numeric parsing is the bridge's problem and column names depend on the
  report style installed in the scenario.
* Round-trip latency is higher and bounded below by TCP behaviour --
  see :meth:`ConnectBackend._configure_socket` for the Nagle interaction
  that otherwise adds up to 40 ms per command.
* Asynchronous Connect *is* available here and is not available over COM,
  so this backend advertises ``capabilities.async_connect = True``.

Prefer the COM backend when the bridge can be co-located with STK.
"""

from __future__ import annotations

import logging
import socket
import threading
from typing import List, Optional, Tuple

from stk_bridge.backends.base import (
    AccessIntervalData,
    BackendCapabilities,
    BackendInfo,
    BackendType,
    ConnectResponse,
    DataProviderData,
    DataProviderQuery,
    LinkBudgetData,
    ObjectInfo,
    PropagatorType,
    StkBackend,
    StkTime,
)
from stk_bridge.backends import connect_protocol as proto
from stk_bridge.exceptions import (
    BackendUnavailableError,
    CommandError,
    ConnectNackError,
    ConnectProtocolError,
    ConnectionLostError,
    ObjectNotFoundError,
    QueryError,
)

LOGGER = logging.getLogger(__name__)

_RECV_SIZE = 65536


class ConnectBackend(StkBackend):
    """STK access over the Connect TCP socket."""

    def __init__(self, host: str = "localhost",
                 port: int = proto.DEFAULT_CONNECT_PORT,
                 timeout_s: float = 10.0,
                 connect_timeout_s: float = 5.0):
        super().__init__(BackendType.CONNECT)
        self._host = host
        self._port = port
        self._timeout_s = timeout_s
        self._connect_timeout_s = connect_timeout_s
        self._sock: Optional[socket.socket] = None
        self._lock = threading.Lock()
        self._info = BackendInfo(
            backend_type=BackendType.CONNECT, host=host, port=port
        )

    # -- lifecycle ---------------------------------------------------------

    def connect(self) -> None:
        if self._connected:
            return
        try:
            self._sock = socket.create_connection(
                (self._host, self._port), timeout=self._connect_timeout_s
            )
        except OSError as exc:
            raise BackendUnavailableError(
                BackendType.CONNECT.value,
                f"cannot reach STK Connect at {self._host}:{self._port} "
                f"({exc}). Check STK is running, that Connect is enabled in "
                f"Edit > Preferences > Connect, and that the port matches.",
            ) from exc

        self._configure_socket()
        self._sock.settimeout(self._timeout_s)
        self._connected = True

        try:
            self._enable_acknowledgement()
            self._probe_version()
            self._probe_capabilities()
            self.configure_units()
        except Exception:
            self.disconnect()
            raise

        LOGGER.info("Connect backend attached to %s:%s (STK %s)",
                    self._host, self._port, self._info.stk_version)

    def _configure_socket(self) -> None:
        """Disable Nagle's algorithm.

        Connect is a small-message request/response protocol. With Nagle
        enabled on our side and delayed ACK on STK's, a small write
        followed by a blocking read stalls until the peer's delayed-ACK
        timer fires -- up to 40 ms on Windows, per command. At a 10 Hz
        control loop issuing several commands per tick that is the entire
        time budget. TCP_NODELAY costs a few extra packets and buys back
        the loop.
        """
        assert self._sock is not None
        try:
            self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:  # pragma: no cover - platform dependent
            LOGGER.warning("could not set TCP_NODELAY; expect added latency")

    def disconnect(self) -> None:
        self._connected = False
        if self._sock is not None:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def configure_units(self) -> None:
        """Force SI units for this Connect session.

        Unit preferences set over Connect apply to the STK application,
        not just this socket, so this does mutate shared state. That is
        accepted deliberately: an SI-consistent STK is what makes the
        bridge's outputs trustworthy, and the alternative (converting per
        query) reintroduces exactly the class of error this avoids. It is
        logged at INFO so it is never a surprise.
        """
        for dimension, unit in (
            ("Distance", "m"),
            ("Time", "sec"),
            ("Angle", "rad"),
            ("Mass", "kg"),
            ("DateFormat", "UTCG"),
        ):
            self._command(f"SetUnits / {dimension} {unit}", expect_data=False,
                          tolerate_nack=True)
        LOGGER.info("Connect session units set to SI (m, sec, rad, kg, UTCG)")

    def _enable_acknowledgement(self) -> None:
        """Turn on ACK/NACK so every command's outcome is observable.

        Without this, a failed command is indistinguishable from a
        successful one and errors surface much later as wrong numbers.
        Sent with acknowledgement assumed *off* for this one command,
        since we do not yet know the session's state.
        """
        assert self._sock is not None
        self._send_raw("ConControl / AckOn")
        reader = proto.ConnectResponseReader(
            expects_data=False, acknowledgement_enabled=False
        )
        self._drain(reader, tolerate_timeout=True)
        self._capabilities.async_connect = True

    def _probe_version(self) -> None:
        try:
            response = self._command("GetSTKVersion /", expect_data=True,
                                     tolerate_nack=True)
            if response.messages:
                self._info.stk_version = response.messages[0].strip()
        except ConnectProtocolError:
            LOGGER.debug("version probe failed; continuing", exc_info=True)

    def _probe_capabilities(self) -> None:
        """Determine licensed capabilities.

        Connect gives no clean licence-introspection command that is
        stable across versions, so capabilities are inferred: reaching
        this point at all proves Integration is licensed (Connect *is*
        Integration), and the rest are probed lazily by
        ``core/scenario_manager.py`` attempting the operation and
        catching the NACK. Recording ``unlicensed_detail`` here keeps the
        eventual error message actionable.
        """
        caps = BackendCapabilities(
            integration=True,
            async_connect=True,
            numpy_bulk_extraction=False,
            graphics=False,
        )
        caps.unlicensed_detail.update({
            "astrogator": "not probed over Connect; verified on first use",
            "realtime_propagator": "not probed over Connect; verified on first use",
        })
        # Optimistic defaults: the Connect backend cannot cheaply prove
        # these, and failing closed would block legitimate use. Real
        # verification happens at point of use.
        caps.astrogator = True
        caps.realtime_propagator = True
        caps.coverage = True
        caps.communications = True
        self._capabilities = caps

    # -- transport ---------------------------------------------------------

    def _send_raw(self, command: str) -> None:
        assert self._sock is not None
        payload = (command + proto.LINE_TERMINATOR).encode(proto.ENCODING)
        try:
            self._sock.sendall(payload)
        except OSError as exc:
            self._connected = False
            raise ConnectionLostError(
                BackendType.CONNECT.value, f"send failed: {exc}"
            ) from exc

    def _drain(self, reader: "proto.ConnectResponseReader",
               tolerate_timeout: bool = False) -> proto.ParsedResponse:
        assert self._sock is not None
        while not reader.is_complete:
            try:
                chunk = self._sock.recv(_RECV_SIZE)
            except socket.timeout:
                if tolerate_timeout:
                    reader.force_complete()
                    break
                raise ConnectProtocolError(
                    f"timed out after {self._timeout_s}s awaiting Connect reply"
                )
            except OSError as exc:
                self._connected = False
                raise ConnectionLostError(
                    BackendType.CONNECT.value, f"recv failed: {exc}"
                ) from exc
            if not chunk:
                self._connected = False
                raise ConnectionLostError(
                    BackendType.CONNECT.value, "STK closed the connection"
                )
            reader.feed(chunk.decode(proto.ENCODING, errors="replace"))
        return reader.result()

    def _command(self, command: str, expect_data: Optional[bool] = None,
                 timeout_s: Optional[float] = None,
                 tolerate_nack: bool = False) -> ConnectResponse:
        """Send one command and collect its reply.

        Serialized by a lock because a single Connect socket has no
        request/response correlation -- replies are matched to commands
        purely by order, so two concurrent senders would interleave and
        each read the other's reply. The bridge's own threading model
        (``core/com_thread.py``) already serializes, but this backend can
        legitimately be used standalone in tests and scripts, so the
        guarantee lives here too.
        """
        self.require_connected()
        if expect_data is None:
            expect_data = proto.looks_like_data_command(command)

        with self._lock:
            assert self._sock is not None
            previous = self._sock.gettimeout()
            if timeout_s is not None:
                self._sock.settimeout(timeout_s)
            try:
                self._send_raw(command)
                reader = proto.ConnectResponseReader(expects_data=expect_data)
                parsed = self._drain(reader, tolerate_timeout=expect_data)
            finally:
                if timeout_s is not None:
                    self._sock.settimeout(previous)

        if not parsed.ack and not tolerate_nack:
            raise ConnectNackError(command, parsed.nack_text or "(no detail)")

        return ConnectResponse(
            command=command,
            ack=parsed.ack,
            messages=parsed.messages,
            raw=parsed.raw,
        )

    # -- scenario ----------------------------------------------------------

    def new_scenario(self, name: str) -> None:
        self._command(f"New / Scenario {proto.quote(name)}", expect_data=False)

    def load_scenario(self, path: str) -> None:
        self._command(f"Load / Scenario {proto.quote(path)}", expect_data=False)

    def save_scenario(self, path: Optional[str] = None) -> None:
        if path:
            self._command(f"Save / Scenario {proto.quote(path)}",
                          expect_data=False)
        else:
            self._command("Save / */Scenario", expect_data=False)

    def close_scenario(self) -> None:
        self._command("Unload / *", expect_data=False, tolerate_nack=True)

    def get_scenario_epoch(self) -> StkTime:
        response = self._command("GetScenarioTimes /", expect_data=True)
        if not response.messages:
            raise QueryError("GetScenarioTimes", "/",
                             "no reply from STK")
        # Reply: "<start> <stop> <epoch>" with quoted UTCG fields.
        fields = _split_quoted(response.messages[0])
        if not fields:
            raise QueryError("GetScenarioTimes", "/",
                             f"unparseable reply: {response.messages[0]!r}")
        return proto.parse_utcg(fields[0])

    def list_objects(self) -> List[ObjectInfo]:
        response = self._command("AllInstanceNames /", expect_data=True,
                                 tolerate_nack=True)
        objects: List[ObjectInfo] = []
        for message in response.messages:
            for token in message.split():
                if "/" not in token:
                    continue
                parts = token.strip('"').split("/")
                obj_type = parts[-2] if len(parts) >= 2 else "Unknown"
                objects.append(ObjectInfo(object_path=token.strip('"'),
                                          object_type=obj_type))
        return objects

    def object_exists(self, object_path: str) -> bool:
        try:
            self._command(f"GetStkVersion {object_path}", expect_data=True,
                          tolerate_nack=True)
        except ConnectProtocolError:
            return False
        return any(o.object_path.endswith(object_path.lstrip("*/"))
                   for o in self.list_objects())

    def create_object(self, object_type: str, name: str,
                      parent_path: str = "") -> str:
        parent = parent_path or "*"
        self._command(f"New / {parent}/{object_type} {proto.quote(name)}",
                      expect_data=False)
        return f"{parent}/{object_type}/{name}"

    def delete_object(self, object_path: str) -> None:
        self._command(f"Unload / {object_path}", expect_data=False)

    # -- time --------------------------------------------------------------

    def get_current_time(self) -> StkTime:
        response = self._command("GetAnimationTime /", expect_data=True)
        if not response.messages:
            raise QueryError("GetAnimationTime", "/", "no reply")
        fields = _split_quoted(response.messages[0])
        return proto.parse_utcg(fields[0] if fields else response.messages[0])

    def set_current_time(self, when: StkTime) -> None:
        self._command(
            f"Animate * SetValues {proto.quote(proto.format_utcg(when))} 1 Reset",
            expect_data=False,
        )

    # -- sensing -----------------------------------------------------------

    def query_data_provider(self, query: DataProviderQuery) -> DataProviderData:
        """Fetch a data provider via ``Report_RM``.

        Text-only, as discussed in the module docstring. Numeric coercion
        is best-effort per cell: a value that will not parse as a float is
        kept as its string, because several useful providers mix numeric
        and categorical columns (Lighting Times' "Condition" being the
        one this bridge actually depends on).
        """
        self.require_connected()

        provider = query.provider_name
        if query.group_element:
            provider = f"{provider}/{query.group_element}"

        parts = [
            "Report_RM", query.object_path,
            "Style", proto.quote(provider),
        ]
        if query.start is not None and query.stop is not None:
            parts += [
                "TimePeriod",
                proto.quote(proto.format_utcg(query.start)),
                proto.quote(proto.format_utcg(query.stop)),
            ]
            if not query.is_instantaneous:
                parts += ["TimeStep", f"{query.step_s!r}"]

        command = " ".join(parts)
        try:
            response = self._command(command, expect_data=True)
        except ConnectNackError as exc:
            raise QueryError(query.provider_name, query.object_path,
                             exc.nack_text) from exc

        column_names, rows = proto.parse_report_rm(response.messages)
        if not column_names:
            raise QueryError(query.provider_name, query.object_path,
                             "report returned no columns; check the provider "
                             "name and that a matching report style exists")

        columns = {name: [] for name in column_names}
        for row in rows:
            for name, cell in zip(column_names, row):
                columns[name].append(_coerce(cell))

        return DataProviderData(
            object_path=query.object_path,
            provider_name=query.provider_name,
            columns=columns,
            row_count=len(rows),
        )

    def compute_access(self, from_path: str, to_path: str,
                       start: StkTime, stop: StkTime
                       ) -> List[AccessIntervalData]:
        command = (
            f"Access_RM {from_path} {to_path} "
            f"TimePeriod {proto.quote(proto.format_utcg(start))} "
            f"{proto.quote(proto.format_utcg(stop))}"
        )
        try:
            response = self._command(command, expect_data=True)
        except ConnectNackError as exc:
            raise QueryError("Access", from_path, exc.nack_text) from exc

        intervals: List[AccessIntervalData] = []
        for message in response.messages:
            fields = _split_quoted(message)
            times = [f for f in fields if _is_utcg(f)]
            if len(times) >= 2:
                intervals.append(AccessIntervalData(
                    start=proto.parse_utcg(times[0]),
                    stop=proto.parse_utcg(times[1]),
                ))
        return intervals

    def get_link_budget(self, tx_path: str, rx_path: str,
                        when: StkTime) -> LinkBudgetData:
        query = DataProviderQuery(
            object_path=rx_path,
            provider_name="Link Information",
            start=when, stop=when,
        )
        data = self.query_data_provider(query)
        return LinkBudgetData(
            carrier_to_noise_db=_safe_first(data, "C/N", "C/No"),
            eb_no_db=_safe_first(data, "Eb/No"),
            margin_db=_safe_first(data, "Link Margin", "Eb/No Margin"),
            received_power_dbw=_safe_first(data, "Rcvd. Iso. Power"),
            free_space_loss_db=_safe_first(data, "Free Space Loss"),
            link_closed=_safe_first(data, "Link Margin") > 0.0,
        )

    # -- actuation ---------------------------------------------------------

    def set_propagator(self, object_path: str, propagator: PropagatorType,
                       look_ahead_s: float = 3600.0,
                       look_behind_s: float = 3600.0,
                       external_file: str = "") -> None:
        if propagator is PropagatorType.REALTIME:
            self._command(
                f"SetPropagator {object_path} RealTime "
                f"LookAheadTime {look_ahead_s!r} "
                f"LookBehindTime {look_behind_s!r}",
                expect_data=False,
            )
        elif propagator is PropagatorType.EXTERNAL:
            if not external_file:
                raise CommandError("SetPropagator", object_path,
                                   "EXTERNAL propagator requires external_file")
            self.load_external_ephemeris(object_path, external_file)
        else:
            self._command(f"SetPropagator {object_path} {propagator.value}",
                          expect_data=False)

    def set_position(self, object_path: str, when: StkTime, frame: str,
                     position_m: Tuple[float, float, float],
                     velocity_mps: Optional[Tuple[float, float, float]] = None
                     ) -> None:
        args = [
            "SetPosition", object_path, proto.quote(proto.format_utcg(when)),
            frame,
            f"{position_m[0]!r}", f"{position_m[1]!r}", f"{position_m[2]!r}",
        ]
        if velocity_mps is not None:
            args += [f"{velocity_mps[0]!r}", f"{velocity_mps[1]!r}",
                     f"{velocity_mps[2]!r}"]
        try:
            self._command(" ".join(args), expect_data=False)
        except ConnectNackError as exc:
            raise CommandError("SetPosition", object_path,
                               exc.nack_text) from exc

    def enable_realtime_attitude(self, object_path: str) -> None:
        try:
            self._command(f"SetAttitude {object_path} Standard",
                          expect_data=False)
        except ConnectNackError as exc:
            raise CommandError("SetAttitude Standard", object_path,
                               exc.nack_text) from exc

    def clear_attitude_data(self, object_path: str) -> None:
        self._command(f"SetAttitude {object_path} ClearData",
                      expect_data=False, tolerate_nack=True)

    def add_attitude_quaternion(self, object_path: str, when: StkTime,
                                quaternion_xyzw: Tuple[float, float, float, float],
                                inertial: bool = False) -> None:
        # STK's Q1..Q4 == (x, y, z, w): vector part then scalar part, the
        # same order as this interface uses, so no reordering here.
        qx, qy, qz, qw = quaternion_xyzw
        keyword = "Quat" if inertial else "CBFQuat"
        command = (
            f"AddAttitude {object_path} {keyword} "
            f"{proto.quote(proto.format_utcg(when))} "
            f"{qx!r} {qy!r} {qz!r} {qw!r}"
        )
        try:
            self._command(command, expect_data=False)
        except ConnectNackError as exc:
            raise CommandError(f"AddAttitude {keyword}", object_path,
                               exc.nack_text) from exc

    def add_attitude_angular_velocity(
        self, object_path: str, when: StkTime,
        angular_velocity_radps: Tuple[float, float, float],
    ) -> None:
        # AddAttitude AngVels is documented as degrees/second and is NOT
        # affected by the session's Angle unit preference, so convert here
        # regardless of configure_units().
        import math

        wx, wy, wz = (math.degrees(v) for v in angular_velocity_radps)
        command = (
            f"AddAttitude {object_path} AngVels "
            f"{proto.quote(proto.format_utcg(when))} {wx!r} {wy!r} {wz!r}"
        )
        try:
            self._command(command, expect_data=False)
        except ConnectNackError as exc:
            raise CommandError("AddAttitude AngVels", object_path,
                               exc.nack_text) from exc

    def load_external_ephemeris(self, object_path: str,
                                file_path: str) -> None:
        try:
            self._command(
                f"SetState {object_path} FromFile {proto.quote(file_path)}",
                expect_data=False,
            )
        except ConnectNackError as exc:
            raise CommandError("SetState FromFile", object_path,
                               exc.nack_text) from exc

    # -- escape hatch ------------------------------------------------------

    def execute_connect(self, command: str,
                        timeout_s: Optional[float] = None) -> ConnectResponse:
        return self._command(command, timeout_s=timeout_s, tolerate_nack=True)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _split_quoted(text: str) -> List[str]:
    """Split a Connect reply on whitespace, honouring double quotes."""
    fields: List[str] = []
    current = []
    in_quotes = False
    for ch in text:
        if ch == '"':
            in_quotes = not in_quotes
            continue
        if ch.isspace() and not in_quotes:
            if current:
                fields.append("".join(current))
                current = []
            continue
        current.append(ch)
    if current:
        fields.append("".join(current))
    return fields


def _is_utcg(text: str) -> bool:
    try:
        proto.parse_utcg(text)
    except ConnectProtocolError:
        return False
    return True


def _coerce(cell: str):
    try:
        return float(cell)
    except (TypeError, ValueError):
        return cell


def _safe_first(data: DataProviderData, *names: str) -> float:
    """First numeric value from whichever of ``names`` exists.

    Link-budget column naming varies between STK versions and report
    styles, so callers pass several candidates rather than pinning one
    that may not exist in this install. Returns 0.0 if none match, which
    ``core/sensing.py`` reports as an unavailable field rather than
    treating as a real measurement.
    """
    for name in names:
        if name in data.columns and data.columns[name]:
            value = data.columns[name][0]
            if isinstance(value, float):
                return value
    return 0.0
