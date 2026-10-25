"""STK Connect wire-protocol encoding and parsing.

Pure functions and a stateful response reader, with no socket and no STK
dependency, so the whole protocol layer is unit-testable offline (see
``test/test_connect_protocol.py``).

Protocol summary
----------------
Connect is line-oriented ASCII over TCP (default port 5001). A client
sends a command terminated by newline; STK replies according to the
control flags currently in force (set via ``ConControl``):

* **Acknowledgement on** (the mode this bridge always uses): every command
  draws a literal ``ACK`` or ``NACK`` message. AGI's documentation is
  explicit that these are asymmetric with respect to data: *if a command
  that should return data instead returns NACK, the command failed and no
  data follows*. The reader therefore stops immediately on NACK instead of
  blocking for a data block that will never arrive.

* **Data-returning commands** reply with a header of the form
  ``<COMMANDNAME> <bytecount>`` followed by the payload, or for
  multi-message replies a count header followed by that many message
  lines, each itself prefixed ``<COMMANDNAME> <bytecount>``.

Header tolerance
----------------
The ACK/NACK semantics and the single- and multi-message shapes are
documented by AGI, but the field count of the multi-message *count
header* is not fully specified. :func:`parse_message_header` therefore
accepts either ``<NAME> <count> <bytes>`` or ``<NAME> <bytes>`` and lets
the reader disambiguate by what follows, so an unexpected header degrades
to "read what is there" instead of desynchronising the stream.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone
from typing import List, Optional, Tuple

from stk_bridge.backends.base import StkTime
from stk_bridge.exceptions import ConnectProtocolError

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

DEFAULT_CONNECT_PORT = 5001
LINE_TERMINATOR = "\n"
ENCODING = "ascii"

ACK_TOKEN = "ACK"
NACK_TOKEN = "NACK"

#: STK's UTCG month abbreviations. STK is case-tolerant on input but emits
#: this exact capitalisation, so parsing uses a case-insensitive lookup and
#: formatting uses this table.
_MONTHS = ("Jan", "Feb", "Mar", "Apr", "May", "Jun",
           "Jul", "Aug", "Sep", "Oct", "Nov", "Dec")
_MONTH_INDEX = {m.lower(): i + 1 for i, m in enumerate(_MONTHS)}

_UTCG_RE = re.compile(
    r"^\s*(\d{1,2})\s+([A-Za-z]{3})\s+(\d{4})\s+"
    r"(\d{1,2}):(\d{2}):(\d{2}(?:\.\d+)?)\s*$"
)

_HEADER_RE = re.compile(r"^([A-Za-z_][A-Za-z0-9_]*)\s+(\d+)(?:\s+(\d+))?\s*$")

_EPOCH = datetime(1970, 1, 1, tzinfo=timezone.utc)


# ---------------------------------------------------------------------------
# Time formatting
# ---------------------------------------------------------------------------

def format_utcg(when: StkTime, decimals: int = 3) -> str:
    """Render an :class:`StkTime` as an STK UTCG string.

    Example: ``"1 Jun 2024 12:00:00.000"``.

    STK accepts a leading zero on the day, but emits none; this function
    matches STK's own output so that round-tripping a time through
    ``format_utcg(parse_utcg(s)) == s`` holds for STK-produced strings.
    """
    if decimals < 0 or decimals > 6:
        raise ValueError("decimals must be in [0, 6]")

    dt = _EPOCH + timedelta(seconds=when.epoch_seconds)
    seconds = dt.second + dt.microsecond / 1e6
    sec_str = f"{seconds:0{3 + decimals if decimals else 2}.{decimals}f}"
    return (
        f"{dt.day} {_MONTHS[dt.month - 1]} {dt.year} "
        f"{dt.hour:02d}:{dt.minute:02d}:{sec_str}"
    )


def parse_utcg(text: str) -> StkTime:
    """Parse an STK UTCG string into an :class:`StkTime`.

    Raises:
        ConnectProtocolError: if ``text`` is not a UTCG timestamp. This is
            deliberately a protocol error rather than a ``ValueError``:
            in practice it means STK returned something other than the
            expected field (usually because the data provider element
            names were wrong), and the caller wants that classified as a
            backend problem.
    """
    match = _UTCG_RE.match(text)
    if not match:
        raise ConnectProtocolError(f"not a UTCG timestamp: {text!r}")

    day, mon, year, hour, minute, sec = match.groups()
    month = _MONTH_INDEX.get(mon.lower())
    if month is None:
        raise ConnectProtocolError(f"unknown month abbreviation: {mon!r}")

    whole_sec = int(float(sec))
    micro = int(round((float(sec) - whole_sec) * 1e6))
    # Guard the 59.9999995 -> 60 rounding case, which datetime rejects.
    if micro >= 1_000_000:
        micro -= 1_000_000
        whole_sec += 1

    try:
        dt = datetime(int(year), month, int(day), int(hour), int(minute),
                      0, 0, tzinfo=timezone.utc)
    except ValueError as exc:
        raise ConnectProtocolError(f"invalid UTCG date {text!r}: {exc}") from exc

    total = (dt - _EPOCH).total_seconds() + whole_sec + micro / 1e6
    return StkTime(total)


def quote(value: str) -> str:
    """Wrap a Connect argument in double quotes if it needs them.

    Connect uses whitespace as its argument separator, so any value
    containing spaces (file paths, UTCG timestamps, object names) must be
    quoted. Quoting unconditionally is *not* safe -- some commands treat a
    quoted numeric literal as a string -- so this quotes only when needed.
    """
    if not value:
        return '""'
    if any(ch.isspace() for ch in value) or '"' in value:
        escaped = value.replace('"', '\\"')
        return f'"{escaped}"'
    return value


# ---------------------------------------------------------------------------
# Command construction
# ---------------------------------------------------------------------------

def build_command(name: str, *args) -> str:
    """Assemble a Connect command line from a name and arguments.

    Floats are rendered with ``repr``-grade precision (17 significant
    digits) rather than ``str``: STK parses these as doubles, and
    truncating to ``str``'s 12 digits loses ~5 digits of position
    precision, which is metres of orbital error over a long propagation.
    """
    parts: List[str] = [name]
    for arg in args:
        if isinstance(arg, bool):
            parts.append("On" if arg else "Off")
        elif isinstance(arg, float):
            parts.append(f"{arg!r}")
        elif isinstance(arg, StkTime):
            parts.append(quote(format_utcg(arg)))
        elif isinstance(arg, (list, tuple)):
            parts.extend(
                f"{v!r}" if isinstance(v, float) else str(v) for v in arg
            )
        else:
            parts.append(str(arg))
    return " ".join(parts)


# ---------------------------------------------------------------------------
# Response parsing
# ---------------------------------------------------------------------------

@dataclass
class MessageHeader:
    """A parsed ``<COMMANDNAME> <bytecount>[ <bytecount2>]`` header."""

    name: str
    first_number: int
    second_number: Optional[int] = None

    @property
    def is_count_header(self) -> bool:
        """True if this looks like a multi-message count header.

        See the module docstring's confidence note: a header carrying two
        numbers is read as ``<count> <total_bytes>``; one number is read
        as ``<bytecount>`` for a single message.
        """
        return self.second_number is not None

    @property
    def byte_count(self) -> int:
        return (self.second_number if self.second_number is not None
                else self.first_number)

    @property
    def message_count(self) -> int:
        return self.first_number if self.is_count_header else 1


def parse_message_header(line: str) -> Optional[MessageHeader]:
    """Parse a Connect message header line, or return None if it isn't one.

    Returning ``None`` rather than raising is deliberate: a line that is
    not a header is usually payload, and the reader needs to treat it as
    such rather than aborting. Only genuinely malformed *framing* (a
    header we started to honour but whose payload never arrived) is an
    error, and that is detected in :class:`ConnectResponseReader`.
    """
    match = _HEADER_RE.match(line)
    if not match:
        return None
    name, first, second = match.groups()
    return MessageHeader(
        name=name,
        first_number=int(first),
        second_number=int(second) if second is not None else None,
    )


@dataclass
class ParsedResponse:
    ack: bool
    messages: List[str] = field(default_factory=list)
    raw: str = ""
    nack_text: str = ""


class ConnectResponseReader:
    """Incremental parser for the Connect reply stream.

    Fed raw decoded text via :meth:`feed`; reports completeness via
    :attr:`is_complete`. Incremental rather than read-until-timeout
    because the latter costs a full timeout interval on *every* command,
    which at 10 Hz is the difference between a working control loop and
    an unusable one. The reader knows a reply is done as soon as it has
    seen the ACK/NACK plus any promised payload.

    Usage::

        reader = ConnectResponseReader(expects_data=True)
        while not reader.is_complete:
            reader.feed(sock.recv(4096).decode("ascii"))
        result = reader.result()
    """

    def __init__(self, expects_data: bool = False,
                 acknowledgement_enabled: bool = True):
        self._expects_data = expects_data
        self._ack_enabled = acknowledgement_enabled
        self._buffer = ""
        self._raw = ""
        self._ack: Optional[bool] = None
        self._nack_text = ""
        self._messages: List[str] = []
        self._pending_messages = 0
        self._saw_header = False
        self._complete = False

        # With acknowledgement disabled and no data expected there is
        # nothing to wait for; mark complete immediately so callers don't
        # block on a reply STK will never send.
        if not acknowledgement_enabled and not expects_data:
            self._ack = True
            self._complete = True

    # -- feeding -----------------------------------------------------------

    def feed(self, chunk: str) -> None:
        if self._complete:
            return
        self._raw += chunk
        self._buffer += chunk
        while LINE_TERMINATOR in self._buffer and not self._complete:
            line, self._buffer = self._buffer.split(LINE_TERMINATOR, 1)
            self._consume_line(line.rstrip("\r"))

    def _consume_line(self, line: str) -> None:
        stripped = line.strip()
        if not stripped:
            return

        # 1. ACK / NACK
        if self._ack is None and self._ack_enabled:
            upper = stripped.upper()
            if upper == ACK_TOKEN or upper.startswith(ACK_TOKEN + " "):
                self._ack = True
                if not self._expects_data:
                    self._complete = True
                return
            if upper == NACK_TOKEN or upper.startswith(NACK_TOKEN + " "):
                # AGI: on NACK for a data-returning command, no data
                # follows. Terminate now instead of waiting for payload.
                self._ack = False
                self._nack_text = stripped[len(NACK_TOKEN):].strip()
                self._complete = True
                return
            # Some commands emit data before the acknowledgement when
            # verbose mode is on; fall through and treat as payload.

        # 2. A promised message payload
        if self._pending_messages > 0:
            header = parse_message_header(stripped)
            if header is not None and not header.is_count_header:
                self._pending_messages -= 1
                return
            self._messages.append(stripped)
            self._pending_messages -= 1
            if self._pending_messages == 0:
                self._complete = True
            return

        # 3. A header introducing payload
        header = parse_message_header(stripped)
        if header is not None and not self._saw_header:
            self._saw_header = True
            if header.is_count_header:
                self._pending_messages = header.message_count
            else:
                self._pending_messages = 1
            return

        # 4. Anything else is payload for a single-message reply.
        self._messages.append(stripped)
        if self._expects_data and self._ack is not None:
            self._complete = True

    # -- results -----------------------------------------------------------

    @property
    def is_complete(self) -> bool:
        return self._complete

    def force_complete(self) -> None:
        """Mark complete on socket timeout.

        Called by the transport when a read times out with a partial but
        plausible reply -- e.g. a command whose data-returning status we
        guessed wrong. Preserves whatever was received rather than
        discarding it, so the caller sees partial data plus the timeout
        in the log instead of an opaque empty result.
        """
        if self._buffer.strip():
            self._messages.append(self._buffer.strip())
            self._buffer = ""
        self._complete = True

    def result(self) -> ParsedResponse:
        if self._ack is None:
            # Acknowledgement disabled, or STK replied with data only.
            self._ack = True
        return ParsedResponse(
            ack=self._ack,
            messages=list(self._messages),
            raw=self._raw,
            nack_text=self._nack_text,
        )


# ---------------------------------------------------------------------------
# Report parsing
# ---------------------------------------------------------------------------

def parse_report_rm(messages: List[str]) -> Tuple[List[str], List[List[str]]]:
    """Parse the output of a ``Report_RM`` / ``ReportCreate`` style reply.

    Returns ``(column_names, rows)``. The first non-empty message is taken
    as the header row and subsequent messages as data rows, split on
    commas and stripped.

    This is the Connect backend's substitute for the COM backend's typed
    data-provider access, and it is strictly worse -- everything arrives
    as text with no type or unit metadata, and column names depend on the
    report style. That asymmetry is the main reason the COM backend is
    the recommended one; see docs/bridge/architecture.md section 1.1.
    """
    rows: List[List[str]] = []
    columns: List[str] = []
    for message in messages:
        if not message.strip():
            continue
        fields = [f.strip() for f in message.split(",")]
        if not columns:
            columns = fields
        else:
            rows.append(fields)
    return columns, rows


def looks_like_data_command(command: str) -> bool:
    """Heuristic: does this Connect command return data?

    Used to choose the reader's ``expects_data`` mode. A wrong guess is
    recoverable (the reader times out and calls
    :meth:`ConnectResponseReader.force_complete`) but costs a timeout
    interval, so the list covers the commands this bridge actually issues
    on its hot path plus the common query verbs.
    """
    verb = command.strip().split(" ", 1)[0].lower()
    return verb in {
        "report_rm", "report", "getreport", "getaccesses", "allaccess",
        "access_rm", "getattitude", "getposition", "getdirectory",
        "getstkversion", "getscenariotimes", "getanimationtime",
        "getlicense", "getunits", "getvalue", "showlicense",
    }
