"""Typed access to STK Data Providers.

Wraps the backend's single ``query_data_provider`` primitive with named
specifications for the quantities the ADCS twin needs, so call sites ask
for "the attitude quaternion of SSS-1" rather than assembling provider
names, group members, and element strings inline.

Two problems this module exists to solve
----------------------------------------

**1. Element names are not stable.** STK's element naming varies by
version, object type, and installed report styles: the same quantity may
be ``"x"``, ``"X"``, or ``"Cartesian X"``. Every spec therefore declares
*candidate* names per field and :func:`resolve_columns` picks whichever
the live install actually returned.

**2. Empty results are silent.** A data provider queried on an object
with no ephemeris returns zero rows and raises nothing (on STK 12.6 a
freshly created, unpropagated satellite returns no columns at all).
:func:`execute` therefore treats an empty result as an error by default,
with a message that names the likely cause, an unpropagated object.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from stk_bridge.backends.base import (
    DataProviderData,
    DataProviderQuery,
    StkBackend,
    StkTime,
)
from stk_bridge.core import conversions as conv
from stk_bridge.exceptions import QueryError

LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True)
class FieldSpec:
    """One value within a data provider result."""

    #: Canonical name used by the rest of the bridge.
    name: str
    #: Candidate STK element names, tried in order.
    candidates: Tuple[str, ...]
    #: True if a missing column should fail the whole query rather than
    #: yield None. Position components are required; a link margin is not.
    required: bool = True
    #: Multiply STK's value by this to reach SI. Almost always 1.0,
    #: because backends force SI unit preferences -- the exception is
    #: angular rate, which STK 12.6 refuses to report in rad/s. Those
    #: specs use ``scale=None`` to mean "ask the measured
    #: AngleRateScale", resolved at read time.
    scale: Optional[float] = 1.0


@dataclass(frozen=True)
class ProviderSpec:
    """A named data provider query template."""

    key: str
    provider_name: str
    fields: Tuple[FieldSpec, ...]
    #: Group member (e.g. a reference frame). None means ungrouped;
    #: "<frame>" is substituted with the caller's requested frame.
    group_element: Optional[str] = None
    #: Alternative provider names to try if the primary is absent.
    provider_alternatives: Tuple[str, ...] = ()
    description: str = ""

    def element_names(self) -> List[str]:
        """Flat list of every candidate element, for ``ExecElements``.

        Passing candidates STK does not recognise makes ``ExecElements``
        fail outright, so this is used only when the caller has already
        confirmed which names exist; the default path requests all
        elements and filters afterwards.
        """
        names: List[str] = []
        for spec in self.fields:
            names.extend(spec.candidates)
        return names


# ---------------------------------------------------------------------------
# Specifications for the quantities the ADCS twin needs
# ---------------------------------------------------------------------------

TIME_FIELD = FieldSpec(
    name="time",
    candidates=("Time", "Epoch", "Time (EpSec)", "EpSec"),
    required=False,
)

CARTESIAN_POSITION = ProviderSpec(
    key="cartesian_position",
    provider_name="Cartesian Position",
    group_element="<frame>",
    fields=(
        TIME_FIELD,
        FieldSpec("x", ("x", "X", "Cartesian X")),
        FieldSpec("y", ("y", "Y", "Cartesian Y")),
        FieldSpec("z", ("z", "Z", "Cartesian Z")),
    ),
    description="Position in the requested reference frame, metres.",
)

CARTESIAN_VELOCITY = ProviderSpec(
    key="cartesian_velocity",
    provider_name="Cartesian Velocity",
    group_element="<frame>",
    fields=(
        TIME_FIELD,
        FieldSpec("vx", ("x", "X", "Velocity x")),
        FieldSpec("vy", ("y", "Y", "Velocity y")),
        FieldSpec("vz", ("z", "Z", "Velocity z")),
    ),
    description="Velocity in the requested reference frame, m/s.",
)

ATTITUDE_QUATERNION = ProviderSpec(
    key="attitude_quaternion",
    provider_name="Attitude Quaternions",
    provider_alternatives=("Attitude Quaternion",),
    # Ungrouped on STK 12.6: the provider takes no reference-frame
    # member and always reports attitude relative to the central body's
    # inertial axes. A frame argument is silently ignored, so comparing
    # it with an Earth-fixed command is wrong by the Earth-rotation angle.
    group_element=None,
    fields=(
        TIME_FIELD,
        FieldSpec("q1", ("q1", "Q1", "qx", "x")),
        FieldSpec("q2", ("q2", "Q2", "qy", "y")),
        FieldSpec("q3", ("q3", "Q3", "qz", "z")),
        FieldSpec("q4", ("q4", "Q4", "qw", "w", "qs")),
    ),
    description=(
        "Body attitude relative to the requested frame. Q1..Q3 vector "
        "part, Q4 scalar -- mapped to ROS (x,y,z,w) by "
        "conversions.quaternion_from_stk_elements."
    ),
)

ECF_ATTITUDE_QUATERNION = ProviderSpec(
    key="ecf_attitude_quaternion",
    provider_name="ECF Attitude Quaternions",
    provider_alternatives=("Attitude Segment ECF Quaternions",),
    group_element=None,
    fields=(
        TIME_FIELD,
        FieldSpec("q1", ("q1", "Q1")),
        FieldSpec("q2", ("q2", "Q2")),
        FieldSpec("q3", ("q3", "Q3")),
        FieldSpec("q4", ("q4", "Q4")),
    ),
    description=(
        "Body attitude relative to central-body-FIXED axes. This is the "
        "correct read-back counterpart for `AddAttitude CBFQuat`; "
        "ATTITUDE_QUATERNION is the counterpart for `AddAttitude Quat`. "
        "Pairing them the wrong way round differs by the Earth-rotation "
        "angle at the sample epoch -- tens of degrees, and entirely "
        "plausible-looking."
    ),
)

#: STK's "Attitude Segment Quaternions" provider carries the quaternion
#: AND the body angular velocity in a single result, documented as "the
#: attitude quaternion and angular velocity of the vehicle's body axes
#: computed with respect to the vehicle's central body inertial
#: coordinate system", with q1..q3 the vector part and q4 the scalar.
#: One query for both quantities is one round trip instead of two.
ATTITUDE_SEGMENT = ProviderSpec(
    key="attitude_segment",
    provider_name="Attitude Segment Quaternions",
    fields=(
        TIME_FIELD,
        FieldSpec("q1", ("q1", "Q1")),
        FieldSpec("q2", ("q2", "Q2")),
        FieldSpec("q3", ("q3", "Q3")),
        FieldSpec("q4", ("q4", "Q4")),
        FieldSpec("wx", ("wx", "x", "Angular Velocity x"),
                  required=False, scale=None),
        FieldSpec("wy", ("wy", "y", "Angular Velocity y"),
                  required=False, scale=None),
        FieldSpec("wz", ("wz", "z", "Angular Velocity z"),
                  required=False, scale=None),
    ),
    description=(
        "Quaternion + body angular velocity w.r.t. central body inertial. "
        "Ungrouped, so it works where the grouped attitude providers "
        "require a frame the object does not define."
    ),
)

ANGULAR_VELOCITY = ProviderSpec(
    key="angular_velocity",
    provider_name="Body Axes Angular Velocity",
    provider_alternatives=(
        "Angular Velocity", "Body Axes Angular Rate",
        "Attitude Segment Quaternions",
    ),
    group_element="<frame>",
    fields=(
        TIME_FIELD,
        # scale=None: resolved from the measured AngleRateScale, because
        # STK 12.6 will not accept rad/s as the AngleRate unit preference.
        FieldSpec("wx", ("x", "X", "wx"), scale=None),
        FieldSpec("wy", ("y", "Y", "wy"), scale=None),
        FieldSpec("wz", ("z", "Z", "wz"), scale=None),
    ),
    description="Body angular velocity. Units measured, not assumed.",
)

LLA_STATE = ProviderSpec(
    key="lla_state",
    provider_name="LLA State",
    group_element="<frame>",
    fields=(
        TIME_FIELD,
        FieldSpec("latitude", ("Lat", "Latitude")),
        FieldSpec("longitude", ("Lon", "Longitude")),
        FieldSpec("altitude", ("Alt", "Altitude")),
    ),
    description="Geodetic position. Angles in radians once units are SI.",
)

#: On STK 12.6 "Lighting Times" is grouped, with one member per
#: condition ("Sunlight", "Penumbra", "Umbra" and Subpoint variants),
#: each returning start/stop intervals rather than a "Condition" column.
#: core/sensing.py queries the members and tests interval membership.
#: The caller supplies the group_element, since the members are
#: conditions, not frames.
LIGHTING = ProviderSpec(
    key="lighting",
    provider_name="Lighting Times",
    provider_alternatives=("Eclipse Times",),
    group_element=None,
    fields=(
        FieldSpec("start", ("Start Time", "Start"), required=False),
        FieldSpec("stop", ("Stop Time", "Stop"), required=False),
        FieldSpec("duration", ("Duration",), required=False),
    ),
    description="Lighting intervals. Grouped by condition on STK 12.6.",
)

#: The group members of "Lighting Times", in the order sensing should
#: test them. Sunlight first because it is the common case.
LIGHTING_CONDITIONS = ("Sunlight", "Penumbra", "Umbra")

LINK_INFORMATION = ProviderSpec(
    key="link_information",
    provider_name="Link Information",
    fields=(
        TIME_FIELD,
        FieldSpec("cn0", ("C/N0", "C/No", "C/N"), required=False),
        FieldSpec("ebno", ("Eb/No", "Eb/N0"), required=False),
        FieldSpec("margin", ("Link Margin", "Eb/No Margin"), required=False),
        FieldSpec("rx_power", ("Rcvd. Iso. Power", "Received Power"),
                  required=False),
        FieldSpec("fspl", ("Free Space Loss",), required=False),
    ),
    description="RF link budget figures.",
)

AER = ProviderSpec(
    key="aer",
    provider_name="AER Data",
    provider_alternatives=("AER",),
    group_element="<frame>",
    fields=(
        TIME_FIELD,
        FieldSpec("azimuth", ("Azimuth",)),
        FieldSpec("elevation", ("Elevation",)),
        FieldSpec("range", ("Range",)),
    ),
    description="Azimuth / Elevation / Range to a target.",
)

ALL_SPECS: Dict[str, ProviderSpec] = {
    spec.key: spec for spec in (
        CARTESIAN_POSITION, CARTESIAN_VELOCITY, ATTITUDE_QUATERNION,
        ECF_ATTITUDE_QUATERNION, ATTITUDE_SEGMENT, ANGULAR_VELOCITY,
        LLA_STATE, LIGHTING, LINK_INFORMATION, AER,
    )
}


# ---------------------------------------------------------------------------
# Column resolution
# ---------------------------------------------------------------------------

def resolve_columns(spec: ProviderSpec, data: DataProviderData
                    ) -> Dict[str, Optional[str]]:
    """Map each field's canonical name to the actual column that carried it.

    Matching is case-insensitive and whitespace-insensitive, because
    STK's element names differ from its report column headers in exactly
    those ways depending on which path the data took.

    Raises:
        QueryError: if a required field matched no column. The message
            lists what was actually returned, which is the information
            needed to add the missing spelling to the spec.
    """
    available = {name.strip().lower(): name for name in data.columns}
    mapping: Dict[str, Optional[str]] = {}
    missing: List[str] = []

    for field_spec in spec.fields:
        found = None
        for candidate in field_spec.candidates:
            key = candidate.strip().lower()
            if key in available:
                found = available[key]
                break
        mapping[field_spec.name] = found
        if found is None and field_spec.required:
            missing.append(
                f"{field_spec.name} (tried: {', '.join(field_spec.candidates)})"
            )

    if missing:
        raise QueryError(
            spec.provider_name, data.object_path,
            "required elements not found in the result.\n"
            "  missing: " + "; ".join(missing) + "\n"
            "  returned columns: " + (", ".join(data.columns) or "(none)") +
            "\nIf the column list is empty, the object has no data at the "
            "requested epoch -- see the empty-result guidance in "
            "core/data_providers.execute(). If it is non-empty, add the "
            "actual spelling to the FieldSpec candidates.",
        )
    return mapping


def execute(backend: StkBackend, spec: ProviderSpec, object_path: str,
            frame: str = "ICRF",
            start: Optional[StkTime] = None,
            stop: Optional[StkTime] = None,
            step_s: float = 1.0,
            allow_empty: bool = False) -> Tuple[DataProviderData,
                                                Dict[str, Optional[str]]]:
    """Run a spec against a backend and resolve its columns.

    Tries ``provider_alternatives`` in turn if the primary provider name
    is unknown to this install, so a rename between STK versions degrades
    to a retry rather than a hard failure.

    Args:
        allow_empty: if False (the default), a result with no rows raises
            :class:`QueryError`. Set True only for providers that are
            legitimately empty sometimes -- access-derived data during a
            gap, for instance. Never set it for state data: an empty
            position result means the control loop has no truth, and
            failing loudly is the only safe response.
    """
    group = spec.group_element
    if group == "<frame>":
        group = frame

    names = (spec.provider_name,) + spec.provider_alternatives
    last_error: Optional[Exception] = None
    data: Optional[DataProviderData] = None

    for provider_name in names:
        query = DataProviderQuery(
            object_path=object_path,
            provider_name=provider_name,
            group_element=group,
            start=start, stop=stop, step_s=step_s,
        )
        try:
            data = backend.query_data_provider(query)
            break
        except QueryError as exc:
            last_error = exc
            LOGGER.debug("provider %r failed on %s: %s",
                         provider_name, object_path, exc)

    if data is None:
        raise QueryError(
            spec.provider_name, object_path,
            f"none of these provider names worked: {list(names)}. "
            f"Last error: {last_error}",
        )

    if not data.columns or data.row_count == 0:
        if not allow_empty:
            raise QueryError(
                spec.provider_name, object_path,
                "the query succeeded but returned NO DATA "
                f"({len(data.columns)} columns, {data.row_count} rows).\n"
                "This is almost always one of:\n"
                "  1. The object has no ephemeris. A satellite created via "
                "Children.New has a propagator and elements but no "
                "ephemeris until Propagate() is called. Use "
                "ScenarioManager.ensure_propagated().\n"
                "  2. The requested epoch is outside the object's ephemeris "
                "interval -- including a real-time object queried beyond "
                "its Look-Ahead window, or before any sample was injected.\n"
                "  3. The group element (reference frame) is wrong for this "
                f"provider; requested {group!r}.\n"
                "Empty results are treated as errors because a silently "
                "empty state query would feed an empty control loop.",
            )
        LOGGER.warning("provider %r on %s returned no rows (allowed)",
                       spec.provider_name, object_path)
        # An allowed-empty result has no columns to resolve against, and
        # resolve_columns() enforces required fields -- calling it here
        # would turn "legitimately no data" into a spurious "required
        # elements missing" error. Callers must check for this case
        # (data.row_count == 0) before reading any field; first_value()
        # raises a clear QueryError if they don't.
        return data, {f.name: None for f in spec.fields}

    return data, resolve_columns(spec, data)


def column_values(data: DataProviderData, spec: ProviderSpec,
                  mapping: Dict[str, Optional[str]], field_name: str,
                  angle_rate_scale: Optional[conv.AngleRateScale] = None
                  ) -> List[float]:
    """Extract one field's values, applying its scale."""
    column_name = mapping.get(field_name)
    if column_name is None:
        return []

    field_spec = next((f for f in spec.fields if f.name == field_name), None)
    raw = data.columns.get(column_name, [])

    if field_spec is None or field_spec.scale == 1.0:
        return [conv.as_float(v, default=float("nan")) for v in raw]

    if field_spec.scale is None:
        scale = angle_rate_scale or conv.AngleRateScale()
        return [scale.to_radps(conv.as_float(v, default=float("nan")))
                for v in raw]

    factor = field_spec.scale
    return [conv.as_float(v, default=float("nan")) * factor for v in raw]


def first_value(data: DataProviderData, spec: ProviderSpec,
                mapping: Dict[str, Optional[str]], field_name: str,
                default: Optional[float] = None,
                angle_rate_scale: Optional[conv.AngleRateScale] = None
                ) -> float:
    values = column_values(data, spec, mapping, field_name, angle_rate_scale)
    if values:
        return values[0]
    if default is None:
        raise QueryError(spec.provider_name, data.object_path,
                         f"field {field_name!r} has no values")
    return default


def describe_available(backend: StkBackend, object_path: str,
                       provider_name: str) -> str:
    """Diagnostic: what does this provider actually return here?

    Used by ``validation.py`` and worth calling by hand when a spec
    stops matching after an STK upgrade.
    """
    try:
        data = backend.query_data_provider(DataProviderQuery(
            object_path=object_path, provider_name=provider_name,
        ))
    except QueryError as exc:
        return f"{provider_name}: query failed -- {exc}"
    return (f"{provider_name}: {data.row_count} rows, columns = "
            f"{list(data.columns) or '(none)'}")


# ---------------------------------------------------------------------------
# Discovery
# ---------------------------------------------------------------------------

def discover(backend: StkBackend, object_path: str, *keywords: str
             ) -> List[Tuple[str, List[str]]]:
    """Find providers on an object whose name contains all ``keywords``.

    Case-insensitive substring matching against
    ``backend.list_data_providers``. This is the escape from
    guess-and-check: rather than hardcoding one more spelling each time
    an STK version renames something, ask the install what it has.

    Returns ``[(provider_name, [group_members...]), ...]``, best match
    (shortest name) first, on the reasoning that ``"Angular Velocity"``
    is more likely the general-purpose provider than
    ``"Sensor Body Angular Velocity Delta"``.
    """
    providers = backend.list_data_providers(object_path)
    wanted = [k.strip().lower() for k in keywords if k.strip()]
    matches = [
        (name, members) for name, members in providers.items()
        if all(k in name.lower() for k in wanted)
    ]
    return sorted(matches, key=lambda item: (len(item[0]), item[0]))


def resolve_spec_provider(backend: StkBackend, spec: ProviderSpec,
                          object_path: str, *keywords: str
                          ) -> Optional[str]:
    """Pick a working provider name for ``spec`` on this object.

    Tries the spec's declared names first (cheap, no enumeration), then
    falls back to keyword discovery. Returns None if nothing matches, so
    callers can produce an error that lists what *is* available rather
    than repeating the names that failed.
    """
    available = backend.list_data_providers(object_path)
    if not available:
        return None

    lowered = {name.lower(): name for name in available}
    for candidate in (spec.provider_name,) + spec.provider_alternatives:
        if candidate.lower() in lowered:
            return lowered[candidate.lower()]

    if keywords:
        found = discover(backend, object_path, *keywords)
        if found:
            return found[0][0]
    return None


def format_available(backend: StkBackend, object_path: str,
                     limit: int = 40) -> str:
    """Human-readable provider inventory, for error messages."""
    providers = backend.list_data_providers(object_path)
    if not providers:
        return "(this backend cannot enumerate data providers)"
    lines = []
    for name in sorted(providers)[:limit]:
        members = providers[name]
        lines.append(f"  {name}" + (f"  [{', '.join(members)}]"
                                    if members else ""))
    if len(providers) > limit:
        lines.append(f"  ... and {len(providers) - limit} more")
    return "\n".join(lines)
