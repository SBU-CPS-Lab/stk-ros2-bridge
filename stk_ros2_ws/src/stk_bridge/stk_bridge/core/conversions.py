"""Unit, type, and convention conversions between STK and ROS 2.

Backend-agnostic and ROS-optional: ``rclpy`` and ``builtin_interfaces``
are imported lazily inside the functions that need them, so this module
can be unit-tested without a ROS installation (``test_conversions.py``).

What lives here
---------------
* :class:`StkTime` <-> ``builtin_interfaces/Time`` conversion.
* Quaternion algebra and ordering guarantees.
* Angle/rate scaling, for the dimensions where STK's unit preferences
  cannot be forced to SI.
* STK object-path canonicalisation.

What does NOT live here
-----------------------
Distance and velocity conversion. Backends force STK's ``UnitPreferences``
to metres at connect time, so position and velocity arrive in SI already.
See ``backends/base.py``'s units contract for why that is the right place
for it.

The one dimension that resists this: **angular rate**. On a live STK 12.6
install, ``SetCurrentUnit("AngleRate", "rad/sec")`` is rejected with
"Cannot set the specified unit." So angular-rate values from data
providers may be in degrees/second regardless of preference.
:class:`AngleRateScale` handles that explicitly rather than assuming, and
``core/validation.py`` measures the actual scale at startup instead of
trusting either guess.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import Any, Iterable, List, Optional, Sequence, Tuple

from stk_bridge.backends.base import StkTime

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi

#: Tolerance for treating a quaternion as unit-norm. Loose enough to
#: accept single-precision round-tripping through STK's text interfaces,
#: tight enough that a genuinely unnormalised input is caught.
QUATERNION_NORM_TOLERANCE = 1e-6

_NANOSECONDS = 1_000_000_000


# ---------------------------------------------------------------------------
# Time
# ---------------------------------------------------------------------------

def stk_time_to_ros(when: StkTime) -> Any:
    """Convert an :class:`StkTime` to ``builtin_interfaces/Time``.

    Splitting into integer seconds plus nanoseconds is done with
    ``divmod`` on an integer nanosecond count rather than by taking the
    fractional part of a float, because the latter loses resolution for
    large epoch values -- a float64 holding ~1.7e9 seconds has only
    ~200 ns of precision left in its mantissa, which is enough to make
    two distinct 1 kHz samples compare equal.
    """
    from builtin_interfaces.msg import Time as RosTime

    total_ns = int(round(when.epoch_seconds * _NANOSECONDS))
    sec, nanosec = divmod(total_ns, _NANOSECONDS)
    return RosTime(sec=int(sec), nanosec=int(nanosec))


def ros_time_to_stk(stamp: Any) -> StkTime:
    """Convert ``builtin_interfaces/Time`` to :class:`StkTime`."""
    return StkTime(float(stamp.sec) + float(stamp.nanosec) / _NANOSECONDS)


def stk_time_to_ros_nanoseconds(when: StkTime) -> int:
    """Nanosecond count, for ``rclpy.time.Time(nanoseconds=...)``."""
    return int(round(when.epoch_seconds * _NANOSECONDS))


def duration_to_ros(seconds: float) -> Any:
    from builtin_interfaces.msg import Duration as RosDuration

    total_ns = int(round(seconds * _NANOSECONDS))
    sec, nanosec = divmod(total_ns, _NANOSECONDS)
    return RosDuration(sec=int(sec), nanosec=int(nanosec))


# ---------------------------------------------------------------------------
# Quaternions
# ---------------------------------------------------------------------------

Quaternion = Tuple[float, float, float, float]


def normalize_quaternion(q: Sequence[float]) -> Quaternion:
    """Return ``q`` scaled to unit norm, as ``(x, y, z, w)``.

    Raises:
        ValueError: if the norm is too small to normalise. A zero
            quaternion is not a degenerate rotation to be silently
            replaced with identity -- it means the producer is broken,
            and substituting identity would hide that behind an attitude
            command that looks plausible.
    """
    if len(q) != 4:
        raise ValueError(f"quaternion must have 4 components, got {len(q)}")
    norm = math.sqrt(sum(float(c) * float(c) for c in q))
    if norm < 1e-12:
        raise ValueError(
            "cannot normalise a near-zero quaternion; the source is "
            "producing invalid attitude data"
        )
    return (float(q[0]) / norm, float(q[1]) / norm,
            float(q[2]) / norm, float(q[3]) / norm)


def is_unit_quaternion(q: Sequence[float],
                       tolerance: float = QUATERNION_NORM_TOLERANCE) -> bool:
    if len(q) != 4:
        return False
    norm = math.sqrt(sum(float(c) * float(c) for c in q))
    return abs(norm - 1.0) <= tolerance


def quaternion_conjugate(q: Sequence[float]) -> Quaternion:
    return (-float(q[0]), -float(q[1]), -float(q[2]), float(q[3]))


def quaternion_multiply(a: Sequence[float], b: Sequence[float]) -> Quaternion:
    """Hamilton product ``a * b``, both and result in ``(x, y, z, w)``."""
    ax, ay, az, aw = (float(v) for v in a)
    bx, by, bz, bw = (float(v) for v in b)
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def rotate_vector(q: Sequence[float],
                  v: Sequence[float]) -> Tuple[float, float, float]:
    """Rotate ``v`` by unit quaternion ``q``."""
    qx, qy, qz, qw = (float(c) for c in q)
    vx, vy, vz = (float(c) for c in v)
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def quaternion_angle_between(a: Sequence[float],
                             b: Sequence[float]) -> float:
    """Smallest rotation angle between two orientations, radians.

    Uses ``abs`` on the dot product so that ``q`` and ``-q`` -- which
    represent the same orientation -- compare as identical. Forgetting
    this double-cover property is a classic source of phantom 180-degree
    attitude errors, and ``validation.py`` relies on this function
    behaving correctly.
    """
    dot = abs(sum(float(x) * float(y) for x, y in zip(a, b)))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def quaternion_from_stk_elements(q1: float, q2: float, q3: float,
                                 q4: float) -> Quaternion:
    """Map STK's Q1..Q4 to ROS ``(x, y, z, w)``.

    STK documents Q1..Q3 as the vector part and Q4 as the scalar part,
    which is the same ordering ROS uses, so this is an identity mapping.
    It exists as a named function anyway so that there is exactly one
    place to change if a given provider turns out to disagree, and so
    that call sites read as an explicit conversion rather than an
    unexamined assumption. ``validation.py`` verifies the mapping empirically.
    """
    return (float(q1), float(q2), float(q3), float(q4))


def quaternion_to_stk_elements(q: Sequence[float]) -> Quaternion:
    """Inverse of :func:`quaternion_from_stk_elements`."""
    return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))


def quaternion_to_euler_zyx(q: Sequence[float]) -> Tuple[float, float, float]:
    """Yaw-pitch-roll (Z-Y-X intrinsic), radians. Diagnostics only."""
    x, y, z, w = (float(c) for c in q)
    sin_pitch = 2.0 * (w * y - z * x)
    sin_pitch = max(-1.0, min(1.0, sin_pitch))
    return (
        math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)),
        math.asin(sin_pitch),
        math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)),
    )


# ---------------------------------------------------------------------------
# Angular rate scaling
# ---------------------------------------------------------------------------

@dataclass
class AngleRateScale:
    """Scale factor from STK's reported angular rate to rad/s.

    Exists because STK 12.6 rejects ``SetCurrentUnit("AngleRate",
    "rad/sec")``, so the backend cannot guarantee SI for this one
    dimension the way it can for distance. Rather than guess,
    ``core/validation.py`` measures the actual scale at startup by
    commanding a known body rate and reading it back, then installs the
    result here.

    Default is 1.0 (assume rad/s) with ``measured=False``, so that an
    unvalidated bridge is at least self-consistent; sensing marks
    angular-rate output as unvalidated in that state rather than
    presenting a guess as truth.
    """

    factor: float = 1.0
    measured: bool = False
    source: str = "assumed rad/s (not measured)"

    def to_radps(self, value: float) -> float:
        return float(value) * self.factor

    def from_radps(self, value: float) -> float:
        return float(value) / self.factor

    @classmethod
    def degrees_per_second(cls) -> "AngleRateScale":
        return cls(factor=DEG_TO_RAD, measured=True,
                   source="measured: STK reports deg/s")

    @classmethod
    def radians_per_second(cls) -> "AngleRateScale":
        return cls(factor=1.0, measured=True,
                   source="measured: STK reports rad/s")


# ---------------------------------------------------------------------------
# STK object paths
# ---------------------------------------------------------------------------

_ABSOLUTE_PATH_RE = re.compile(
    r"^/Application/STK/Scenario/(?P<scenario>[^/]+)/(?P<rest>.+)$"
)


def canonical_object_path(path: str) -> str:
    """Reduce an STK object path to its scenario-relative ``*/`` form.

    STK reports absolute paths
    (``/Application/STK/Scenario/MyScn/Satellite/SSS-1``) from
    ``child.Path``, but accepts and is more conveniently addressed by the
    relative form (``*/Satellite/SSS-1``). Mixing the two is a real
    source of bugs: a dictionary keyed on one form silently misses
    lookups made with the other, so every path entering ``core/`` is
    canonicalised here first.
    """
    path = (path or "").strip().strip('"')
    if not path:
        return path
    match = _ABSOLUTE_PATH_RE.match(path)
    if match:
        return "*/" + match.group("rest")
    if path.startswith("*/"):
        return path
    if path.startswith("/"):
        return path
    return "*/" + path.lstrip("/")


def object_name(path: str) -> str:
    """Trailing name component of an object path."""
    return canonical_object_path(path).rstrip("/").rsplit("/", 1)[-1]


def object_class(path: str) -> str:
    """The STK class name of an object path (``Satellite``, ``Sensor``...)."""
    parts = canonical_object_path(path).rstrip("/").split("/")
    return parts[-2] if len(parts) >= 2 else ""


def child_path(parent: str, object_class_name: str, name: str) -> str:
    return f"{canonical_object_path(parent)}/{object_class_name}/{name}"


# ---------------------------------------------------------------------------
# Numeric helpers
# ---------------------------------------------------------------------------

def as_float(value: Any, default: Optional[float] = None) -> float:
    """Coerce a data-provider cell to float.

    Data providers return strings for some elements even when the value
    is numeric, and return sentinel text (``"N/A"``, ``"--"``) where a
    quantity is undefined at that epoch. Callers pass ``default=None`` to
    get an exception on unparseable input, or a value to substitute.
    """
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return float(value)
    try:
        return float(str(value).strip())
    except (TypeError, ValueError):
        if default is None:
            raise ValueError(f"cannot interpret {value!r} as a number")
        return default


def vector3(values: Iterable[Any]) -> Tuple[float, float, float]:
    out: List[float] = [as_float(v) for v in values]
    if len(out) != 3:
        raise ValueError(f"expected 3 components, got {len(out)}")
    return (out[0], out[1], out[2])


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))
