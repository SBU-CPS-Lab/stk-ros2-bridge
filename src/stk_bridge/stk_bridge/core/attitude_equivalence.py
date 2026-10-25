"""High-resolution comparison of two attitude quaternions.

Shared by the equivalence diagnostic and the combined attitude/rate read
(``sensing.combine_attitude_rate``), both of which must tell 1e-12 rad
from zero. ``conversions.quaternion_angle_between`` cannot: its
``2*acos(|a.b|)`` form has a floor of about 3e-8 rad in double precision
(acos is flat at 1). That is harmless for its own use, the 0.1 deg
landed-attitude check, and it is left unchanged.
"""

from __future__ import annotations

import math
from typing import Sequence

#: Largest sign-aligned angle at which the segment quaternion is accepted
#: as the validated read-back (analysis decision Q6).
COMBINE_TOLERANCE_RAD = 1e-12


def sign_aligned_angle(a: Sequence[float], b: Sequence[float]) -> float:
    """Rotation angle between two orientations, radians, resolvable to 1e-12.

    ``4*atan2(|a - s*b|, |a + s*b|)`` with ``s = sign(a.b)``, so q and -q
    compare as identical. Algebraically the same angle as
    ``conversions.quaternion_angle_between`` (``2*acos(|a.b|)``).
    """
    s = 1.0 if sum(x * y for x, y in zip(a, b)) >= 0.0 else -1.0
    difference = math.sqrt(sum((x - s * y) ** 2 for x, y in zip(a, b)))
    total = math.sqrt(sum((x + s * y) ** 2 for x, y in zip(a, b)))
    return 4.0 * math.atan2(difference, total)
