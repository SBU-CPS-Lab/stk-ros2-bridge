"""Quaternion algebra and attitude kinematics for the ACAV twin.

The conventions here are the ones the whole study runs on, and every one
of them is **pinned by a test that asserts a physical outcome** rather
than by a comment. That is deliberate: a sign or ordering error in this
file produces a twin that flies smoothly to the wrong attitude, STK
dutifully reports no access, and the result looks like a control-design
finding instead of a bug. The bridge's own bring-up lost a cycle to
exactly this (an ECF-commanded attitude read back against an inertial
provider, wrong by the Earth-rotation angle, entirely plausible).

Conventions
-----------
**Component order** ``(x, y, z, w)`` -- vector part first, scalar last.
This matches the bridge's contract (``backends/base.py``) and ROS's
``geometry_msgs/Quaternion``. STK reports ``Q1..Q3`` as the vector part
and ``Q4`` as the scalar, which ``conversions.py`` already maps here.

**Direction** ``q`` is the attitude of the body with respect to
inertial: it rotates a vector *from inertial into body components*.
``rotate_to_body(q, v_inertial) -> v_body``. The matrix is

    R(q) = (w^2 - |v|^2) I + 2 v v^T - 2 w [v x]

**Product** Hamilton, and composition follows ``R(a (x) b) = R(b) R(a)``
-- verified in ``test_attitude.py`` rather than asserted, because the
two common conventions differ by exactly this and both look right.

**Rates** ``omega`` is the angular velocity of the body with respect to
inertial, expressed in **body** axes, rad/s. The kinematics are then
``qdot = 0.5 * q (x) (omega, 0)``.

Why not a library
-----------------
A dependency would have to be installed on the Windows deploy machine
and matched by an equivalent in MATLAB, and the MATLAB side has to be
hand-written regardless. Two hand-written implementations that must
agree is the risk this module exists to manage, so it is small, exact,
and cross-checked against the MATLAB twin through exported fixtures
(``scripts/export_twin_fixtures.py``).
"""

from __future__ import annotations

import math
from typing import Sequence, Tuple

Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]

IDENTITY: Quaternion = (0.0, 0.0, 0.0, 1.0)


# ---------------------------------------------------------------------------
# Vector helpers
# ---------------------------------------------------------------------------

def cross(a: Sequence[float], b: Sequence[float]) -> Vector3:
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def dot(a: Sequence[float], b: Sequence[float]) -> float:
    return sum(x * y for x, y in zip(a, b))


def norm(a: Sequence[float]) -> float:
    return math.sqrt(dot(a, a))


def normalize(a: Sequence[float]) -> Vector3:
    magnitude = norm(a)
    if magnitude < 1e-12:
        raise ValueError(
            f"cannot normalize a zero-length vector {tuple(a)}; this usually "
            f"means two directions that should differ are coincident -- a "
            f"target directly along the constraint axis, say")
    return (a[0] / magnitude, a[1] / magnitude, a[2] / magnitude)


def scale(a: Sequence[float], factor: float) -> Vector3:
    return (a[0] * factor, a[1] * factor, a[2] * factor)


def subtract(a: Sequence[float], b: Sequence[float]) -> Vector3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


# ---------------------------------------------------------------------------
# Quaternion algebra
# ---------------------------------------------------------------------------

def quat_normalize(q: Sequence[float]) -> Quaternion:
    """Renormalize, preserving sign.

    Called every integration step. Numerical drift off the unit sphere
    is slow but monotone, and an un-normalized quaternion silently
    scales the rotation matrix -- so the attitude stays plausible while
    slowly becoming wrong.
    """
    magnitude = math.sqrt(sum(c * c for c in q))
    if magnitude < 1e-12:
        raise ValueError(f"quaternion has no magnitude: {tuple(q)}")
    return (q[0] / magnitude, q[1] / magnitude,
            q[2] / magnitude, q[3] / magnitude)


def quat_conjugate(q: Sequence[float]) -> Quaternion:
    return (-q[0], -q[1], -q[2], q[3])


def quat_multiply(a: Sequence[float], b: Sequence[float]) -> Quaternion:
    """Hamilton product ``a (x) b``."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + bw * ax + ay * bz - az * by,
        aw * by + bw * ay + az * bx - ax * bz,
        aw * bz + bw * az + ax * by - ay * bx,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_from_axis_angle(axis: Sequence[float], angle_rad: float
                         ) -> Quaternion:
    unit = normalize(axis)
    half = 0.5 * angle_rad
    s = math.sin(half)
    return (unit[0] * s, unit[1] * s, unit[2] * s, math.cos(half))


def axis_angle_from_quat(q: Sequence[float]) -> Tuple[Vector3, float]:
    """Rotation axis and angle in [0, pi], shortest way round.

    The sign flip matters: ``q`` and ``-q`` are the same attitude, and
    without it a controller fed the long-way-round error would slew
    350 degrees to reach a 10 degree target. That failure is silent --
    the vehicle does arrive.
    """
    q = quat_normalize(q)
    if q[3] < 0.0:
        q = (-q[0], -q[1], -q[2], -q[3])
    vector_norm = math.sqrt(q[0] ** 2 + q[1] ** 2 + q[2] ** 2)
    angle = 2.0 * math.atan2(vector_norm, q[3])
    if vector_norm < 1e-12:
        return (1.0, 0.0, 0.0), 0.0
    return (q[0] / vector_norm, q[1] / vector_norm, q[2] / vector_norm), angle


def rotate_to_body(q: Sequence[float], v_inertial: Sequence[float]
                   ) -> Vector3:
    """Express an inertial vector in body axes."""
    x, y, z, w = q
    v = (x, y, z)
    term_a = scale(v_inertial, w * w - dot(v, v))
    term_b = scale(v, 2.0 * dot(v, v_inertial))
    term_c = scale(cross(v, v_inertial), -2.0 * w)
    return (term_a[0] + term_b[0] + term_c[0],
            term_a[1] + term_b[1] + term_c[1],
            term_a[2] + term_b[2] + term_c[2])


def rotate_to_inertial(q: Sequence[float], v_body: Sequence[float]
                       ) -> Vector3:
    return rotate_to_body(quat_conjugate(q), v_body)


def quat_derivative(q: Sequence[float], omega_body: Sequence[float]
                    ) -> Quaternion:
    """``qdot = 0.5 * q (x) (omega, 0)``, inertial-to-body convention."""
    return scale4(quat_multiply(q, (omega_body[0], omega_body[1],
                                    omega_body[2], 0.0)), 0.5)


def scale4(q: Sequence[float], factor: float) -> Quaternion:
    return (q[0] * factor, q[1] * factor, q[2] * factor, q[3] * factor)


def quat_error(q: Sequence[float], q_reference: Sequence[float]
               ) -> Quaternion:
    """Rotation from the reference body frame to the actual body frame.

    Its vector part is what a quaternion-feedback controller drives to
    zero, expressed in **body** axes, which is where the torque is
    applied.
    """
    return quat_multiply(quat_conjugate(q_reference), q)


def pointing_error_rad(q: Sequence[float], q_reference: Sequence[float]
                       ) -> float:
    """Total angular separation between two attitudes, radians."""
    return axis_angle_from_quat(quat_error(q, q_reference))[1]


def boresight_error_rad(q: Sequence[float], q_reference: Sequence[float],
                        boresight_body: Sequence[float] = (0.0, 0.0, 1.0)
                        ) -> float:
    """Angle between the achieved and desired boresight directions.

    Distinct from :func:`pointing_error_rad`, and the distinction is the
    one that decides captures: rotation *about* the boresight does not
    move the footprint, so a twin that is settled in boresight but not
    in roll still images the target. STK's access computation cares
    about the boresight only, so the capture criterion must too --
    scoring total attitude error would fail targets the sensor actually
    sees.
    """
    achieved = rotate_to_inertial(q, boresight_body)
    desired = rotate_to_inertial(q_reference, boresight_body)
    # atan2 of the cross and dot products, not acos of the dot product.
    # acos loses half its significant digits near zero -- it returned
    # 2e-8 rad for two exactly-parallel directions -- and this number is
    # compared against a 0.5 deg capture threshold, so it should be
    # accurate where it is small rather than only where it is large.
    return math.atan2(norm(cross(achieved, desired)),
                      dot(achieved, desired))


def quat_from_dcm_columns(x_axis: Sequence[float], y_axis: Sequence[float],
                          z_axis: Sequence[float]) -> Quaternion:
    """Attitude whose body axes are the given inertial-frame directions.

    ``x_axis`` etc. are the body axes *expressed in inertial
    components*, so the inertial-to-body matrix has them as its **rows**.
    Shepperd's method: pick the largest of the four denominators, which
    keeps it conditioned for every attitude including the 180 degree
    cases where the naive formula divides by zero.
    """
    r = ((x_axis[0], x_axis[1], x_axis[2]),
         (y_axis[0], y_axis[1], y_axis[2]),
         (z_axis[0], z_axis[1], z_axis[2]))
    trace = r[0][0] + r[1][1] + r[2][2]

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (r[1][2] - r[2][1]) / s
        y = (r[2][0] - r[0][2]) / s
        z = (r[0][1] - r[1][0]) / s
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2.0
        w = (r[1][2] - r[2][1]) / s
        x = 0.25 * s
        y = (r[1][0] + r[0][1]) / s
        z = (r[2][0] + r[0][2]) / s
    elif r[1][1] > r[2][2]:
        s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2.0
        w = (r[2][0] - r[0][2]) / s
        x = (r[1][0] + r[0][1]) / s
        y = 0.25 * s
        z = (r[2][1] + r[1][2]) / s
    else:
        s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2.0
        w = (r[0][1] - r[1][0]) / s
        x = (r[2][0] + r[0][2]) / s
        y = (r[2][1] + r[1][2]) / s
        z = 0.25 * s

    return quat_normalize((x, y, z, w))


def quat_align(primary_body: Sequence[float],
               primary_inertial: Sequence[float],
               secondary_body: Sequence[float],
               secondary_inertial: Sequence[float]) -> Quaternion:
    """Attitude aligning a body axis with a direction, plus a constraint.

    ``primary_body`` is pointed exactly at ``primary_inertial``; the
    secondary pair is satisfied as nearly as the primary allows. This is
    the TRIAD construction, and it is how every commanded attitude in
    the study is built: point the boresight at the target, and use the
    orbit-normal or velocity direction to pin the remaining rotation
    about it.

    Raises if the two inertial directions are parallel, rather than
    silently returning an arbitrary roll -- an arbitrary but *stable*
    roll would look fine and quietly change which way the footprint is
    smeared.
    """
    p_b = normalize(primary_body)
    p_i = normalize(primary_inertial)
    s_b = normalize(secondary_body)
    s_i = normalize(secondary_inertial)

    if abs(dot(p_i, s_i)) > 1.0 - 1e-9:
        raise ValueError(
            "the primary and secondary inertial directions are parallel, so "
            "the rotation about the boresight is undefined. Choose a "
            "different constraint axis for this geometry.")

    # Orthonormal triads in each frame, then the rotation between them.
    b1 = p_b
    b2 = normalize(cross(p_b, s_b))
    b3 = cross(b1, b2)
    i1 = p_i
    i2 = normalize(cross(p_i, s_i))
    i3 = cross(i1, i2)

    # Body axes expressed in inertial components: rows of the
    # inertial-to-body matrix.
    x_axis = tuple(b1[k] * i1[j] + b2[k] * i2[j] + b3[k] * i3[j]
                   for j, k in ((0, 0), (1, 0), (2, 0)))
    y_axis = tuple(b1[k] * i1[j] + b2[k] * i2[j] + b3[k] * i3[j]
                   for j, k in ((0, 1), (1, 1), (2, 1)))
    z_axis = tuple(b1[k] * i1[j] + b2[k] * i2[j] + b3[k] * i3[j]
                   for j, k in ((0, 2), (1, 2), (2, 2)))
    return quat_from_dcm_columns(x_axis, y_axis, z_axis)
