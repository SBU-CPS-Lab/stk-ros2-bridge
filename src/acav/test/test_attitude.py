"""Tests that pin the attitude conventions to physical outcomes.

Every convention in `acav/attitude.py` has a plausible-looking opposite,
and picking the wrong one yields a twin that flies smoothly to the wrong
place while every number stays finite and every plot looks reasonable.
So none of these tests check a formula against itself: each asserts
something physical -- where a vector ends up after a known rotation,
which way a body turns under a known rate -- so that an implementation
satisfying them is right regardless of which sign convention its author
had in mind.
"""

import math

import pytest

from acav.attitude import (
    IDENTITY,
    axis_angle_from_quat,
    boresight_error_rad,
    cross,
    dot,
    normalize,
    pointing_error_rad,
    quat_align,
    quat_conjugate,
    quat_derivative,
    quat_error,
    quat_from_axis_angle,
    quat_multiply,
    quat_normalize,
    rotate_to_body,
    rotate_to_inertial,
)

X, Y, Z = (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)


def approx_vec(actual, expected, abs_tol=1e-9):
    assert len(actual) == len(expected)
    for got, want in zip(actual, expected):
        assert got == pytest.approx(want, abs=abs_tol)


class TestRotationDirection:
    """`q` takes an inertial vector into body components."""

    def test_identity_changes_nothing(self):
        approx_vec(rotate_to_body(IDENTITY, (1.0, 2.0, 3.0)), (1.0, 2.0, 3.0))

    def test_body_yawed_ninety_degrees_about_z(self):
        """Rotate the BODY +90 deg about z. The inertial x axis must then
        appear along the body's -y axis.

        This is the test that fixes the direction of the convention. The
        opposite convention puts it on +y, and every subsequent sign in
        the controller follows from which one is true here.
        """
        q = quat_from_axis_angle(Z, math.radians(90.0))
        approx_vec(rotate_to_body(q, X), (0.0, -1.0, 0.0))
        approx_vec(rotate_to_body(q, Y), (1.0, 0.0, 0.0))

    def test_round_trip_through_both_directions(self):
        q = quat_from_axis_angle((1.0, 2.0, -0.5), 0.9)
        v = (0.3, -1.7, 2.2)
        approx_vec(rotate_to_inertial(q, rotate_to_body(q, v)), v)

    def test_rotation_preserves_length_and_angles(self):
        q = quat_from_axis_angle((0.2, -1.0, 0.4), 2.1)
        a, b = (1.0, 2.0, 3.0), (-2.0, 0.5, 1.0)
        ra, rb = rotate_to_body(q, a), rotate_to_body(q, b)
        assert dot(ra, ra) == pytest.approx(dot(a, a))
        assert dot(ra, rb) == pytest.approx(dot(a, b))


class TestComposition:
    """`R(a (x) b) = R(b) R(a)` -- asserted, not assumed."""

    def test_composition_order(self):
        a = quat_from_axis_angle(Z, 0.4)
        b = quat_from_axis_angle(X, -0.7)
        v = (0.5, -1.2, 0.3)
        approx_vec(rotate_to_body(quat_multiply(a, b), v),
                   rotate_to_body(b, rotate_to_body(a, v)))

    def test_conjugate_is_the_inverse(self):
        q = quat_from_axis_angle((1.0, -1.0, 2.0), 1.3)
        product = quat_multiply(q, quat_conjugate(q))
        approx_vec(product, IDENTITY)


class TestKinematics:
    """A constant body rate must produce the rotation it physically would."""

    def test_constant_rate_about_z_matches_the_closed_form(self):
        """Integrate omega = (0,0,W) and compare against the exact
        attitude at the end. An error in the sign or the product order
        shows up here as a rotation the wrong way, which is otherwise
        indistinguishable from a controller that overshoots.
        """
        rate = 0.05
        duration = 4.0
        steps = 40000
        dt = duration / steps

        q = IDENTITY
        for _ in range(steps):
            derivative = quat_derivative(q, (0.0, 0.0, rate))
            q = quat_normalize(tuple(c + d * dt
                                     for c, d in zip(q, derivative)))

        exact = quat_from_axis_angle(Z, rate * duration)
        assert pointing_error_rad(q, exact) < 1e-6
        approx_vec(rotate_to_body(q, X),
                   (math.cos(rate * duration), -math.sin(rate * duration),
                    0.0), abs_tol=1e-6)

    def test_rate_about_a_body_axis_stays_about_that_body_axis(self):
        """Body-frame rates rotate with the body. Integrating a constant
        body rate about +X from a yawed initial attitude must keep the
        rotation axis fixed in the BODY, not in inertial space."""
        q = quat_from_axis_angle(Z, 1.0)
        rate = 0.1
        dt = 1e-4
        for _ in range(20000):
            derivative = quat_derivative(q, (rate, 0.0, 0.0))
            q = quat_normalize(tuple(c + d * dt
                                     for c, d in zip(q, derivative)))
        # Composition of the initial yaw and a body-frame roll.
        expected = quat_multiply(quat_from_axis_angle(Z, 1.0),
                                 quat_from_axis_angle(X, rate * 2.0))
        assert pointing_error_rad(q, expected) < 1e-6


class TestErrorQuaternion:
    def test_zero_when_attitudes_match(self):
        q = quat_from_axis_angle((0.3, 1.0, -0.2), 0.8)
        assert pointing_error_rad(q, q) == pytest.approx(0.0, abs=1e-12)

    def test_error_angle_equals_the_separation(self):
        reference = quat_from_axis_angle(Y, 0.4)
        offset = math.radians(17.0)
        actual = quat_multiply(reference, quat_from_axis_angle(X, offset))
        assert pointing_error_rad(actual, reference) == pytest.approx(offset)

    def test_error_vector_is_expressed_in_body_axes(self):
        """The controller applies torque in body axes, so the error's
        vector part has to live there. Offsetting about body +X must put
        the error entirely in its x component whatever the reference
        attitude is."""
        reference = quat_from_axis_angle((0.2, -0.9, 0.5), 1.1)
        actual = quat_multiply(reference,
                               quat_from_axis_angle(X, math.radians(10.0)))
        error = quat_error(actual, reference)
        assert error[0] == pytest.approx(math.sin(math.radians(5.0)))
        assert error[1] == pytest.approx(0.0, abs=1e-12)
        assert error[2] == pytest.approx(0.0, abs=1e-12)

    @pytest.mark.parametrize("angle_deg", [10.0, 179.0, 181.0, 350.0])
    def test_never_takes_the_long_way_round(self, angle_deg):
        """`q` and `-q` are the same attitude. A controller handed the
        long-way-round error slews 350 deg to reach a 10 deg target, and
        it does arrive, so nothing looks wrong."""
        reference = IDENTITY
        actual = quat_from_axis_angle(Z, math.radians(angle_deg))
        expected = math.radians(min(angle_deg % 360.0,
                                    360.0 - angle_deg % 360.0))
        assert pointing_error_rad(actual, reference) == pytest.approx(
            expected, abs=1e-9)

    def test_sign_flipped_quaternion_is_the_same_attitude(self):
        q = quat_from_axis_angle((1.0, 0.5, -2.0), 2.0)
        flipped = tuple(-c for c in q)
        assert pointing_error_rad(q, flipped) == pytest.approx(0.0, abs=1e-9)


class TestBoresightError:
    """Rotation about the boresight does not move the footprint."""

    def test_roll_about_the_boresight_is_not_a_pointing_error(self):
        reference = quat_from_axis_angle(X, 0.3)
        rolled = quat_multiply(reference,
                               quat_from_axis_angle(Z, math.radians(40.0)))
        assert boresight_error_rad(rolled, reference) == pytest.approx(
            0.0, abs=1e-9)
        # ... while total attitude error sees the full 40 deg. Scoring
        # captures on that would fail targets the sensor actually images.
        assert pointing_error_rad(rolled, reference) == pytest.approx(
            math.radians(40.0))

    def test_tilt_off_the_boresight_is_measured_in_full(self):
        reference = quat_from_axis_angle(Z, 1.2)
        tilted = quat_multiply(reference,
                               quat_from_axis_angle(Y, math.radians(3.0)))
        assert boresight_error_rad(tilted, reference) == pytest.approx(
            math.radians(3.0), abs=1e-9)


class TestQuatAlign:
    """Building a commanded attitude: point the boresight, pin the roll."""

    def test_primary_axis_lands_exactly_on_the_target_direction(self):
        target = normalize((0.3, -0.8, -1.0))
        velocity = (1.0, 0.1, 0.0)
        q = quat_align(Z, target, X, velocity)
        approx_vec(rotate_to_inertial(q, Z), target)

    def test_secondary_axis_is_as_close_as_the_primary_allows(self):
        """The constraint axis cannot be met exactly unless it happens to
        be perpendicular to the boresight; what it must do is remove the
        remaining freedom, leaving the secondary in the plane spanned by
        the primary and the requested direction."""
        target = normalize((0.0, 0.0, -1.0))
        velocity = normalize((1.0, 0.0, 0.0))
        q = quat_align(Z, target, X, velocity)
        achieved_x = rotate_to_inertial(q, X)
        assert dot(achieved_x, target) == pytest.approx(0.0, abs=1e-9)
        assert dot(achieved_x, velocity) > 0.99

    def test_parallel_directions_raise_rather_than_pick_a_roll(self):
        """An arbitrary but stable roll would look completely fine and
        quietly change which way the footprint is oriented."""
        with pytest.raises(ValueError, match="parallel"):
            quat_align(Z, (0.0, 0.0, 1.0), X, (0.0, 0.0, 1.0))

    def test_nadir_pointing_from_a_real_orbit_state(self):
        """The nominal ACAV attitude: +Z at nadir, +X along velocity."""
        position = (6_878_137.0, 0.0, 0.0)
        velocity = (0.0, 1000.0, 7500.0)
        nadir = normalize(tuple(-c for c in position))
        q = quat_align(Z, nadir, X, velocity)

        approx_vec(rotate_to_inertial(q, Z), nadir)
        # +Y completes a right-handed set and should sit near the
        # negative orbit normal.
        orbit_normal = normalize(cross(position, velocity))
        assert dot(rotate_to_inertial(q, Y), orbit_normal) < -0.99
