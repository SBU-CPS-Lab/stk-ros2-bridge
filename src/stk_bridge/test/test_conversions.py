"""Unit tests for core/conversions.py.

Every test here corresponds to a real failure mode this project hit or
deliberately guards against -- see the referenced module docstrings for
context. Pure functions, no ROS/STK dependency; runs anywhere.
"""

import math

import pytest

from stk_bridge.core import conversions as conv
from stk_bridge.backends.base import StkTime

# stk_time_to_ros()/ros_time_to_stk() lazily import builtin_interfaces
# (see conversions.py's module docstring on why this stays lazy: core/
# must remain importable with no ROS installed). That means the time-
# conversion tests specifically need a ROS 2 workspace on the path, while
# everything else in this file does not.
builtin_interfaces = pytest.importorskip(
    "builtin_interfaces", reason="requires a sourced ROS 2 workspace")


class TestQuaternionOrdering:
    """The class of bug that produced a 99 deg 'ordering error' that was
    actually a frame-pairing error during Phase 3 bring-up (see
    core/validation.py's docstring). These tests guard the primitives
    that check depends on.
    """

    def test_double_cover_q_and_negative_q_are_identical(self):
        q = conv.normalize_quaternion((0.1, 0.2, 0.3, 0.9))
        neg_q = tuple(-c for c in q)
        assert conv.quaternion_angle_between(q, neg_q) < 1e-9

    def test_scalar_vector_swap_is_detected(self):
        """A scalar/vector transposition must NOT look like identity."""
        q = conv.normalize_quaternion((0.0692, 0.1383, 0.2075, 0.9659))
        swapped = (q[3], q[0], q[1], q[2])   # (w,x,y,z) reinterpreted as (x,y,z,w)
        error_deg = math.degrees(conv.quaternion_angle_between(q, swapped))
        assert error_deg > 30.0, (
            "a scalar/vector swap must produce a large, detectable error"
        )

    def test_frame_rotation_changes_magnitudes_permutation_does_not(self):
        """The diagnostic validation.py uses to distinguish an ordering
        fault (magnitudes match, order doesn't) from a frame fault
        (magnitudes genuinely differ)."""
        q = (0.069172, 0.138345, 0.207517, 0.965926)
        permuted = (q[1], q[2], q[3], q[0])
        assert sorted(abs(c) for c in q) == pytest.approx(
            sorted(abs(c) for c in permuted))

        # A real frame rotation (not a permutation) changes the sorted
        # magnitude set.
        rotated = conv.quaternion_multiply((0, 0, 0.5, math.sqrt(0.75)), q)
        assert sorted(abs(c) for c in q) != pytest.approx(
            sorted(abs(c) for c in rotated), abs=1e-6)

    def test_stk_element_mapping_is_identity(self):
        assert conv.quaternion_from_stk_elements(1, 2, 3, 4) == (1, 2, 3, 4)

    def test_normalize_rejects_near_zero(self):
        with pytest.raises(ValueError):
            conv.normalize_quaternion((1e-14, 0, 0, 0))

    def test_rotate_vector_identity_quaternion(self):
        v = (1.0, 2.0, 3.0)
        assert conv.rotate_vector((0, 0, 0, 1), v) == pytest.approx(v)

    def test_rotate_vector_90deg_about_z(self):
        half = math.pi / 4
        q = (0, 0, math.sin(half), math.cos(half))
        result = conv.rotate_vector(q, (1.0, 0.0, 0.0))
        assert result == pytest.approx((0.0, 1.0, 0.0), abs=1e-9)


class TestTimeConversion:
    def test_round_trip_preserves_millisecond_precision(self):
        original = StkTime(1_717_200_000.123456)
        ros_time = conv.stk_time_to_ros(original)
        back = conv.ros_time_to_stk(ros_time)
        assert back.epoch_seconds == pytest.approx(
            original.epoch_seconds, abs=1e-6)

    def test_large_epoch_does_not_lose_sub_second_precision(self):
        """Guards the float64-mantissa precision-loss failure mode
        described in conversions.stk_time_to_ros's docstring."""
        t1 = StkTime(1_900_000_000.001)
        t2 = StkTime(1_900_000_000.002)
        ros1, ros2 = conv.stk_time_to_ros(t1), conv.stk_time_to_ros(t2)
        assert (ros1.sec, ros1.nanosec) != (ros2.sec, ros2.nanosec)


class TestAngleRateScale:
    def test_default_is_unmeasured_identity(self):
        scale = conv.AngleRateScale()
        assert scale.measured is False
        assert scale.to_radps(1.0) == 1.0

    def test_degrees_per_second_conversion(self):
        scale = conv.AngleRateScale.degrees_per_second()
        assert scale.to_radps(180.0) == pytest.approx(math.pi)

    def test_round_trip(self):
        scale = conv.AngleRateScale.degrees_per_second()
        assert scale.from_radps(scale.to_radps(2.5)) == pytest.approx(2.5)


class TestObjectPaths:
    def test_absolute_and_relative_forms_unify(self):
        absolute = "/Application/STK/Scenario/Scn/Satellite/SSS-1"
        relative = "*/Satellite/SSS-1"
        assert conv.canonical_object_path(absolute) == \
            conv.canonical_object_path(relative)

    def test_object_name_and_class(self):
        path = "*/Satellite/SSS-1"
        assert conv.object_name(path) == "SSS-1"
        assert conv.object_class(path) == "Satellite"

    def test_child_path_construction(self):
        assert conv.child_path("*/Satellite/SSS-1", "Sensor", "StarTracker1") \
            == "*/Satellite/SSS-1/Sensor/StarTracker1"


class TestNumericHelpers:
    def test_as_float_passthrough(self):
        assert conv.as_float(3.5) == 3.5
        assert conv.as_float("3.5") == 3.5

    def test_as_float_default_on_garbage(self):
        assert conv.as_float("N/A", default=0.0) == 0.0

    def test_as_float_raises_without_default(self):
        with pytest.raises(ValueError):
            conv.as_float("garbage")

    def test_vector3_requires_three_components(self):
        with pytest.raises(ValueError):
            conv.vector3([1.0, 2.0])
