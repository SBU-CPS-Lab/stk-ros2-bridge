"""Unit tests for core/frames.py.

Includes regression tests for the J2000 issue found during Phase 3
bring-up: STK 12.6 has no "J2000" Cartesian Position group member.
"""

import pytest

from stk_bridge.core.frames import FrameRegistry
from stk_bridge.exceptions import FrameResolutionError


class TestFrameRegistry:
    def test_default_root_is_icrf(self):
        assert FrameRegistry().root_frame() == "icrf"

    def test_resolve_by_ros_id_and_stk_name(self):
        registry = FrameRegistry()
        assert registry.resolve("icrf").stk_name == "ICRF"
        assert registry.resolve("ICRF").ros_id == "icrf"   # case-insensitive
        assert registry.resolve("Fixed").ros_id == "fixed"

    def test_unknown_frame_raises(self):
        with pytest.raises(FrameResolutionError):
            FrameRegistry().resolve("nonexistent_frame")

    def test_j2000_resolves_via_alias_not_as_a_real_frame(self):
        """Regression test: STK 12.6's Cartesian Position groups are
        TrueOfDateRotating/Fixed/ICRF/MeanOfDate/MeanOfEpoch/TrueOfDate
        -- there is no "J2000" member. j2000 must still resolve (people
        will ask for it) but as an alias, and its STK name must be one
        STK 12.6 actually offers.
        """
        registry = FrameRegistry()
        frame = registry.resolve("j2000")
        assert frame.stk_name in (
            "MeanOfEpoch", "MeanOfDate", "TrueOfDate", "ICRF",
        ), f"j2000 alias resolves to {frame.stk_name!r}, not a real 12.6 frame"

    def test_confirmed_12_6_frames_all_resolve(self):
        """Every group member seen in a live STK 12.6
        'Cartesian Position' inventory must resolve to something."""
        registry = FrameRegistry()
        for stk_name in ("TrueOfDateRotating", "Fixed", "ICRF",
                         "MeanOfDate", "MeanOfEpoch", "TrueOfDate"):
            frame = registry.resolve(stk_name)
            assert frame.stk_name == stk_name


class TestAttitudeReferenceValidation:
    def test_fixed_and_icrf_are_valid_attitude_references(self):
        registry = FrameRegistry()
        registry.require_attitude_reference("fixed")
        registry.require_attitude_reference("icrf")

    def test_lvlh_is_rejected_as_attitude_reference(self):
        """AddAttitude only accepts CBFQuat (fixed) or Quat (inertial).
        LVLH must be rejected before a command is ever built, not
        silently misinterpreted as one of the two."""
        registry = FrameRegistry()
        with pytest.raises(FrameResolutionError):
            registry.require_attitude_reference("lvlh")

    def test_is_inertial_attitude_reference(self):
        registry = FrameRegistry()
        assert registry.is_inertial_attitude_reference("icrf") is True
        assert registry.is_inertial_attitude_reference("fixed") is False


class TestAliases:
    def test_add_alias(self):
        registry = FrameRegistry()
        registry.add_alias("map", "icrf")
        assert registry.resolve("map").ros_id == "icrf"

    def test_alias_cannot_shadow_a_real_frame(self):
        registry = FrameRegistry()
        with pytest.raises(ValueError):
            registry.add_alias("icrf", "fixed")

    def test_alias_target_must_exist(self):
        registry = FrameRegistry()
        with pytest.raises(FrameResolutionError):
            registry.add_alias("my_alias", "no_such_frame")


class TestVehicleFrames:
    def test_vehicle_frame_id_sanitises_object_name(self):
        registry = FrameRegistry()
        frame_id = registry.vehicle_frame_id("*/Satellite/SSS-1", "body")
        assert frame_id == "SSS_1/body"
        assert " " not in frame_id and "*" not in frame_id


class TestFromConfig:
    def test_empty_config_uses_builtins(self):
        registry = FrameRegistry.from_config({})
        assert registry.root_frame() == "icrf"

    def test_config_aliases_applied(self):
        registry = FrameRegistry.from_config(
            {"frames": {"aliases": {"map": "icrf"}}})
        assert registry.resolve("map").ros_id == "icrf"
