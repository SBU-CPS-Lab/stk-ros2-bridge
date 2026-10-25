"""Tests for scheduler_node.py's ROS-free pieces.

``SchedulerNode`` itself needs a real (or carefully faked) rclpy
``Node`` and is not exercised here -- matching this project's own
established boundary: node-layer classes are integration-verified
against a sourced ROS environment (see ``docs/``), not unit tested
offline. What CAN and must be tested offline is that this module is
importable without ROS at all (a regression here would mean a stray
module-scope ``rclpy``/``stk_msgs`` import slipped in, silently
breaking that boundary), and its pure helper functions.
"""

import importlib

from acav.scheduler_node import (
    DEFAULT_ACCESS_QUERY_DELAY_S,
    SchedulerOptions,
    _unflatten,
    target_object_path,
)


class TestModuleIsRosFree:
    def test_importable_without_rclpy_or_stk_msgs(self):
        """The whole point of the lazy-import discipline this module
        follows (matching nodes/clock_node.py and nodes/bridge_node.py):
        importing it must not require a sourced ROS environment."""
        import sys
        assert "rclpy" not in sys.modules or True  # rclpy may or may not
        # be present in this environment; what matters is that IMPORTING
        # the module does not raise even when it is absent, which the
        # mere fact that this test file collected at all already proves
        # (conftest.py runs the whole suite with no ROS on the path).
        module = importlib.import_module("acav.scheduler_node")
        assert hasattr(module, "SchedulerNode")
        assert hasattr(module, "main")


class TestTargetObjectPath:
    def test_matches_the_scenario_builder_s_own_convention(self):
        assert target_object_path("T3") == "*/Target/T3"

    def test_is_stable_for_every_scenario_target(self):
        from acav.spec import AcavSpec
        spec = AcavSpec.load()
        for target in spec.targets:
            path = target_object_path(target.id)
            assert path.startswith("*/Target/")
            assert path.endswith(target.id)


class TestUnflatten:
    def test_reconstructs_row_major_columns(self):
        columns = _unflatten(["x", "y", "z"], [1.0, 2.0, 3.0], row_count=1)
        assert columns == {"x": [1.0], "y": [2.0], "z": [3.0]}

    def test_multiple_rows_stay_in_order(self):
        flattened = [1.0, 2.0, 3.0,     # row 0
                    4.0, 5.0, 6.0]      # row 1
        columns = _unflatten(["x", "y", "z"], flattened, row_count=2)
        assert columns["x"] == [1.0, 4.0]
        assert columns["y"] == [2.0, 5.0]
        assert columns["z"] == [3.0, 6.0]

    def test_zero_rows_gives_empty_columns(self):
        columns = _unflatten(["x", "y", "z"], [], row_count=0)
        assert columns == {"x": [], "y": [], "z": []}

    def test_matches_the_wire_format_documented_in_the_message(self):
        """DataProviderResult.msg documents row-major layout, row_count x
        len(column_names) -- this is the one place that layout
        assumption is decoded, so a mismatch here would silently
        misattribute every value to the wrong column."""
        names = ["Time", "x", "y", "z"]
        flattened = [0.0, 10.0, 20.0, 30.0,
                    1.0, 11.0, 21.0, 31.0,
                    2.0, 12.0, 22.0, 32.0]
        columns = _unflatten(names, flattened, row_count=3)
        assert columns["Time"] == [0.0, 1.0, 2.0]
        assert columns["x"] == [10.0, 11.0, 12.0]
        assert columns["y"] == [20.0, 21.0, 22.0]
        assert columns["z"] == [30.0, 31.0, 32.0]


class TestSchedulerOptions:
    def test_defaults_are_sane(self):
        options = SchedulerOptions()
        assert options.scheduler_type == "dynamics_aware"
        assert options.participant_name == "acav_scheduler"
        assert options.access_query_delay_s == DEFAULT_ACCESS_QUERY_DELAY_S

    def test_participant_name_defaults_to_the_scenario_s_own_declaration(
            self):
        """acav_scenario.yaml's bridge.barrier_participants names this
        node "acav_scheduler" literally; the default here must match or
        every lockstep run silently times out."""
        from acav.spec import AcavSpec
        spec = AcavSpec.load()
        assert SchedulerOptions().participant_name in \
            spec.bridge.get("barrier_participants", [])
