"""The moving-geometry fixtures keep testing what they exist to test.

Each ``moving_twin_step`` case targets one path that fixed-geometry
fixtures cannot reach. If a control-design change quietly turned a case
into an ordinary one (no replan, no motion), the MATLAB cross-check would
still pass and stop meaning anything -- so the property each case exists
for is asserted here, on the generated fixtures themselves.
"""

import math

import pytest

from acav.spec import AcavSpec
from acav.twin_fixtures import build_fixtures, moving_geometry


@pytest.fixture(scope="module")
def cases():
    fx = build_fixtures(AcavSpec.load(None))
    return {c["name"]: c for c in fx["moving_twin_step"]}


def _switch_s(case):
    return case["schedule"][-1]["start_s"]


def _modes(case):
    return [(t["time_s"], t["mode"]) for t in case["mode_transitions"]]


def test_the_geometry_actually_moves():
    _, _, start = moving_geometry(0.0, "TA")
    _, _, later = moving_geometry(30.0, "TA")
    assert math.dist(start, later) > 10_000.0      # metres, Earth rotation


def test_a_moving_target_is_tracked_not_lagged(cases):
    # Without the desired-rate feedforward and the transported profile,
    # tracking a moving line of sight leaves a standing error.
    final = cases["track_moving_target"]["checkpoints"][-1]
    assert final["mode"] == "IMAGE"
    assert math.degrees(final["boresight_error_rad"]) < 0.05


def test_retarget_mid_slew_replans_without_leaving_slew(cases):
    case = cases["retarget_mid_slew"]
    assert case["slew_starts_s"] == pytest.approx([0.0, _switch_s(case)])
    first_settle = next(t for t, m in _modes(case) if m == "SETTLE")
    assert first_settle > _switch_s(case)


def test_retarget_as_the_slew_ends_replans_on_that_step(cases):
    # The bug this section found: the change was consumed on the step the
    # old profile ended, and feedback alone closed the new slew.
    case = cases["retarget_as_slew_ends"]
    assert case["slew_starts_s"] == pytest.approx([0.0, _switch_s(case)])


def test_retarget_in_settle_goes_back_to_slew(cases):
    case = cases["retarget_in_settle"]
    modes = _modes(case)
    switch = _switch_s(case)
    assert (pytest.approx(switch), "SLEW") in [(t, m) for t, m in modes]
    before = [m for t, m in modes if t < switch]
    assert before[-1] == "SETTLE"


def test_losing_the_target_returns_to_nadir_without_a_slew(cases):
    case = cases["target_lost_to_nadir"]
    assert case["schedule"][-1]["target_id"] == ""
    assert case["slew_starts_s"] == [0.0]
    final = case["checkpoints"][-1]
    assert math.degrees(final["boresight_error_rad"]) < 0.5
