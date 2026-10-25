import math
import pytest
import acav.twin_reference as tr
from acav.spec import AcavSpec


def test_moving_profile_endpoint_and_rate_match_numerical_derivative():
    cfg = tr.TwinConfig.from_spec(AcavSpec.load())
    q0 = (0, 0, 0, 1)
    finish = tr.quat_from_axis_angle((1, 0, 0), .3)
    profile = tr.SlewProfile.between(q0, finish, cfg.profile_limits())
    rate = .002
    actual = tr.quat_from_axis_angle((0, 1, 0), .4)
    def desired(t):
        return tr.quat_multiply(tr.quat_from_axis_angle((0, 0, 1), rate*t), finish)
    feed = tr.rotate_to_body(actual, (0, 0, rate))
    t = profile.duration_s * .4
    q, omega = profile.tracking_reference(t, desired(t), actual, feed)
    h = 1e-5
    qnext, _ = profile.tracking_reference(t+h, desired(t+h), actual, feed)
    axis, angle = tr.axis_angle_from_quat(tr.quat_multiply(qnext, tr.quat_conjugate(q)))
    numerical = tr.rotate_to_body(actual, tr.scale(axis, angle/h))
    assert omega == pytest.approx(numerical, abs=2e-7)
    end, endrate = profile.tracking_reference(profile.duration_s, desired(profile.duration_s), actual, feed)
    assert abs(sum(a*b for a,b in zip(end,desired(profile.duration_s)))) == pytest.approx(1)
    assert endrate == pytest.approx(feed)
