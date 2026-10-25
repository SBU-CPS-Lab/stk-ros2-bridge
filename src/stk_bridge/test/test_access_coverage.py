"""ComputeAccess waits until the attitude covers the requested stop time.

Found on a free-run run at rtf 1.0: T2's access was
computed when the newest attitude written for the satellite was stamped
109.959 s, before any sample at or after its 110.0 s image epoch, and
STK reported a near-nadir attitude there (no access, with a 0.0018 deg
pointing error). Computing access over an epoch the twin's attitude
does not yet reach measures a race in the harness, not the coupling.

The bridge checks, on its COM worker (so in order with the writes),
that the satellite's newest written attitude is stamped at or after the
request's stop time; if not it answers success=False with a fixed
message the scheduler recognises and retries. An object the bridge has
never written attitude for is not checked (unchanged behaviour).
"""

import sys
import types
from types import SimpleNamespace

import pytest

from stk_bridge.backends.base import StkTime
from stk_bridge.core.actuation import (
    ATTITUDE_NOT_COVERED,
    attitude_coverage_gap,
)

EPOCH = 1717237800.0
SENSOR = "*/Satellite/SSS1/Sensor/Camera"
SATELLITE = "*/Satellite/SSS1"


class _Actuation:
    def __init__(self, written=None):
        self.written = written
        self.asked = []

    def last_attitude(self, path):
        self.asked.append(path)
        if self.written is None:
            return None
        return (self.written, (0.0, 0.0, 0.0, 1.0), True)


class TestCoverageGap:
    def test_an_object_never_written_is_not_checked(self):
        assert attitude_coverage_gap(_Actuation(None), SENSOR,
                                     StkTime(EPOCH + 110.0)) == ""

    def test_the_sensor_is_checked_through_its_satellite(self):
        actuation = _Actuation(EPOCH + 200.0)
        attitude_coverage_gap(actuation, SENSOR, StkTime(EPOCH + 110.0))
        assert actuation.asked == [SATELLITE]

    def test_an_attitude_short_of_the_stop_time_is_a_gap(self):
        gap = attitude_coverage_gap(_Actuation(EPOCH + 109.959), SENSOR,
                                    StkTime(EPOCH + 110.0))
        assert gap.startswith(ATTITUDE_NOT_COVERED)
        assert "0.041 s short" in gap

    @pytest.mark.parametrize("written", [110.0, 110.0 - 2.4e-7, 110.537])
    def test_at_or_after_the_stop_time_is_covered(self, written):
        """At the stop time itself counts, within the 1e-6 s the bridge
        uses for written-sample matches (epoch rounding is ~2.4e-7 s)."""
        assert attitude_coverage_gap(_Actuation(EPOCH + written), SENSOR,
                                     StkTime(EPOCH + 110.0)) == ""


# -- the service handler, without ROS ----------------------------------

@pytest.fixture
def fake_msgs(monkeypatch):
    """Minimal stand-ins for the generated message modules."""
    msg = types.ModuleType("stk_msgs.msg")

    class AccessIntervalArray:
        def __init__(self):
            self.intervals = []
            self.from_object_path = ""
            self.to_object_path = ""

    msg.AccessInterval = lambda: SimpleNamespace()
    msg.AccessIntervalArray = AccessIntervalArray
    package = types.ModuleType("stk_msgs")
    package.msg = msg
    monkeypatch.setitem(sys.modules, "stk_msgs", package)
    monkeypatch.setitem(sys.modules, "stk_msgs.msg", msg)
    builtin = types.ModuleType("builtin_interfaces")
    builtin_msg = types.ModuleType("builtin_interfaces.msg")
    builtin_msg.Time = lambda sec, nanosec: SimpleNamespace(
        sec=sec, nanosec=nanosec)
    builtin.msg = builtin_msg
    monkeypatch.setitem(sys.modules, "builtin_interfaces", builtin)
    monkeypatch.setitem(sys.modules, "builtin_interfaces.msg", builtin_msg)


def _servers(written):
    from stk_bridge.interfaces.ros_services import ServiceServers

    servers = ServiceServers.__new__(ServiceServers)
    backend = SimpleNamespace(calls=[])

    def compute_access(from_path, to_path, start, stop):
        backend.calls.append((from_path, to_path, start, stop))
        return []

    backend.compute_access = compute_access
    servers._worker = SimpleNamespace(
        call=lambda function, description, timeout_s=None: function(backend))
    servers._ctx = SimpleNamespace(actuation=_Actuation(written))
    return servers, backend


def _request(stop_s):
    def stamp(seconds):
        whole = int(seconds)
        return SimpleNamespace(sec=whole,
                               nanosec=int(round((seconds - whole) * 1e9)))
    return SimpleNamespace(from_object_path=SENSOR,
                           to_object_path="*/Target/T2",
                           start_time=stamp(EPOCH + stop_s - 2.0),
                           stop_time=stamp(EPOCH + stop_s))


class TestService:
    def test_not_covered_answers_without_computing(self, fake_msgs):
        servers, backend = _servers(EPOCH + 109.959)
        response = servers._compute_access(_request(110.0),
                                           SimpleNamespace())
        assert response.success is False
        assert response.message.startswith(ATTITUDE_NOT_COVERED)
        assert backend.calls == []

    def test_covered_computes_once(self, fake_msgs):
        servers, backend = _servers(EPOCH + 110.2)
        response = servers._compute_access(_request(110.0),
                                           SimpleNamespace())
        assert response.success is True
        assert len(backend.calls) == 1

    def test_never_written_computes_as_before(self, fake_msgs):
        servers, backend = _servers(None)
        response = servers._compute_access(_request(110.0),
                                           SimpleNamespace())
        assert response.success is True
        assert len(backend.calls) == 1
