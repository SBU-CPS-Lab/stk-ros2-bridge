"""The scheduler retries access until the bridge says the attitude covers t_i.

The bridge answers a ComputeAccess whose stop time the satellite's
written attitude does not reach yet with success=False and a fixed
message (stk_bridge.core.actuation.ATTITUDE_NOT_COVERED, see
stk_bridge/test/test_access_coverage.py). The scheduler then asks again
on the next tick, records how long each target waited (scenario and
wall seconds), and, if the attitude never covers t_i by the end of the
run, records ``access_unverified``: not reportable in any mode, like a
missing access answer.

Lockstep writes the sample stamped t_i before the scheduler's first
query at t_i + 0.2 s, so it is answered at once: the same requests at
the same epochs as before, with zero wait.
"""

import json
import math
import sys
import types
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from acav import scheduler_node as module
from acav.metrics import CaptureOutcome, RunResult, TargetResult
from acav.run_record import Provenance, RunRecord, read_record, write_record
from acav.scheduler_node import SchedulerNode
from stk_bridge.backends.base import StkTime
from stk_bridge.core.actuation import ATTITUDE_NOT_COVERED

EPOCH = 1717237800.0
TARGETS = {"T1": 60.0, "T2": 110.0, "T3": 127.0, "T4": 250.0, "T6": 400.0}


@pytest.fixture(autouse=True)
def fake_msgs(monkeypatch):
    srv = types.ModuleType("stk_msgs.srv")
    srv.ComputeAccess = SimpleNamespace(Request=SimpleNamespace)
    package = types.ModuleType("stk_msgs")
    package.srv = srv
    monkeypatch.setitem(sys.modules, "stk_msgs", package)
    monkeypatch.setitem(sys.modules, "stk_msgs.srv", srv)
    builtin = types.ModuleType("builtin_interfaces")
    builtin_msg = types.ModuleType("builtin_interfaces.msg")
    builtin_msg.Time = lambda sec, nanosec: SimpleNamespace(
        sec=sec, nanosec=nanosec)
    builtin.msg = builtin_msg
    monkeypatch.setitem(sys.modules, "builtin_interfaces", builtin)
    monkeypatch.setitem(sys.modules, "builtin_interfaces.msg", builtin_msg)


@pytest.fixture
def wall(monkeypatch):
    now = {"t": 5000.0}
    monkeypatch.setattr(module.time, "monotonic", lambda: now["t"])
    return now


class _Client:
    """Records each ComputeAccess request with the tick it was sent on."""

    def __init__(self):
        self.sent = []          # (target stop epoch, request, callback box)

    def wait_for_service(self, timeout_sec):
        return True

    def call_async(self, request):
        box = {}
        future = SimpleNamespace(
            add_done_callback=lambda cb: box.setdefault("cb", cb))
        self.sent.append((request, box))
        return future


def _answer(box, success=True, message="", covering=None):
    intervals = [] if covering is None else [SimpleNamespace(
        start_time=_stamp(EPOCH + covering - 1.0),
        stop_time=_stamp(EPOCH + covering + 1.0))]
    response = SimpleNamespace(success=success, message=message,
                               accesses=SimpleNamespace(intervals=intervals))
    box["cb"](SimpleNamespace(result=lambda: response))


def _stamp(seconds):
    whole = int(seconds)
    return SimpleNamespace(sec=whole,
                           nanosec=int(round((seconds - whole) * 1e9)))


def _entry(target_id, t_image_s):
    return SimpleNamespace(target_id=target_id, t_image_s=t_image_s,
                           target=SimpleNamespace(id=target_id,
                                                  t_image_s=t_image_s))


def _node(targets=TARGETS):
    node = SchedulerNode.__new__(SchedulerNode)
    node._epoch = StkTime(EPOCH)
    node._results = {}
    node._finalized = set()
    node._access_requested = set()
    node._init_access_state()
    node._spec = SimpleNamespace(
        window_duration_s=720.0,
        targets=[SimpleNamespace(id=i, t_image_s=t)
                 for i, t in targets.items()],
        pointing=SimpleNamespace(settle_hold_s=2.0),
        sensor=SimpleNamespace(object_path="*/Satellite/SSS1/Sensor/Cam"))
    node._plan = SimpleNamespace(
        attempted=[_entry(i, t) for i, t in targets.items()],
        scheduler_type=SimpleNamespace(value="dynamics_aware"))
    node._options = SimpleNamespace(
        run_id="run", seed=0, output_json="", auto_shutdown=False,
        access_query_delay_s=module.DEFAULT_ACCESS_QUERY_DELAY_S,
        prefix="stk")
    node._client_access = _Client()
    node._tracker = Mock()
    node._tracker.finalize.side_effect = lambda entry, has_access: (
        TargetResult(target_id=entry.target_id, attempted=True,
                     outcome=(CaptureOutcome.CAPTURED if has_access
                              else CaptureOutcome.NO_ACCESS),
                     image_time_s=entry.t_image_s, error_at_image_deg=0.002,
                     max_error_in_hold_deg=0.003))
    node._aggregate = Mock()
    node._latency = Mock()
    node._tick_intervals_ms = []
    node._shutdown_requested = False
    node._window_elapsed_wall = None
    return node


def _stop_s(request):
    return (request.stop_time.sec + request.stop_time.nanosec / 1e9) - EPOCH


class TestLockstepIsUnchanged:
    def test_same_requests_at_the_same_epochs_answered_at_once(self, wall):
        """Each target is asked about once, on the first 0.1 s tick at or
        after t_i + 0.2 s, over [t_i - 2, t_i] -- the rule before this
        change -- and waits zero."""
        node = _node()
        client = node._client_access
        sent_at = []
        for k in range(7201):
            tick = k * 0.1
            before = len(client.sent)
            node._check_access_due(tick)
            for request, box in client.sent[before:]:
                sent_at.append((round(_stop_s(request), 6), tick))
                _answer(box, covering=_stop_s(request))
        expected = []
        for t_image in TARGETS.values():
            first = next(k * 0.1 for k in range(7201) if k * 0.1 >= t_image
                         + module.DEFAULT_ACCESS_QUERY_DELAY_S)
            expected.append((t_image, first))
        assert sent_at == expected
        for target_id in TARGETS:
            result = node._results[target_id]
            assert result.outcome is CaptureOutcome.CAPTURED
            assert result.access_wait_s == 0.0
            assert result.access_wait_wall_s == 0.0


class TestFreeRunDeferral:
    def test_deferred_until_the_attitude_is_written(self, wall):
        node = _node({"T2": 110.0})
        client = node._client_access
        node._check_access_due(110.2)
        request, box = client.sent[-1]
        assert _stop_s(request) == pytest.approx(110.0)
        _answer(box, success=False,
                message=f"{ATTITUDE_NOT_COVERED}: newest 109.959")
        assert "T2" not in node._finalized
        assert "T2" not in node._results

        wall["t"] += 0.25
        node._check_access_due(110.45)              # asked again
        assert len(client.sent) == 2
        _answer(client.sent[-1][1], success=False,
                message=f"{ATTITUDE_NOT_COVERED}: newest 109.959")

        wall["t"] += 0.15
        node._check_access_due(110.6)
        _answer(client.sent[-1][1], covering=110.0)
        result = node._results["T2"]
        assert result.outcome is CaptureOutcome.CAPTURED
        assert result.access_wait_s == pytest.approx(0.4)
        assert result.access_wait_wall_s == pytest.approx(0.4)
        assert "T2" in node._finalized

    def test_immediate_when_already_written(self, wall):
        node = _node({"T2": 110.0})
        node._check_access_due(110.2)
        _answer(node._client_access.sent[-1][1], covering=110.0)
        result = node._results["T2"]
        assert result.outcome is CaptureOutcome.CAPTURED
        assert result.access_wait_s == 0.0

    def test_a_real_failure_is_still_an_error_and_retried(self, wall):
        node = _node({"T2": 110.0})
        node._check_access_due(110.2)
        _answer(node._client_access.sent[-1][1], success=False,
                message="QueryError: Access")
        node._check_access_due(110.3)
        assert len(node._client_access.sent) == 2
        node._finish_run(forced=True)
        assert node._results["T2"].outcome is (
            CaptureOutcome.NO_ACCESS_ANSWER)

    def test_never_covered_is_access_unverified(self, wall):
        node = _node({"T2": 110.0})
        node._check_access_due(110.2)
        _answer(node._client_access.sent[-1][1], success=False,
                message=f"{ATTITUDE_NOT_COVERED}: newest 109.959")
        wall["t"] += 3.0
        node._check_access_due(113.2)
        _answer(node._client_access.sent[-1][1], success=False,
                message=f"{ATTITUDE_NOT_COVERED}: newest 109.959")
        node._finish_run(forced=True)
        result = node._results["T2"]
        assert result.outcome is CaptureOutcome.ACCESS_UNVERIFIED
        assert result.attempted
        assert "109.959" in result.reason
        assert result.access_wait_s == pytest.approx(3.0)
        assert result.access_wait_wall_s == pytest.approx(3.0)


def _record(mode, outcome, waits=(float("nan"), float("nan"))):
    return RunRecord(
        result=RunResult(run_id="run", scheduler="dynamics_aware", targets=[
            TargetResult(target_id="T2", attempted=True, outcome=outcome,
                         image_time_s=110.0, access_wait_s=waits[0],
                         access_wait_wall_s=waits[1])]),
        provenance=Provenance(backend_type="com", execution_mode=mode))


class TestRecord:
    @pytest.mark.parametrize("mode", ["lockstep", "free_run"])
    def test_access_unverified_is_never_reportable(self, mode):
        record = _record(mode, CaptureOutcome.ACCESS_UNVERIFIED)
        assert not record.reportable
        assert any("T2" in r for r in record.unreportable_reasons)

    def test_waits_round_trip_and_stay_out_of_the_signature(self, tmp_path):
        waited = _record("free_run", CaptureOutcome.CAPTURED, (0.4, 0.38))
        path = tmp_path / "r.json"
        write_record(str(path), waited)
        target = read_record(str(path)).result.targets[0]
        assert target.access_wait_s == 0.4
        assert target.access_wait_wall_s == 0.38
        assert waited.signature == _record(
            "free_run", CaptureOutcome.CAPTURED).signature

    def test_unmeasured_waits_are_not_written(self, tmp_path):
        path = tmp_path / "r.json"
        write_record(str(path), _record("free_run", CaptureOutcome.CAPTURED))
        target = json.loads(path.read_text())["result"]["targets"][0]
        assert "access_wait_s" not in target
        back = read_record(str(path)).result.targets[0]
        assert math.isnan(back.access_wait_s)

    def test_the_outcome_has_a_label_and_colour(self):
        from acav.report_style import STATUS_COLOR, status_label

        assert status_label("access_unverified") == "Access unverified"
        assert "access_unverified" in STATUS_COLOR
