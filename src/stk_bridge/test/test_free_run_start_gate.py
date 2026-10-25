"""Free-run: the clock waits at t = 0 until every participant has acked.

Free-run time follows the wall clock from bridge start, so before this
the window ran on through the MATLAB handoff and the scheduler's target
resolution (1-3 min), and each free-run run's outcome depended on how
long the operator took. With time.wait_for_startup_participants (as in
lockstep), a free-run bridge now starts its clock paused at exactly
t = 0, senses once, republishes that initial step every
STARTUP_REPUBLISH_S without COM work, and resumes from t = 0 once every
barrier participant (twin and scheduler) has acked; it logs the wall
time spent and writes it to the bridge config's startup_gate_file.
Lockstep is unchanged.
"""

import json
import logging
from types import SimpleNamespace

import pytest

from stk_bridge.backends.base import StkTime
from stk_bridge.core.time_sync import TimeSync, TimeSyncConfig
from stk_bridge.nodes import bridge_node as bridge_module
from stk_bridge.nodes.bridge_node import StkBridgeNode

EPOCH = 1717237800.0
PARTICIPANTS = ["acav_scheduler", "matlab_twin"]


def _time_sync(mode, wait=True):
    return TimeSync(TimeSyncConfig.from_config({"time": {
        "mode": mode, "step_s": 0.1, "barrier_participants": PARTICIPANTS,
        "wait_for_startup_participants": wait}}))


def _node(tmp_path, mode="free_run", wait=True):
    node = StkBridgeNode.__new__(StkBridgeNode)
    node.config = {"time": {"wait_for_startup_participants": wait},
                   "startup_gate_file": str(tmp_path / "gate.json")}
    node.time_sync = _time_sync(mode, wait)
    node.scheduler = SimpleNamespace(next_due_time=lambda now: 0.0,
                                     fastest_period_s=lambda: 0.1,
                                     run_due=lambda *a, **k: 0)
    node._timings = SimpleNamespace(timed=lambda name: _Null())
    node._last_startup_log = 0.0
    node.calls = []
    for name in ("_apply_commands", "_publish_clock", "_sense_and_publish",
                 "_run_rate_groups", "_publish_status",
                 "_sensing_summary_checkpoint"):
        setattr(node, name, (lambda n: lambda *a, **k: node.calls.append(
            (n, a[0].epoch_seconds - EPOCH if a and isinstance(a[0], StkTime)
             else None)))(name))
    node._arm_startup_gate(StkTime(EPOCH))
    return node


class _Null:
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


@pytest.fixture(autouse=True)
def fast(monkeypatch):
    monkeypatch.setattr(bridge_module, "STARTUP_REPUBLISH_S", 0.0)
    monkeypatch.setattr(bridge_module, "FREE_RUN_GATE_POLL_S", 0.0)


class TestTimeSync:
    def test_start_paused_holds_exactly_zero(self):
        ts = _time_sync("free_run")
        ts.start(StkTime(EPOCH), paused=True)
        assert ts.paused and ts.sim_seconds() == 0.0
        ts.resume()
        assert 0.0 <= ts.sim_seconds() < 1.0

    def test_missing_acks_in_free_run(self):
        ts = _time_sync("free_run")
        ts.start(StkTime(EPOCH), paused=True)
        assert ts.missing_acks() == PARTICIPANTS
        ts.acknowledge("matlab_twin", 0)
        assert ts.missing_acks() == ["acav_scheduler"]


class TestGate:
    def test_paused_until_both_participants_ack(self, tmp_path, caplog):
        node = _node(tmp_path)
        assert node.time_sync.paused
        with caplog.at_level(logging.INFO):
            node._free_running_iteration()
            node._free_running_iteration()
            node.time_sync.acknowledge("matlab_twin", 0)
            node._free_running_iteration()
        names = [name for name, _ in node.calls]
        assert names.count("_sense_and_publish") == 1     # once, at t = 0
        assert "_apply_commands" not in names
        assert all(t == 0.0 for _, t in node.calls if t is not None)
        assert node.time_sync.paused
        assert not (tmp_path / "gate.json").exists()

        node.time_sync.acknowledge("acav_scheduler", 0)
        with caplog.at_level(logging.INFO):
            node._free_running_iteration()
        assert not node.time_sync.paused
        assert "free-run start gate opened after" in caplog.text
        gate = json.loads((tmp_path / "gate.json").read_text(
            encoding="utf-8"))
        assert gate["opened"] is True and gate["wait_s"] >= 0.0
        assert gate["participants"] == PARTICIPANTS

        node.calls.clear()
        node._free_running_iteration()                 # an ordinary one
        names = [name for name, _ in node.calls]
        assert "_apply_commands" in names and "_sense_and_publish" in names
        times = [t for _, t in node.calls if t is not None]
        assert times and all(0.0 <= t < 1.0 for t in times)

    def test_the_initial_step_is_republished_without_com_work(self, tmp_path):
        node = _node(tmp_path)
        node._startup_samples = {"*/Satellite/SSS-1": ["sample"]}
        republished = []
        node.publishers = {"*/Satellite/SSS-1": SimpleNamespace(
            publish_samples=republished.append)}
        node._free_running_iteration()                 # senses at t = 0
        for _ in range(3):
            node._free_running_iteration()
        assert [n for n, _ in node.calls].count("_sense_and_publish") == 1
        assert len(republished) == 3

    def test_no_gate_without_the_setting(self, tmp_path):
        node = _node(tmp_path, wait=False)
        assert not node.time_sync.paused
        node._free_running_iteration()
        assert "_apply_commands" in [n for n, _ in node.calls]

    def test_lockstep_is_never_gated(self, tmp_path):
        node = _node(tmp_path, mode="lockstep")
        assert not node.time_sync.paused
        assert getattr(node, "_free_gate", None) is None
        assert node.time_sync.sim_seconds() == 0.0
