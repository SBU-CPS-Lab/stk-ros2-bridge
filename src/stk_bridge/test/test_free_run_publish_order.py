"""In free-run, ScenarioTime is published after the cycle's sensing.

The twin steps only when the attitude, orbit and rate stamps it holds
equal the latest ScenarioTime stamp (build_acav_twin.m, UnpackTelemetry).
Publishing ScenarioTime before sensing left a window of a whole sensing
time per cycle in which the twin held a new clock and old telemetry and
could not step; at rtf 1.0, with 56 % of cycles overrunning, it answered only 31 % of the bridge's samples. The clock is now
published after sensing with the same ``now``, so a cycle's messages
match as soon as they are out.

Lockstep is unchanged (test_lockstep_trace_golden.py pins its order),
and so is the free-run start gate, which senses once and republishes at
t = 0.
"""

from types import SimpleNamespace

from stk_bridge.backends.base import StkTime
from stk_bridge.core.rate_group import RateGroupScheduler
from stk_bridge.nodes.bridge_node import StkBridgeNode

EPOCH = 1717237800.0


class _Null:
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


def _node():
    sim = {"t": 0.0}
    node = StkBridgeNode.__new__(StkBridgeNode)
    node.config = {"time": {}}
    node.time_sync = SimpleNamespace(
        now=lambda: StkTime(EPOCH + sim["t"]),
        sim_seconds=lambda: sim["t"],
        sleep_until_next=lambda due: None, step_index=0)
    node.scheduler = RateGroupScheduler.from_config({"sensing": {
        "rate_groups": {"fast": {"rate_hz": 10.0}}}})
    node._timings = SimpleNamespace(timed=lambda name: _Null())
    node.calls = []

    def record(name):
        def call(*args, **kwargs):
            node.calls.append((name, args[0] if args else None))
            if name == "sense":
                sim["t"] += 0.12          # sensing takes wall time
        return call

    node._apply_commands = record("apply")
    node._publish_clock = record("clock")
    node._sense_and_publish = record("sense")
    node._publish_status = record("status")
    node._sensing_summary_checkpoint = lambda *a, **k: None
    return node, sim


def test_the_clock_is_published_after_sensing():
    node, _ = _node()
    node._free_running_iteration()
    names = [name for name, _ in node.calls]
    assert names == ["apply", "sense", "clock", "status"]


def test_the_clock_carries_the_sensing_epoch():
    node, _ = _node()
    for _ in range(3):
        node.calls.clear()
        node._free_running_iteration()
        stamps = dict(node.calls)
        assert stamps["clock"] is stamps["sense"]     # the same `now`
        assert stamps["status"] is stamps["sense"]


def test_the_start_gate_is_unchanged():
    """The gate still publishes the held clock first, then senses."""
    node, _ = _node()
    node._free_gate = {"published": False, "last_publish": 0.0,
                       "started": 0.0}
    node.time_sync.missing_acks = lambda: ["matlab_twin"]
    node._last_startup_log = float("inf")
    import stk_bridge.nodes.bridge_node as module
    sleep, module.time.sleep = module.time.sleep, lambda s: None
    try:
        node._free_running_iteration()
    finally:
        module.time.sleep = sleep
    assert [name for name, _ in node.calls] == ["clock", "sense", "status"]
