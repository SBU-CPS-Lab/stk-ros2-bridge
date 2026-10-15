"""Lockstep stays byte-identical across the free-run pacing fix.

Drives the real StkBridgeNode._lockstep_iteration with the real TimeSync
and RateGroupScheduler for 250 steps and records, per step, every stage
the iteration calls with the epoch it passes, plus the rate groups'
final state. The trace is compared with a golden file captured from the
code BEFORE the free-run pacing change (commit 48813ac): any difference
in step sequence, epochs, rate-group marking or sleeping fails.

Regenerate the golden file only for a deliberate lockstep change:
    python test_lockstep_trace_golden.py --write
"""

import json
import os
import sys
import threading
from contextlib import nullcontext
from types import SimpleNamespace

from stk_bridge.backends.base import StkTime
from stk_bridge.core.rate_group import RateGroupScheduler
from stk_bridge.core.time_sync import TimeSync, TimeSyncConfig
from stk_bridge.nodes.bridge_node import StkBridgeNode

GOLDEN = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      "fixtures", "lockstep_trace_48813ac.json")
EPOCH = 1717237800.0
STEPS = 250


def lockstep_trace(monkeypatch=None):
    node = StkBridgeNode.__new__(StkBridgeNode)
    node.config = {"time": {"auto_step": True}}
    node.time_sync = TimeSync(TimeSyncConfig.from_config({"time": {
        "mode": "lockstep", "step_s": 0.1,
        "barrier_participants": ["matlab_twin"]}}))
    node.time_sync.start(StkTime(EPOCH))
    node.time_sync.await_barrier = lambda timeout_s=None: []
    node.scheduler = RateGroupScheduler.from_config({"sensing": {
        "rate_groups": {"fast": {"rate_hz": 10.0},
                        "medium": {"rate_hz": 1.0},
                        "slow": {"rate_hz": 0.1}}}})
    node._running = threading.Event()
    node._running.set()
    node._step_request = threading.Event()
    node._step_lock = threading.Lock()
    node._step_size = 0.0
    node._advance_ms = (0.0, 0)
    node._last_startup_log = 0.0
    node._timings = SimpleNamespace(timed=lambda name: nullcontext(),
                                    record=lambda name, ms: None)
    node.health = SimpleNamespace(record_error=lambda message: None)
    events = []

    def recorder(name):
        def record(*args, **kwargs):
            when = args[0] if args else None
            events.append([name, repr(when.epoch_seconds)
                           if isinstance(when, StkTime) else None])
        return record

    for name in ("_apply_commands", "_publish_clock", "_sense_and_publish",
                 "_publish_status", "_sensing_summary_checkpoint",
                 "_await_attitude_commands"):
        setattr(node, name, recorder(name))
    if monkeypatch is not None:
        import time as time_module

        def no_sleep(seconds):
            raise AssertionError("lockstep must never sleep")

        monkeypatch.setattr(time_module, "sleep", no_sleep)
    for _ in range(STEPS):
        node._lockstep_iteration()
    groups = {group.name: [group.runs, group.overruns,
                           repr(group._last_run_s)]
              for group in node.scheduler._groups.values()}
    return {"steps": node.time_sync.step_index,
            "sim_seconds": repr(node.time_sync.sim_seconds()),
            "events": events, "groups": groups}


def test_lockstep_trace_matches_the_pre_change_golden(monkeypatch):
    with open(GOLDEN, encoding="utf-8") as handle:
        golden = json.load(handle)
    assert lockstep_trace(monkeypatch) == golden


if __name__ == "__main__" and "--write" in sys.argv:
    os.makedirs(os.path.dirname(GOLDEN), exist_ok=True)
    with open(GOLDEN, "w", encoding="utf-8", newline="\n") as handle:
        json.dump(lockstep_trace(), handle, indent=1)
        handle.write("\n")
    print("wrote", GOLDEN)
