"""Free-run iterations take the same summary and graphics checkpoints.

Only the lockstep iteration called _sensing_summary_checkpoint, so a
free-run bridge (E3) never logged its final summaries and never took
the terminal graphics sample: every free-run record would come out
"unverified" and fail stk_graphics_end. Free-run time comes from the
wall clock and step_index stays 0, so the free-run loop counts its own
iterations: running checkpoints every SUMMARY_EVERY_STEPS iterations,
the final ones once when the time reaches the scenario stop.
"""

import json
from types import SimpleNamespace

from stk_bridge.backends.base import StkTime
from stk_bridge.core import graphics_state as gs
from stk_bridge.nodes.bridge_node import SUMMARY_EVERY_STEPS, StkBridgeNode

EPOCH = 1717237800.0
OFF = {"windows_2d": 0, "windows_3d": 0, "main_window": "minimized",
       "visible": True}


def _free_node(tmp_path, times):
    node = StkBridgeNode.__new__(StkBridgeNode)
    node.config = {}
    clock = iter(times)
    node.time_sync = SimpleNamespace(
        step_index=0, now=lambda: StkTime(next(clock)),
        sim_seconds=lambda: 0.0, sleep_until_next=lambda due: None)
    node.scheduler = SimpleNamespace(next_due_time=lambda now: 0.0,
                                     fastest_period_s=lambda: 0.1,
                                     run_due=lambda *a, **k: 0)
    node._timings = SimpleNamespace(timed=lambda name: _Null())
    for name in ("_apply_commands", "_publish_clock", "_sense_and_publish",
                 "_run_rate_groups", "_publish_status"):
        setattr(node, name, lambda *a: None)
    node.sensing = SimpleNamespace(
        combine_summary_line=lambda: "combined attitude/rate summary: x",
        orbit_batch_summary_line=lambda: "orbit batch summary: y")
    node._scenario_stop = StkTime(EPOCH + 720.0)
    backend = SimpleNamespace(graphics_state=lambda: OFF)
    node.worker = SimpleNamespace(call=lambda fn, description, **k:
                                  fn(backend))
    node._graphics_log = gs.GraphicsLog(str(tmp_path / "g.json"))
    return node


class _Null:
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


def test_free_run_takes_running_and_final_checkpoints(tmp_path, caplog):
    import logging
    # 2500 iterations at irregular times, crossing the stop near the end.
    times = [EPOCH + 0.29 * i for i in range(1, 2501)]
    node = _free_node(tmp_path, times)
    node._sample_graphics("startup")          # as start() does
    with caplog.at_level(logging.INFO):
        for _ in times:
            node._free_running_iteration()
    data = json.loads((tmp_path / "g.json").read_text(encoding="utf-8"))
    assert [s["step"] for s in data["samples"]] == [1000, 2000]
    final_at = next(i for i, t in enumerate(times, 1)
                    if t >= EPOCH + 720.0)
    assert data["end"]["step"] == final_at
    assert gs.summarize(data) == "off"
    finals = [r.getMessage() for r in caplog.records
              if r.getMessage().startswith("combined attitude/rate summary")]
    assert len(finals) == 1                      # once, at the stop


def test_lockstep_checkpoints_still_follow_the_step_index(tmp_path):
    node = _free_node(tmp_path, [])
    node.time_sync.step_index = SUMMARY_EVERY_STEPS
    node._sensing_summary_checkpoint(StkTime(EPOCH + 100.0))
    data = json.loads((tmp_path / "g.json").read_text(encoding="utf-8"))
    assert [s["step"] for s in data["samples"]] == [SUMMARY_EVERY_STEPS]
