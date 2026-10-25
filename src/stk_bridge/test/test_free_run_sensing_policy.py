"""Free-run: the combined read is off, and the pacing summary is written.

The combined read verifies only at an epoch where the bridge wrote an
inertial sample, which a free-run sense epoch never is, so it stayed
"pending" for the whole run. A free-run bridge now turns it off at
startup with one INFO line, even if its config enables it (the runner
already writes it off); lockstep is untouched. The free-run pacing
summary is written to the file the config names (free_run_pacing_file)
whenever the summaries are logged.
"""

import json
import logging
from types import SimpleNamespace

from stk_bridge.core.free_run_pacer import FreeRunPacer
from stk_bridge.core.time_sync import ExecutionMode
from stk_bridge.nodes.bridge_node import StkBridgeNode


class FakeSensing:
    def __init__(self, state="pending"):
        self.state = state
        self.reasons = []

    @property
    def combine_state(self):
        return self.state

    def disable_combine(self, reason):
        self.reasons.append(reason)
        self.state = "off"

    def combine_summary_line(self):
        return "combined attitude/rate summary: x"

    def orbit_batch_summary_line(self):
        return "orbit batch summary: y"


def _node(mode, state="pending"):
    node = StkBridgeNode.__new__(StkBridgeNode)
    node.time_sync = SimpleNamespace(mode=mode)
    node.sensing = FakeSensing(state)
    return node


def test_free_run_turns_the_combined_read_off_with_one_info_line(caplog):
    node = _node(ExecutionMode.FREE_RUN)
    with caplog.at_level(logging.INFO):
        node._apply_free_run_sensing_policy()
    assert node.sensing.state == "off"
    lines = [r for r in caplog.records
             if "combined attitude/rate read is off in free-run" in
             r.getMessage()]
    assert len(lines) == 1 and lines[0].levelno == logging.INFO


def test_free_run_with_b_already_off_logs_nothing(caplog):
    node = _node(ExecutionMode.FREE_RUN, state="off")
    with caplog.at_level(logging.INFO):
        node._apply_free_run_sensing_policy()
    assert node.sensing.reasons == []
    assert "(B) is off in free-run" not in caplog.text


def test_lockstep_is_untouched():
    node = _node(ExecutionMode.LOCKSTEP)
    node._apply_free_run_sensing_policy()
    assert node.sensing.state == "pending" and node.sensing.reasons == []


def test_the_pipeline_disables_b():
    from stk_bridge.core.frames import FrameRegistry
    from stk_bridge.core.sensing import SensingConfig, SensingPipeline

    from test_equivalence_diagnostic import SAT, FakeBackend, _report

    pipeline = SensingPipeline(
        FakeBackend(), FrameRegistry(),
        SensingConfig(satellites=[SAT], combine_attitude_rate=True),
        _report())
    assert pipeline.combine_state == "pending"
    pipeline.disable_combine("free-run")
    assert pipeline.combine_state == "off"
    assert "state off" in pipeline.combine_summary_line()


def test_the_pacing_summary_is_written(tmp_path):
    node = _node(ExecutionMode.FREE_RUN)
    node.config = {"free_run_pacing_file": str(tmp_path / "pacing.json")}
    node._free_pacer = FreeRunPacer(0.1)
    node._free_pacer.begin(0.0)
    node._free_pacer.end(0.25)
    node._log_sensing_summaries(final=False)
    assert json.loads((tmp_path / "pacing.json").read_text(
        encoding="utf-8")) == {"cycles": 1, "overruns": 1, "period_s": 0.1}


def test_no_pacing_file_without_a_pacer(tmp_path):
    node = _node(ExecutionMode.LOCKSTEP)
    node.config = {"free_run_pacing_file": str(tmp_path / "pacing.json")}
    node._log_sensing_summaries(final=True)
    assert not (tmp_path / "pacing.json").exists()
