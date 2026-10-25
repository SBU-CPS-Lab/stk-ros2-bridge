"""Where the bridge's startup time goes, recorded as it happens.

A slow startup can make the runner give up with "no BridgeStatus received
at all" while bridge.log stops inside convention validation. Launch to
CONNECTED takes 43.7-76.1 s, so the log needs per-phase durations rather
than timestamps from which the phases must be rebuilt.

The bridge times each startup phase -- connect, each step of
scenario setup, each convention-validation check, sensing, interfaces --
logs each one's duration, and writes them to startup_phases_file as
they happen, so a run killed part-way through still says which phase it
was in and for how long. It also publishes BridgeStatus STARTING when
startup begins and at every phase change, so the runner sees a live
bridge before CONNECTED.
"""

import json
import logging

import pytest

from stk_bridge.backends.mock_backend import MockBackend
from stk_bridge.core.frames import FrameRegistry
from stk_bridge.core.health import BridgeState
from stk_bridge.core.scenario_manager import ScenarioConfig, ScenarioManager
from stk_bridge.core.startup_phases import StartupPhases


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


def _read(path):
    return json.loads(path.read_text(encoding="utf-8"))


def test_each_phase_is_written_as_it_starts_and_ends(tmp_path):
    path = tmp_path / "bridge_startup_phases.json"
    clock = Clock()
    phases = StartupPhases(str(path), clock=clock)
    with phases.phase("connect"):
        state = _read(path)
        assert state["current"] == "connect"
        assert state["finished"] is False
        clock.now += 1.5
    state = _read(path)
    assert state["current"] == ""
    assert state["phases"][0]["phase"] == "connect"
    assert state["phases"][0]["s"] == pytest.approx(1.5)
    assert state["phases"][0]["ok"] is True


def test_nested_phases_name_the_innermost_as_current(tmp_path):
    path = tmp_path / "phases.json"
    clock = Clock()
    phases = StartupPhases(str(path), clock=clock)
    with phases.phase("validation"):
        with phases.phase("validation/quaternion_ordering"):
            assert _read(path)["current"] == \
                "validation/quaternion_ordering"
            clock.now += 2.0
        assert _read(path)["current"] == "validation"
        clock.now += 1.0
    durations = phases.durations()
    assert durations == {"validation/quaternion_ordering": 2.0,
                         "validation": 3.0}


def test_a_phase_that_raises_is_recorded_failed_and_the_error_kept(
        tmp_path):
    path = tmp_path / "phases.json"
    phases = StartupPhases(str(path), clock=Clock())
    with pytest.raises(RuntimeError, match="COM stalled"):
        with phases.phase("validation"):
            raise RuntimeError("COM stalled")
    phases.finish("failed", "RuntimeError: COM stalled")
    state = _read(path)
    assert state["phases"][0]["ok"] is False
    assert state["finished"] is True
    assert state["outcome"] == "failed"
    assert state["error"] == "RuntimeError: COM stalled"
    assert state["last_phase"] == "validation"


def test_each_duration_is_logged(caplog):
    clock = Clock()
    phases = StartupPhases("", clock=clock)
    with caplog.at_level(logging.INFO):
        with phases.phase("frames"):
            clock.now += 19.5
    assert "startup phase frames: 19.5 s" in caplog.text


def test_with_no_file_nothing_is_written(tmp_path):
    phases = StartupPhases("", clock=Clock())
    with phases.phase("connect"):
        pass
    phases.finish("connected")
    assert list(tmp_path.iterdir()) == []


def test_every_change_is_announced():
    seen = []
    phases = StartupPhases("", clock=Clock(), on_change=seen.append)
    with phases.phase("connect"):
        pass
    assert seen == ["connect", ""]


def test_an_announcement_that_fails_does_not_stop_startup():
    def broken(current):
        raise RuntimeError("publisher gone")

    phases = StartupPhases("", clock=Clock(), on_change=broken)
    with phases.phase("connect"):
        pass
    assert phases.durations() == {"connect": 0.0}


# --- the scenario manager and the validator report their phases -----------

SETUP_PHASES = ["capabilities", "open_scenario", "epoch", "objects",
                "frames", "validation", "realtime_objects"]


def test_scenario_setup_reports_each_step_and_each_validation_check():
    backend = MockBackend()
    backend.connect()
    try:
        backend.new_scenario("ACAV_SSS1")
        phases = StartupPhases("", clock=Clock())
        ScenarioManager(backend, FrameRegistry(), ScenarioConfig(
            use_existing=True, scenario_name="ACAV_SSS1")).setup(
            run_validation=True, phases=phases)
    finally:
        backend.disconnect()
    names = [entry["phase"] for entry in phases.to_dict()["phases"]]
    top = [name for name in names if "/" not in name]
    assert top == SETUP_PHASES
    checks = [name for name in names if name.startswith("validation/")]
    assert checks[0] == "validation/setup_object"
    assert "validation/quaternion_ordering" in checks
    assert "validation/attitude_history_clears" in checks
    assert checks[-1] == "validation/teardown_object"


def test_scenario_setup_without_phases_is_unchanged():
    backend = MockBackend()
    backend.connect()
    try:
        backend.new_scenario("ACAV_SSS1")
        report = ScenarioManager(backend, FrameRegistry(), ScenarioConfig(
            use_existing=True, scenario_name="ACAV_SSS1")).setup(
            run_validation=True)
    finally:
        backend.disconnect()
    assert report.ok


# --- the bridge node: phases file and STARTING status ----------------------

class FakePublishers:
    def __init__(self, sink):
        self.sink = sink

    def publish_status(self, **fields):
        self.sink.append(fields)


def _start_bridge(tmp_path, monkeypatch, published, built):
    from stk_bridge.interfaces import qos as qos_module
    from stk_bridge.nodes import bridge_node

    monkeypatch.setattr(qos_module, "from_config", lambda config: {})

    def make_publishers(self, qos_profiles):
        built.append(qos_profiles)
        return FakePublishers(published)

    monkeypatch.setattr(bridge_node.StkBridgeNode,
                        "_make_bridge_publishers", make_publishers)
    monkeypatch.setattr(bridge_node.StkBridgeNode, "_build_interfaces",
                        lambda self, qos_profiles: None)
    monkeypatch.setattr(bridge_node.StkBridgeNode, "_arm_startup_gate",
                        lambda self, epoch: None)
    path = tmp_path / "bridge_startup_phases.json"
    config = {"backend": {"type": "mock",
                          "mock": {"satellite_name": "SSS-1"}},
              "scenario": {"use_existing": False,
                           "scenario_name": "ACAV_SSS1"},
              "sensing": {"satellites": ["*/Satellite/SSS-1"]},
              "startup_phases_file": str(path)}
    bridge = bridge_node.StkBridgeNode(node=None, config=config)
    bridge._loop = lambda: None
    return bridge, path


def test_the_bridge_writes_its_startup_phases(tmp_path, monkeypatch):
    published, built = [], []
    bridge, path = _start_bridge(tmp_path, monkeypatch, published, built)
    try:
        bridge.start()
    finally:
        bridge.stop()
    state = _read(path)
    assert state["finished"] is True and state["outcome"] == "connected"
    top = [entry["phase"] for entry in state["phases"]
           if "/" not in entry["phase"]]
    assert top == (["connect"] + SETUP_PHASES
                   + ["sensing", "interfaces"])
    assert all(entry["ok"] for entry in state["phases"])


def test_the_bridge_publishes_starting_before_it_connects(
        tmp_path, monkeypatch):
    published, built = [], []
    bridge, _ = _start_bridge(tmp_path, monkeypatch, published, built)
    try:
        bridge.start()
    finally:
        bridge.stop()
    assert len(built) == 1, "one status publisher, not two"
    assert published, "no STARTING status before CONNECTED"
    assert {message["state"] for message in published} == {
        int(BridgeState.STARTING)}
    assert len(published) >= len(SETUP_PHASES)


def test_a_failed_startup_is_recorded_with_its_phase(tmp_path, monkeypatch):
    from stk_bridge.core import scenario_manager

    published, built = [], []
    bridge, path = _start_bridge(tmp_path, monkeypatch, published, built)

    def stalled(self):
        raise RuntimeError("frames check failed")

    monkeypatch.setattr(scenario_manager.ScenarioManager, "_check_frames",
                        stalled)
    with pytest.raises(Exception, match="frames check failed"):
        try:
            bridge.start()
        finally:
            bridge.stop()
    state = _read(path)
    assert state["outcome"] == "failed"
    assert state["last_phase"] == "frames"
    assert "frames check failed" in state["error"]
