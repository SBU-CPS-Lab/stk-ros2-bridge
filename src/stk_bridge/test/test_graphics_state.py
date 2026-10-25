"""The bridge measures STK's graphics state and writes it for the runner.

STK's 2D/3D Graphics windows redraw on every step: closing them and
minimising STK cut an E6 step from 120.3 to 70.2 ms with an identical
signature. The state is read through documented, read-only
Object Model calls (IAgUiApplication.Windows / WindowState / Visible,
IAgUiWindow.GetServiceByType) at startup, every SUMMARY_EVERY_STEPS steps
and at the terminal epoch, and written to the file named by the bridge
config's graphics_state_file.

"off" is exactly the measured condition: no 2D window, no 3D window and
the main window minimised. Anything else is "on".
"""

import json
import logging
import threading
from types import SimpleNamespace

import pytest

from stk_bridge.backends.base import StkTime
from stk_bridge.core import graphics_state as gs
from stk_bridge.nodes.bridge_node import SUMMARY_EVERY_STEPS, StkBridgeNode

OFF = {"windows_2d": 0, "windows_3d": 0, "main_window": "minimized",
       "visible": True}


class TestClassify:
    def test_the_measured_condition_is_off(self):
        assert gs.classify(OFF) == "off"

    @pytest.mark.parametrize("change", [
        {"windows_2d": 1}, {"windows_3d": 2},
        {"main_window": "normal"}, {"main_window": "maximized"}])
    def test_anything_else_is_on(self, change):
        assert gs.classify({**OFF, **change}) == "on"

    def test_no_state_is_unavailable(self):
        assert gs.classify(None) == "unavailable"


class TestLog:
    def test_written_after_every_sample(self, tmp_path):
        path = tmp_path / "stk_graphics.json"
        log = gs.GraphicsLog(str(path))
        log.add("startup", OFF, step=0)
        data = json.loads(path.read_text(encoding="utf-8"))
        assert data["startup"] == {**OFF, "mode": "off", "step": 0}
        assert data["samples"] == [] and data["end"] is None
        log.add("running", {**OFF, "windows_3d": 1}, step=1000)
        log.add("end", OFF, step=7200)
        data = json.loads(path.read_text(encoding="utf-8"))
        assert [s["mode"] for s in data["samples"]] == ["on"]
        assert data["end"]["step"] == 7200

    def test_no_path_keeps_it_in_memory(self):
        log = gs.GraphicsLog("")
        log.add("startup", OFF, step=0)
        assert log.data["startup"]["mode"] == "off"


class TestSummarize:
    def _data(self, *modes, end=True):
        states = {"off": OFF, "on": {**OFF, "windows_2d": 1}}
        log = gs.GraphicsLog("")
        log.add("startup", states[modes[0]], step=0)
        for mode in modes[1:-1] if end else modes[1:]:
            log.add("running", states[mode], step=1000)
        if end:
            log.add("end", states[modes[-1]], step=7200)
        return log.data

    def test_consistent(self):
        assert gs.summarize(self._data("off", "off", "off")) == "off"
        assert gs.summarize(self._data("on", "on")) == "on"

    def test_a_change_mid_run_is_mixed(self):
        assert gs.summarize(self._data("off", "on", "off")) == "mixed"
        assert gs.summarize(self._data("off", "on")) == "mixed"

    def test_no_end_sample_is_unverified(self):
        assert gs.summarize(self._data("off", "off", end=False)) == \
            "unverified"

    def test_no_startup_sample(self):
        assert gs.summarize({"startup": None, "samples": [],
                             "end": None}) is None

    def test_describe(self):
        assert gs.describe({**OFF, "mode": "off"}) == (
            "off (2D windows 0, 3D windows 0, main window minimized, "
            "visible True)")


class TestComBackend:
    def test_counts_and_main_window(self):
        uicore = pytest.importorskip("agi.stk12.uicore")
        from stk_bridge.backends.com_backend import ComBackend

        service = uicore.AgEWindowService

        def window(*kinds):
            return SimpleNamespace(GetServiceByType=lambda kind: (
                object() if kind in kinds else None))

        backend = ComBackend()
        backend._stk = SimpleNamespace(
            Windows=[window(service.eWindowService2DWindow),
                     window(service.eWindowService3DWindow),
                     window(service.eWindowService3DWindow), window()],
            WindowState=uicore.AgEWindowState.eWindowStateNormal,
            Visible=False)
        assert backend.graphics_state() == {
            "windows_2d": 1, "windows_3d": 2, "main_window": "normal",
            "visible": False}

    def test_minimized_and_no_windows(self):
        uicore = pytest.importorskip("agi.stk12.uicore")
        from stk_bridge.backends.com_backend import ComBackend

        backend = ComBackend()
        backend._stk = SimpleNamespace(
            Windows=[], Visible=True,
            WindowState=uicore.AgEWindowState.eWindowStateMinimized)
        assert gs.classify(backend.graphics_state()) == "off"

    def test_not_connected(self):
        from stk_bridge.backends.com_backend import ComBackend
        assert ComBackend().graphics_state() is None

    def test_other_backends_report_nothing(self):
        from stk_bridge.backends.mock_backend import MockBackend
        assert MockBackend().graphics_state() is None


class TestBridgeSampling:
    def _node(self, tmp_path, states):
        node = StkBridgeNode.__new__(StkBridgeNode)
        node.config = {"graphics_state_file": str(tmp_path / "g.json")}
        backend = SimpleNamespace(graphics_state=lambda: states.pop(0))
        node.worker = SimpleNamespace(
            call=lambda fn, description, **k: fn(backend))
        node.time_sync = SimpleNamespace(step_index=0)
        node._graphics_log = gs.GraphicsLog(node.config["graphics_state_file"])
        return node

    def test_startup_running_and_terminal_samples(self, tmp_path, caplog):
        epoch = 1717237800.0
        node = self._node(tmp_path, [OFF] * 10)
        node.sensing = SimpleNamespace(
            combine_summary_line=lambda: "combined attitude/rate summary: x",
            orbit_batch_summary_line=lambda: "orbit batch summary: y")
        node._scenario_stop = StkTime(epoch + 720.0)
        with caplog.at_level(logging.INFO):
            node._sample_graphics("startup")
            for index in range(1, 7201):
                node.time_sync.step_index = index
                node._sensing_summary_checkpoint(StkTime(epoch + index / 10))
        data = json.loads((tmp_path / "g.json").read_text(encoding="utf-8"))
        assert data["startup"]["step"] == 0
        assert [s["step"] for s in data["samples"]] == [
            1000 * k for k in range(1, 8)]
        assert data["end"]["step"] == 7200
        assert gs.summarize(data) == "off"
        assert "STK graphics (startup): off (2D windows 0" in caplog.text
        assert SUMMARY_EVERY_STEPS == 1000

    def test_a_failed_read_is_recorded_as_unavailable(self, tmp_path):
        node = self._node(tmp_path, [])

        def failing(fn, description, **k):
            raise RuntimeError("COM error")

        node.worker = SimpleNamespace(call=failing)
        node._sample_graphics("startup")
        data = json.loads((tmp_path / "g.json").read_text(encoding="utf-8"))
        assert data["startup"]["mode"] == "unavailable"
        assert "COM error" in data["startup"]["error"]

    def test_a_node_without_the_log_does_nothing(self):
        node = StkBridgeNode.__new__(StkBridgeNode)
        node._sample_graphics("startup")          # no attribute: no-op
