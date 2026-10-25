"""A run cannot hang after the scheduler exits.

A run once stalled 11.5 min between ``scheduler_exit`` and
``stk_graphics_end``: the runner's log line went to run.log and then
blocked writing to the console, where a QuickEdit selection had paused
output, until a keystroke; meanwhile the bridge, never stopped, timed out
at step 7200. Three safeguards:

- console output goes through a background thread, so a paused console
  never blocks the runner (acav.console);
- the runner turns QuickEdit off for its console while it runs and puts
  it back afterwards, recorded in summary.json (console_quickedit);
- every post-run step -- the graphics end check, stopping the bridge,
  stopping MATLAB, closing the topic capture -- runs under a bound: on
  timeout it is recorded as failed and the run goes on.

The automated twin's own console output goes to matlab_console.log, not
to the shared console.
"""

import ctypes
import io
import json
import logging
import os
import threading
import time
from types import SimpleNamespace

import pytest

import acav.deploy_run_cli as deploy_run_cli
from acav import console
from acav import twin_launcher as tl
from acav.deploy_run_cli import ConnectResult, Deps, execute_run

from test_deploy_run_cli import (  # noqa: I100, F401 - shared helpers
    FakeProcess, _config, offline_graph)

HANG = threading.Event()        # never set: a call that never returns


@pytest.fixture(autouse=True)
def short_bounds(monkeypatch):
    monkeypatch.setattr(deploy_run_cli, "POST_RUN_TIMEOUT_S",
                        {"stk_graphics_end": 0.3, "bridge_stop": 0.3,
                         "scheduler_stop": 0.3,
                         "matlab_session": 0.3, "topic_capture": 0.3})


def _deps(**overrides):
    base = dict(popen=lambda *a, **k: FakeProcess(),
                wait_for_bridge_connected=lambda *a: ConnectResult(True,
                                                                   "com"),
                wait_for_twin_ack=lambda *a: True, prompt=lambda m: None,
                disable_quickedit=lambda: ("disabled", None))
    base.update(overrides)
    return Deps(**base)


def _summary(config):
    return json.load(open(os.path.join(config.resolved_log_dir(),
                                       "summary.json")))


def _step(summary, name):
    return [s for s in summary["steps"] if s["step"] == name]


class TestBounds:
    def test_a_hanging_graphics_end_check(self, tmp_path):
        config = _config(tmp_path)

        entry = {"windows_2d": 0, "windows_3d": 0,
                 "main_window": "minimized", "visible": True, "mode": "off",
                 "step": 0}

        def read(path, phase, timeout):
            if phase == "end":
                HANG.wait()
            return {"startup": entry, "samples": [], "end": None}

        t0 = time.monotonic()
        code = execute_run(config, _deps(read_graphics_state=read))
        assert time.monotonic() - t0 < 10
        assert code == deploy_run_cli.EXIT_STEP_FAILED
        step = _step(_summary(config), "stk_graphics_end")
        assert step and not step[-1]["ok"]
        assert "timed out" in step[-1]["detail"]

    def test_a_bridge_that_will_not_stop(self, tmp_path):
        class Stuck(FakeProcess):
            def wait(self, timeout=None):
                if self.terminated:
                    HANG.wait()
                return 0

        config = _config(tmp_path)
        launched = []

        def popen(argv, **kwargs):
            launched.append(Stuck() if "stk_bridge.nodes.bridge_node"
                            in " ".join(argv) else FakeProcess())
            return launched[-1]

        t0 = time.monotonic()
        code = execute_run(config, _deps(popen=popen))
        assert time.monotonic() - t0 < 10
        assert code == deploy_run_cli.EXIT_STEP_FAILED
        step = _step(_summary(config), "bridge_stop")
        assert step and not step[0]["ok"] and "timed out" in step[0]["detail"]

    def test_a_topic_capture_that_will_not_close(self, tmp_path):
        config = _config(tmp_path)

        def recorder(*a):
            return SimpleNamespace(close=HANG.wait, error=None)

        t0 = time.monotonic()
        code = execute_run(config, _deps(start_recorder=recorder))
        assert time.monotonic() - t0 < 10
        assert code == deploy_run_cli.EXIT_STEP_FAILED
        step = _step(_summary(config), "topic_capture")
        assert step and not step[0]["ok"]

    def test_a_matlab_stop_that_hangs(self, tmp_path):
        class Twin:
            argv = ["matlab"]

            def start(self):
                pass

            def exit_code(self):
                return None

            def stop(self, timeout_s=None):
                HANG.wait()

        config = _config(tmp_path, twin_launch="matlab-batch")
        t0 = time.monotonic()
        code = execute_run(config, _deps(launch_twin=lambda rc, d: Twin()))
        assert time.monotonic() - t0 < 10
        assert code == deploy_run_cli.EXIT_STEP_FAILED
        step = _step(_summary(config), "matlab_session")
        assert step and not step[0]["ok"] and "timed out" in step[0]["detail"]

    def test_a_clean_run_records_no_extra_steps(self, tmp_path):
        config = _config(tmp_path)
        assert execute_run(config, _deps()) == 0
        names = [s["step"] for s in _summary(config)["steps"]]
        assert "bridge_stop" not in names        # recorded only on failure


class TestQuickEdit:
    def test_recorded_and_restored(self, tmp_path):
        restored = []
        config = _config(tmp_path)
        deps = _deps(disable_quickedit=lambda: ("disabled",
                                                lambda: restored.append(1)))
        assert execute_run(config, deps) == 0
        assert _summary(config)["console_quickedit"] == "disabled"
        assert restored == [1]

    def test_the_console_mode_bits(self):
        class Kernel32:
            def __init__(self, mode, ok=True):
                self.mode, self.ok, self.set = mode, ok, []

            def GetStdHandle(self, which):
                assert which == console.STD_INPUT_HANDLE
                return 7

            def GetConsoleMode(self, handle, pointer):
                pointer._obj.value = self.mode
                return self.ok

            def SetConsoleMode(self, handle, mode):
                self.set.append(mode)
                return True

        k = Kernel32(0x1F7)                      # QuickEdit on
        status, restore = console.disable_quickedit(kernel32=k)
        assert status == "disabled"
        assert k.set[-1] & console.ENABLE_QUICK_EDIT_MODE == 0
        assert k.set[-1] & console.ENABLE_EXTENDED_FLAGS
        restore()
        assert k.set[-1] == 0x1F7

        status, restore = console.disable_quickedit(
            kernel32=Kernel32(0x1F7 & ~console.ENABLE_QUICK_EDIT_MODE))
        assert status == "already off" and restore is None
        status, restore = console.disable_quickedit(
            kernel32=Kernel32(0, ok=False))
        assert status.startswith("no console") and restore is None

    def test_not_windows(self, monkeypatch):
        monkeypatch.setattr(console.os, "name", "posix")
        assert console.disable_quickedit() == ("not windows", None)


class TestConsoleLogging:
    def test_a_paused_console_does_not_block_logging(self):
        class Paused(io.StringIO):
            def write(self, text):
                HANG.wait()

        handler = console.BackgroundConsoleHandler(Paused())
        logger = logging.getLogger("p9e.test")
        logger.addHandler(handler)
        logger.propagate = False
        try:
            t0 = time.monotonic()
            for _ in range(5):
                logger.warning("still running")
            assert time.monotonic() - t0 < 1.0
        finally:
            logger.removeHandler(handler)
            handler.close()                    # bounded, does not hang

    def test_lines_still_reach_the_console(self):
        stream = io.StringIO()
        handler = console.BackgroundConsoleHandler(stream)
        handler.setFormatter(logging.Formatter("%(message)s"))
        logger = logging.getLogger("p9e.test2")
        logger.addHandler(handler)
        logger.propagate = False
        logger.warning("hello")
        handler.close()
        logger.removeHandler(handler)
        assert stream.getvalue() == "hello\n"


def test_the_twin_does_not_write_to_the_shared_console(tmp_path):
    seen = {}

    def popen(argv, **kwargs):
        seen.update(kwargs)
        return SimpleNamespace(pid=1, poll=lambda: 0)

    twin = tl.MatlabBatchTwin(str(tmp_path / "r.json"),
                              str(tmp_path / "logs"), popen=popen,
                              matlab_dir=str(tmp_path))
    twin.start()
    assert seen["stdout"].name == str(tmp_path / "logs" /
                                      tl.MATLAB_CONSOLE_FILE)
    assert seen["stderr"] == tl.subprocess.STDOUT
    twin.stop(timeout_s=1.0)
    assert seen["stdout"].closed


def test_kill_tree_is_bounded(monkeypatch):
    seen = {}
    monkeypatch.setattr(tl.subprocess, "run",
                        lambda argv, **k: seen.update(k))
    tl.kill_process_tree(5)
    assert seen["timeout"] > 0
