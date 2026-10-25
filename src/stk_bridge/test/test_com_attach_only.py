"""The COM backend attaches to the operator's STK; it starts one only
when told to.

P10 live session (3186642): build_acav_scenario found no STK and
silently started one through COM ("no running STK found; starting a
new one"; AgUiApplication.exe -Embedding, parent svchost). E6 on that
STK stepped at 95.7 ms (sense 57.3 ms) against 60.0-61.1 ms on the P9
runs, and a free-run bridge's convention validation on it ran past the
runner's 90 s. One run, so a correlation, not a measured cause; final
runs use an STK the operator started, by protocol.

Now attaching is the only default: with no STK to attach to, connect
fails and says what to do. backend.com.start_stk: true (build and
verify scripts: --start-stk) restores the old fallback, with a warning.
"""

import logging
import sys
import types

import pytest

from stk_bridge.backends import create_backend
from stk_bridge.backends.com_backend import ComBackend
from stk_bridge.exceptions import BackendUnavailableError


class FakeDesktop:
    def __init__(self, running):
        self.running = running
        self.started = []

    def AttachToApplication(self):
        if not self.running:
            raise RuntimeError("no STK in the running object table")
        return "attached"

    def StartApplication(self, visible=True, userControl=True):
        self.started.append((visible, userControl))
        return "started"


@pytest.fixture
def desktop(monkeypatch):
    def install(running):
        fake = FakeDesktop(running)
        module = types.ModuleType("agi.stk12.stkdesktop")
        module.STKDesktop = fake
        for name in ("agi", "agi.stk12"):
            monkeypatch.setitem(sys.modules, name,
                                types.ModuleType(name))
        monkeypatch.setitem(sys.modules, "agi.stk12.stkdesktop", module)
        return fake
    return install


def test_with_no_stk_it_refuses_to_start_one(desktop):
    fake = desktop(running=False)
    with pytest.raises(BackendUnavailableError) as raised:
        ComBackend()._obtain_application()
    assert fake.started == []
    message = str(raised.value)
    assert "no running STK Desktop to attach to" in message
    assert "Start STK Desktop yourself" in message
    assert "start_stk" in message


def test_a_running_stk_is_attached_to(desktop):
    fake = desktop(running=True)
    assert ComBackend()._obtain_application() == "attached"
    assert fake.started == []


def test_start_stk_starts_one_and_says_so_loudly(desktop, caplog):
    fake = desktop(running=False)
    with caplog.at_level(logging.WARNING):
        assert ComBackend(start_stk=True)._obtain_application() == "started"
    assert fake.started == [(True, True)]
    assert "STARTED A NEW STK Desktop through COM" in caplog.text


def test_not_attaching_at_all_is_a_start_too_and_says_so(desktop, caplog):
    fake = desktop(running=True)
    with caplog.at_level(logging.WARNING):
        assert ComBackend(attach_to_existing=False)._obtain_application() \
            == "started"
    assert fake.started
    assert "STARTED A NEW STK Desktop through COM" in caplog.text


def test_the_factory_defaults_to_attach_only():
    assert create_backend({"backend": {"type": "com"}})._start_stk is False
    assert create_backend({"backend": {
        "type": "com", "com": {"start_stk": True}}})._start_stk is True
