"""Regression coverage for attaching to a prebuilt ACAV scenario."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from stk_bridge.backends.com_backend import ComBackend
from stk_bridge.backends.mock_backend import MockBackend
from stk_bridge.core.frames import FrameRegistry
from stk_bridge.core.scenario_manager import ScenarioConfig, ScenarioManager
from stk_bridge.exceptions import QueryError


@pytest.fixture
def backend():
    backend = MockBackend()
    backend.connect()
    yield backend
    backend.disconnect()


def manager(backend, **kwargs):
    return ScenarioManager(backend, FrameRegistry(), ScenarioConfig(**kwargs))


def test_reuse_preserves_objects(backend):
    backend.new_scenario("ACAV_SSS1")
    backend.create_object("Satellite", "KeepMe")
    before = backend.list_objects()
    backend.create_object = Mock(side_effect=AssertionError("must not recreate"))
    backend.new_scenario = Mock(side_effect=AssertionError("must not create"))
    backend.load_scenario = Mock(side_effect=AssertionError("must not load"))
    manager(backend, use_existing=True, scenario_name="ACAV_SSS1",
            ensure_objects=[{"type": "Satellite", "name": "SSS-1"}]).setup(
        run_validation=False)
    assert backend.list_objects() == before
    assert backend.get_current_scenario_name() == "ACAV_SSS1"


@pytest.mark.parametrize("root", ["*", "*/"])
def test_scenario_root_child_path(root):
    from stk_bridge.core.conversions import child_path
    assert child_path(root, "Satellite", "SSS-1") == "*/Satellite/SSS-1"


@pytest.mark.parametrize("current", [None, "Unrelated"])
def test_reuse_rejects_missing_or_wrong_scenario(backend, current):
    if current:
        backend.new_scenario(current)
    before = backend.list_objects()
    with pytest.raises(QueryError, match="no scenario|expected"):
        manager(backend, use_existing=True,
                scenario_name="ACAV_SSS1")._open_scenario()
    assert backend.get_current_scenario_name() == current
    assert backend.list_objects() == before


def test_default_still_creates(backend):
    manager(backend, scenario_name="New")._open_scenario()
    assert backend.get_current_scenario_name() == "New"


def test_file_mode_still_loads(backend):
    backend.load_scenario = Mock()
    manager(backend, scenario_path="saved.sc")._open_scenario()
    backend.load_scenario.assert_called_once_with("saved.sc")


@pytest.mark.parametrize("kwargs", [
    {"use_existing": "true"},
    {"use_existing": True, "scenario_path": "saved.sc"},
    {"use_existing": True, "scenario_name": " "},
])
def test_invalid_reuse_config(kwargs):
    with pytest.raises(ValueError):
        ScenarioConfig.from_config({"scenario": kwargs})


@pytest.mark.parametrize("name", [None, "ACAV_SSS1"])
def test_com_reads_current_scenario(name):
    backend = ComBackend()
    backend._connected = True
    backend._root = SimpleNamespace(CurrentScenario=(
        None if name is None else SimpleNamespace(InstanceName=name)))
    assert backend.get_current_scenario_name() == name
