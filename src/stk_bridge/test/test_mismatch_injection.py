"""E1 interface-mismatch injection at the bridge's publish side.

Five opt-in mismatches (sensing.inject_mismatch), applied to the samples
SatellitePublishers.publish_samples is about to publish -- the one path
the bridge publishes telemetry through (nodes/bridge_node.py, three
calls) -- and so to what the twin reads, never to what the bridge
writes to or reads from STK:

  quaternion_order      (x, y, z, w) published as (w, x, y, z)
  quaternion_conjugate  the conjugate (body->inertial instead of
                        inertial->body)
  angle_rate_units      body rate in deg/s instead of rad/s
  distance_units        position and velocity in km and km/s
  time_offset           telemetry stamped 37 s late (TAI - UTC); the
                        clock and ScenarioTime are not shifted

Off by default; with it off, every message published is identical to
the previous implementation's: the reference is the fixture
fixtures/publisher_messages.json, written before the
injection existed by running this file with --write.

ROS is faked: message classes record attributes, builtin_interfaces
Time is a plain class, so the real conversions run.
"""

import json
import math
import os
import sys
import types

import pytest

from stk_bridge.backends.base import (
    AngularRateSample,
    AttitudeSample,
    EclipseCondition,
    EclipseSample,
    LLASample,
    StateSample,
    StkTime,
)
from stk_bridge.core.frames import FrameRegistry

FIXTURE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "fixtures", "publisher_messages.json")
SAT = "*/Satellite/SSS-1"
EPOCH = 1717237800.123456
QUAT = (0.1, -0.2, 0.3, math.sqrt(1 - 0.01 - 0.04 - 0.09))


class Message:
    """Records every attribute set on it; children appear on first use."""

    UMBRA, PENUMBRA, SUNLIT = 0, 1, 2

    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        child = Message()
        object.__setattr__(self, name, child)
        return child

    def to_dict(self):
        return {key: (value.to_dict() if isinstance(value, Message)
                      else value)
                for key, value in sorted(vars(self).items())}


class Time:
    def __init__(self, sec=0, nanosec=0):
        self.sec, self.nanosec = sec, nanosec

    def to_dict(self):
        return {"sec": self.sec, "nanosec": self.nanosec}


def _install_fakes(monkeypatch):
    msgs = types.ModuleType("stk_msgs.msg")
    for name in ("OrbitState", "AttitudeState", "AngularRate", "LLAState",
                 "EclipseState", "SatelliteState"):
        setattr(msgs, name, type(name, (Message,), {}))
    std = types.ModuleType("std_msgs.msg")
    std.Header = type("Header", (Message,), {})
    builtin = types.ModuleType("builtin_interfaces.msg")
    builtin.Time = Time
    for name, module in (("stk_msgs", types.ModuleType("stk_msgs")),
                         ("stk_msgs.msg", msgs),
                         ("std_msgs", types.ModuleType("std_msgs")),
                         ("std_msgs.msg", std),
                         ("builtin_interfaces",
                          types.ModuleType("builtin_interfaces")),
                         ("builtin_interfaces.msg", builtin)):
        monkeypatch.setitem(sys.modules, name, module)


class Node:
    def __init__(self):
        self.published = []

    def create_publisher(self, kind, topic, qos):
        node = self

        class Publisher:
            def publish(self, message):
                node.published.append((topic, _plain(message)))
        return Publisher()


def _plain(value):
    if isinstance(value, (Message, Time)):
        return {k: _plain(v) for k, v in value.to_dict().items()}
    if isinstance(value, dict):
        return {k: _plain(v) for k, v in value.items()}
    return value


def _samples(step=0):
    when = StkTime(EPOCH + 0.1 * step)
    return {
        "orbit": StateSample(when, "icrf", (6878137.0 + step, 1.5, -2.25),
                             (0.5, 7612.3, -1.0)),
        "attitude": AttitudeSample(when, "icrf", QUAT),
        "angular_rate": AngularRateSample(when, (0.01, -0.02, 0.003)),
        "lla": LLASample(when, 0.5, -1.2, 500000.0),
        "eclipse": EclipseSample(when, EclipseCondition.UMBRA),
    }


def publish(monkeypatch, **kwargs):
    _install_fakes(monkeypatch)
    from stk_bridge.interfaces.ros_publishers import SatellitePublishers

    node = Node()
    publishers = SatellitePublishers(
        node, SAT, FrameRegistry(), {"telemetry": None}, "stk", **kwargs)
    for step in range(3):
        publishers.publish_samples(_samples(step))
    return node.published


def _by_topic(published, suffix):
    return [m for topic, m in published if topic.endswith(suffix)]


# --- off: identical to the previous implementation --------------------------

def test_with_injection_off_every_message_is_unchanged(monkeypatch):
    with open(FIXTURE, encoding="utf-8") as handle:
        expected = json.load(handle)
    published = publish(monkeypatch)
    assert json.loads(json.dumps(published)) == expected


def test_an_empty_mismatch_is_the_same_as_none(monkeypatch):
    with open(FIXTURE, encoding="utf-8") as handle:
        expected = json.load(handle)
    published = publish(monkeypatch, mismatch="")
    assert json.loads(json.dumps(published)) == expected


# --- each type ----------------------------------------------------------------

def _q(message):
    o = message["orientation"]
    return (o["x"], o["y"], o["z"], o["w"])


def test_quaternion_order(monkeypatch):
    attitude = _by_topic(publish(monkeypatch,
                                 mismatch="quaternion_order"), "/attitude")
    x, y, z, w = QUAT
    assert _q(attitude[0]) == (w, x, y, z)


def test_quaternion_conjugate(monkeypatch):
    attitude = _by_topic(publish(monkeypatch,
                                 mismatch="quaternion_conjugate"),
                         "/attitude")
    x, y, z, w = QUAT
    assert _q(attitude[0]) == (-x, -y, -z, w)


def test_angle_rate_units(monkeypatch):
    rate = _by_topic(publish(monkeypatch, mismatch="angle_rate_units"),
                     "/angular_rate")
    w = rate[0]["angular_velocity"]
    assert (w["x"], w["y"], w["z"]) == pytest.approx(
        tuple(math.degrees(v) for v in (0.01, -0.02, 0.003)))


def test_distance_units(monkeypatch):
    orbit = _by_topic(publish(monkeypatch, mismatch="distance_units"),
                      "/orbit")
    p, v = orbit[0]["position"], orbit[0]["velocity"]
    assert (p["x"], p["y"], p["z"]) == pytest.approx(
        (6878.137, 0.0015, -0.00225))
    assert (v["x"], v["y"], v["z"]) == pytest.approx(
        (0.0005, 7.6123, -0.001))


def test_time_offset_shifts_every_telemetry_stamp_by_37_s(monkeypatch):
    off = publish(monkeypatch)
    late = publish(monkeypatch, mismatch="time_offset")
    for suffix in ("/orbit", "/attitude", "/angular_rate"):
        a = _by_topic(off, suffix)[0]["header"]["stamp"]
        b = _by_topic(late, suffix)[0]["header"]["stamp"]
        assert (b["sec"] - a["sec"]) + (b["nanosec"] - a["nanosec"]) * 1e-9 \
            == pytest.approx(37.0, abs=1e-9)


@pytest.mark.parametrize("kind", ["quaternion_order", "quaternion_conjugate",
                                  "angle_rate_units", "distance_units",
                                  "time_offset"])
def test_each_type_changes_only_its_own_fields(monkeypatch, kind):
    """The other channels stay byte-identical; the composite state topic
    carries the same injected values as the per-channel topics."""
    with open(FIXTURE, encoding="utf-8") as handle:
        expected = json.load(handle)
    published = json.loads(json.dumps(publish(monkeypatch, mismatch=kind)))
    touched = {"quaternion_order": ("/attitude", "/state"),
               "quaternion_conjugate": ("/attitude", "/state"),
               "angle_rate_units": ("/angular_rate", "/state"),
               "distance_units": ("/orbit", "/state"),
               "time_offset": ("/orbit", "/attitude", "/angular_rate",
                               "/state")}[kind]
    assert [t for t, _ in published] == [t for t, _ in expected]
    for (topic, got), (_, want) in zip(published, expected):
        if topic.endswith(touched):
            assert got != want, topic
        else:
            assert got == want, topic


def test_an_unknown_type_is_refused():
    from stk_bridge.core.mismatch_injection import inject
    with pytest.raises(ValueError, match="unknown mismatch"):
        inject("degrees_everywhere", _samples())


def test_the_sensing_config_reads_it_and_defaults_to_off():
    from stk_bridge.core.sensing import SensingConfig
    assert SensingConfig.from_config({}).inject_mismatch == ""
    assert SensingConfig.from_config({"sensing": {
        "inject_mismatch": "distance_units"}}).inject_mismatch == \
        "distance_units"


def test_the_types_are_the_documented_five():
    from stk_bridge.core.mismatch_injection import MISMATCH_TYPES
    assert MISMATCH_TYPES == ("quaternion_order", "quaternion_conjugate",
                              "angle_rate_units", "distance_units",
                              "time_offset")


if __name__ == "__main__" and "--write" in sys.argv:
    class _Patch:
        def setitem(self, mapping, name, value):
            mapping[name] = value
    published = publish(_Patch())
    os.makedirs(os.path.dirname(FIXTURE), exist_ok=True)
    with open(FIXTURE, "w", encoding="utf-8") as handle:
        json.dump(published, handle, indent=1, sort_keys=True)
        handle.write("\n")
    print(f"wrote {len(published)} messages to {FIXTURE}")
