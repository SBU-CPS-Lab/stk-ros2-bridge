"""The record's attitude provider is what the bridge reported, not what was asked.

``Provenance.attitude_provider`` used to be copied from the scheduler's
own launch parameter, so E1's treatment label proved nothing about what
the bridge read through (the 2026-10-08 E1 pair needed its bridge log
checked by hand). It is now derived from ``BridgeStatus``, the same way
the backend type is.

``SchedulerNode`` is built with ``__new__``; nothing here touches rclpy.
"""

import logging
from types import SimpleNamespace

from acav.run_record import Provenance
from acav.scheduler_node import SchedulerNode

ECF = "ECF Attitude Quaternions"
INERTIAL = "Attitude Quaternions"


def _node(requested=""):
    node = SchedulerNode.__new__(SchedulerNode)
    node._options = SimpleNamespace(attitude_provider=requested)
    node._readback_provider = ""
    node._readback_forced = None
    return node


def _status(provider, forced):
    return SimpleNamespace(attitude_provider=provider,
                           attitude_provider_forced=forced)


def test_a_forced_provider_is_recorded_from_the_bridge():
    node = _node(requested=ECF)
    node._observe_attitude_provider(_status(ECF, True))
    assert node._recorded_attitude_provider() == ECF
    assert node._readback_provider == ECF


def test_a_validated_provider_records_no_override():
    node = _node()
    node._observe_attitude_provider(_status(INERTIAL, False))
    assert node._recorded_attitude_provider() == ""
    assert node._readback_provider == INERTIAL


def test_a_treatment_the_bridge_did_not_apply_is_recorded_as_a_control(caplog):
    """The case the old label hid: asked for ECF, bridge read validated."""
    node = _node(requested=ECF)
    with caplog.at_level(logging.ERROR):
        node._observe_attitude_provider(_status(INERTIAL, False))
    assert node._recorded_attitude_provider() == ""
    assert "launched for forced provider" in caplog.text


def test_a_provider_forced_by_a_stale_yaml_is_recorded(caplog):
    node = _node(requested="")
    with caplog.at_level(logging.ERROR):
        node._observe_attitude_provider(_status(ECF, True))
    assert node._recorded_attitude_provider() == ECF
    assert "launched for forced provider" in caplog.text


def test_status_before_sensing_is_configured_is_not_an_observation():
    node = _node(requested=ECF)
    node._observe_attitude_provider(_status("", False))
    assert node._readback_forced is None
    # Unobserved: falls back to the request, and the campaign runner
    # rejects an E1 record whose read-back provider is empty.
    assert node._recorded_attitude_provider() == ECF
    assert node._readback_provider == ""


def test_an_old_bridge_status_without_the_fields_is_tolerated():
    node = _node()
    node._observe_attitude_provider(SimpleNamespace())
    assert node._readback_forced is None


def test_the_read_back_provider_round_trips_and_old_records_load():
    provenance = Provenance(attitude_provider=ECF,
                            attitude_readback_provider=ECF)
    assert Provenance.from_dict(provenance.to_dict()) == provenance
    old = provenance.to_dict()
    del old["attitude_readback_provider"]
    assert Provenance.from_dict(old).attitude_readback_provider == ""
