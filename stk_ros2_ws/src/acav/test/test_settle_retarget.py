from dataclasses import replace
import acav.twin_reference as tr
from acav.spec import AcavSpec


def test_settle_retargets_once_and_safe_stays_latched():
    cfg = tr.TwinConfig.from_spec(AcavSpec.load())
    inputs = tr.TwinInputs((0, 0, 0, 1), (0, 0, 0),
                          (6878137, 0, 0), (0, 7500, 0), 10,
                          (6378137, 100000, 0), True, 'T2')
    state = tr.TwinState(mode_state=tr.ModeState(mode=tr.MissionMode.SETTLE),
                         previous_target_key=(True, 'T1'))
    result = tr.twin_step(cfg, inputs, state)
    assert result.state.mode_state.mode is tr.MissionMode.SLEW
    assert result.state.mode_state.slew_started_s == 10
    same = tr.twin_step(cfg, inputs, replace(state, previous_target_key=(True, 'T2')))
    assert same.state.mode_state.mode is tr.MissionMode.SETTLE
    again = tr.twin_step(cfg, replace(inputs, time_s=10.1), result.state)
    assert again.state.mode_state.slew_started_s == 10
    safe = tr.twin_step(cfg, inputs, replace(state, mode_state=tr.ModeState(mode=tr.MissionMode.SAFE)))
    assert safe.state.mode_state.mode is tr.MissionMode.SAFE
