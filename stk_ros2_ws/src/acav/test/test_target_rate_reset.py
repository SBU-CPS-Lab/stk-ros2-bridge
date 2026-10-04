from dataclasses import replace
from unittest.mock import patch
import acav.twin_reference as tr
from acav.spec import AcavSpec


def test_target_identity_and_acquisition_reset_rate_but_motion_does_not():
    cfg = tr.TwinConfig.from_spec(AcavSpec.load())
    pos, vel = (6878137, 0, 0), (0, 7500, 0)
    target = (6378137, 0, 0)
    q = tr.target_attitude(pos, vel, target)
    first = tr.TwinInputs(q, (0,0,0), pos, vel, 0, target, True, 'T1')
    state = tr.twin_step(cfg, first, tr.TwinState()).state
    # Keep the reference error small enough to exercise tracking, not SLEW.
    moved = replace(first, time_s=cfg.step_s, target_eci_m=(6378137, 100, 0))
    def rate(inputs, previous):
        with patch.object(tr, 'control_torque', wraps=tr.control_torque) as controller:
            result = tr.twin_step(cfg, inputs, previous)
            return controller.call_args.args[-1], result.state
    same, _ = rate(moved, state)
    assert sum(x*x for x in same) > 0
    changed, changed_state = rate(replace(moved, target_id='T2'), state)
    assert changed == (0,0,0)
    resumed, _ = rate(replace(moved, target_id='T2', time_s=2*cfg.step_s,
                              target_eci_m=(6378137,200,0)), changed_state)
    assert sum(x*x for x in resumed) > 0
    lost, nadir_state = rate(replace(moved, have_target=False), state)
    assert lost == (0,0,0)
    acquired, _ = rate(replace(moved, time_s=2*cfg.step_s), nadir_state)
    assert acquired == (0,0,0)
