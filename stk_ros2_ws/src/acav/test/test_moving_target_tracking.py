import math
from types import SimpleNamespace
from acav.scheduler_node import interpolate_target_position, SchedulerNode
from acav.twin_reference import TwinConfig, TwinState, TwinInputs, twin_step, target_attitude
from acav.spec import AcavSpec


def test_target_interpolation_uses_current_time():
    positions = [(10, 20, 30), (20, 40, 60), (30, 60, 90)]
    assert interpolate_target_position(positions, 5) == (15, 30, 45)
    assert interpolate_target_position(positions, 20) == positions[-1]
    assert interpolate_target_position(positions, 0) == positions[0]


def test_scheduler_keeps_all_position_samples_from_bridge_response():
    node = SchedulerNode.__new__(SchedulerNode)
    node._spec = SimpleNamespace(window_duration_s=20)
    node._target_positions = {}
    result = SimpleNamespace(success=True, row_count=3,
                             column_names=['x', 'y', 'z'],
                             flattened_data=[10,20,30,20,40,60,30,60,90])
    future = SimpleNamespace(result=lambda: SimpleNamespace(result=result))
    node._on_target_position_response('T1','*/Target/T1',future)
    assert interpolate_target_position(node._target_positions['T1'],5) == (15,30,45)


def test_moving_line_of_sight_does_not_have_zero_rate_tracking_bias():
    cfg = TwinConfig.from_spec(AcavSpec.load())
    state = TwinState()
    pos, vel = (6878137, 0, 0), (0, 7500, 0)
    q = target_attitude(pos, vel, (6378137, 0, 0))
    w = (0, 0, 0)
    errors = []
    for i in range(601):
        t = i * cfg.step_s
        target = (pos[0] - 500000 * math.cos(.01*t), 500000*math.sin(.01*t), 0)
        out = twin_step(cfg, TwinInputs(q, w, pos, vel, t, target, True), state)
        q, w, state = out.quaternion_cmd, out.rate_next_rad_s, out.state
        if t >= 58:
            errors.append(math.degrees(out.boresight_error_rad))
    assert max(errors) < .1
