function test_target_rate_reset()
%TEST_TARGET_RATE_RESET The reference-rate history is kept for the same
% target and reset when the target changes.
    cfg = adcs.config();
    pos = [6878137 0 0]; vel = [0 7500 0]; target = [6378137 0 0];
    q = adcs.quat.align(cfg.boresightBody, (target-pos)/norm(target-pos), [1 0 0], vel);
    in = struct('quaternion',q,'omega',[0 0 0],'position',pos, ...
        'velocity',vel,'targetEci',target,'haveTarget',true,'t',0,'targetId','T1');
    first = adcs.twin_step(in,cfg,adcs.initial_state());
    in.t = cfg.stepS; in.targetEci = target+[0 100 0];
    same = adcs.twin_step(in,cfg,first.state);
    fresh = first.state; fresh.previousDesired = [];
    expected = adcs.twin_step(in,cfg,fresh);
    assert(norm(same.torque-expected.torque)>1e-10,'Same-target motion lost feedforward.');
    in.targetId = 'T2';
    changed = adcs.twin_step(in,cfg,first.state);
    assert(norm(changed.torque-expected.torque)<1e-12,'Target change produced a rate impulse.');
    in.t = 2*cfg.stepS; in.targetEci = target+[0 200 0];
    resumed = adcs.twin_step(in,cfg,changed.state);
    fresh = changed.state; fresh.previousDesired = [];
    expected = adcs.twin_step(in,cfg,fresh);
    assert(norm(resumed.torque-expected.torque)>1e-10,'Feedforward did not resume.');
    fprintf('PASS: target switch resets rate; same-target motion retains feedforward.\n');
end
