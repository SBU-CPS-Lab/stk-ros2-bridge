function test_settle_retarget()
%TEST_SETTLE_RETARGET A target change during SETTLE starts a new slew;
% the same target does not.
    cfg = adcs.config(); state = adcs.initial_state();
    state.mode = 'SETTLE'; state.previousTargetKey = {true,'T1'};
    in = struct('quaternion',[0 0 0 1],'omega',[0 0 0], ...
        'position',[6878137 0 0],'velocity',[0 7500 0], ...
        'targetEci',[6378137 100000 0],'haveTarget',true,'t',10,'targetId','T2');
    changed = adcs.twin_step(in,cfg,state);
    assert(strcmp(changed.state.mode,'SLEW'));
    state.previousTargetKey = {true,'T2'};
    same = adcs.twin_step(in,cfg,state);
    assert(strcmp(same.state.mode,'SETTLE'));
    in.t = 10.1;
    next = adcs.twin_step(in,cfg,changed.state);
    assert(next.state.slewStarted == 10);
    state.mode = 'SAFE';
    safe = adcs.twin_step(in,cfg,state);
    assert(strcmp(safe.state.mode,'SAFE'));
    fprintf('PASS: SETTLE retargets once; SAFE remains latched.\n');
end
