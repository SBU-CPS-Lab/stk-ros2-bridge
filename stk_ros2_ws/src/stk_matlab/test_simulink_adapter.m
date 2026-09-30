function test_simulink_adapter()
%TEST_SIMULINK_ADAPTER Check the Normal-mode boundary against twin_step.
    cfg = adcs.config();
    state = adcs.initial_state();
    q = [0 0 0 1]; omega = [0 0 0];
    pos = [6878137 0 0]; vel = [0 7500 0]; target = [6378137 0 0];
    names = {'NADIR','SLEW','SETTLE','IMAGE','SAFE'};
    % Startup defaults and invalid samples must neither throw nor advance
    % the controller. The first valid sample must match a fresh twin.
    [~,~,~,~,h,~,energy,ready,~,invalidOmega] = adcs.simulink_step( ...
        zeros(1,4),zeros(1,3),zeros(1,3),zeros(1,3),zeros(1,3),false,0,true,'');
    assert(~ready && all(h==0) && energy==0 && all(invalidOmega==0));
    for k = 1:600
        t = (k-1)*cfg.stepS;
        in = struct('quaternion',q,'omega',omega,'position',pos, ...
            'velocity',vel,'targetEci',target,'haveTarget',true,'t',t);
        expected = adcs.twin_step(in,cfg,state);
        if k == 5
            [~,~,~,~,h,~,e,valid] = adcs.simulink_step( ...
                [NaN 0 0 1],omega,pos,vel,target,true,t,false,'');
            assert(~valid && isequal(h,state.wheelMomentum) && e==state.wheelEnergyJ);
        end
        qRead = q;
        if mod(k,2)==0, qRead = -q; end
        [torque,qCmd,mode,err,momentum,saturated,energy,ready,~,omegaCmd] = ...
            adcs.simulink_step(qRead,zeros(1,3),pos,vel,target,true,t,false,'');
        assert(ready);
        assert(norm(torque-expected.torque)<1e-12);
        assert(norm(qCmd-expected.quaternionCmd)<1e-12);
        assert(norm(omegaCmd-expected.omegaNext)<1e-12);
        assert(mode~=5,'Nominal slew must not enter SAFE.');
        assert(strcmp(names{mode},expected.state.mode));
        assert(abs(err-expected.boresightErrorRad)<1e-12);
        assert(norm(momentum-expected.state.wheelMomentum)<1e-12);
        assert(saturated==expected.wheelsSaturated);
        assert(abs(energy-expected.state.wheelEnergyJ)<1e-12);
        [~,repeatQ,~,~,repeatH,~,repeatEnergy,valid,fresh,repeatOmega] = ...
            adcs.simulink_step(q,omega,pos,vel,target,true,t,false,'');
        assert(valid && ~fresh && isequal(repeatQ,qCmd));
        assert(isequal(repeatH,momentum) && repeatEnergy==energy);
        assert(isequal(repeatOmega,omegaCmd));
        state = expected.state; q = qCmd; omega = expected.omegaNext;
    end
    assert(err<deg2rad(0.5),'Nominal slew must converge below 0.5 deg.');
    fprintf('600-step endpoint regression: final error %.6g deg.\n',rad2deg(err));
    [~,first] = adcs.simulink_step([0 0 0 1],[0 0 0],pos,vel,target,true,0,true,'');
    [~,again] = adcs.simulink_step([0 0 0 1],[0 0 0],pos,vel,target,true,0,true,'');
    assert(isequal(first,again),'Simulation restart must reset controller state.');
    assertError(@() adcs.simulink_step([1 0 0 0],[0 0 0], ...
        pos,vel,target,true,cfg.stepS,false,''), ...
        'adcs:simulink_step:AttitudeMismatch');
    assertError(@() adcs.simulink_step(again,[0 0 0], ...
        pos,vel,target,true,2*cfg.stepS,false,''), ...
        'adcs:simulink_step:TimeGap');
    runs = cell(1, 2);
    for variant = 1:2
        qv = [0 0 0 1];
        for k = 1:6
            fed = [0 0 0];
            if variant == 2 && k > 1, fed = [27 -31 19]; end
            [~, qv] = adcs.simulink_step(qv, fed, pos, vel, target, true, ...
                (k-1)*cfg.stepS, k == 1, '');
        end
        runs{variant} = qv;
    end
    assert(norm(runs{1}-runs{2}) < 1e-12, ...
        'A reported-rate spike changed the integrated attitude.');
    fprintf('PASS: startup/reset, endpoint continuity, quaternion sign, mismatch/time-gap guards.\n');
end

function assertError(f, id)
    try
        f();
    catch err
        assert(strcmp(err.identifier,id),err.message);
        return
    end
    error('Expected %s',id);
end
