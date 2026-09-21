function test_simulink_adapter()
%TEST_SIMULINK_ADAPTER Check the Normal-mode boundary against twin_step.
    cfg = adcs.config();
    state = adcs.initial_state();
    q = [0 0 0 1]; omega = [0 0 0];
    pos = [6878137 0 0]; vel = [0 7500 0]; target = [6378137 0 0];
    names = {'NADIR','SLEW','SETTLE','IMAGE','SAFE'};
    % Startup defaults and invalid samples must neither throw nor advance
    % the controller. The first valid sample must match a fresh twin.
    [~,~,~,~,h,~,energy,ready] = adcs.simulink_step( ...
        zeros(1,4),zeros(1,3),zeros(1,3),zeros(1,3),zeros(1,3),false,0,true,'');
    assert(~ready && all(h==0) && energy==0);
    for k = 1:12
        t = (k-1)*cfg.stepS;
        in = struct('quaternion',q,'omega',omega,'position',pos, ...
            'velocity',vel,'targetEci',target,'haveTarget',true,'t',t);
        expected = adcs.twin_step(in,cfg,state);
        if k == 5
            [~,~,~,~,h,~,e,valid] = adcs.simulink_step( ...
                [NaN 0 0 1],omega,pos,vel,target,true,t,false,'');
            assert(~valid && isequal(h,state.wheelMomentum) && e==state.wheelEnergyJ);
        end
        [torque,qCmd,mode,err,momentum,saturated,energy,ready] = ...
            adcs.simulink_step(q,omega,pos,vel,target,true,t,false,'');
        assert(ready);
        assert(norm(torque-expected.torque)<1e-12);
        assert(norm(qCmd-expected.quaternionCmd)<1e-12);
        assert(strcmp(names{mode},expected.state.mode));
        assert(abs(err-expected.boresightErrorRad)<1e-12);
        assert(norm(momentum-expected.state.wheelMomentum)<1e-12);
        assert(saturated==expected.wheelsSaturated);
        assert(abs(energy-expected.state.wheelEnergyJ)<1e-12);
        [~,repeatQ,~,~,repeatH,~,repeatEnergy,valid,fresh] = ...
            adcs.simulink_step(q,omega,pos,vel,target,true,t,false,'');
        assert(valid && ~fresh && isequal(repeatQ,qCmd));
        assert(isequal(repeatH,momentum) && repeatEnergy==energy);
        state = expected.state; q = qCmd; omega = expected.omegaNext;
    end
    [~,first] = adcs.simulink_step([0 0 0 1],[0 0 0],pos,vel,target,true,0,true,'');
    [~,again] = adcs.simulink_step([0 0 0 1],[0 0 0],pos,vel,target,true,0,true,'');
    assert(isequal(first,again),'Simulation restart must reset controller state.');
    fprintf('PASS: zero startup and invalid telemetry are gated; 12 steps match; reset works.\n');
end
