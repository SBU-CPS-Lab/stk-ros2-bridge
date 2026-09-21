function [torque, qCmd, mode, err, momentum, saturated, energy, ready, fresh] = ...
    simulink_step(qMeas, omega, pos, vel, targetEci, haveTarget, t, reset, runConfigPath)
%SIMULINK_STEP MATLAB-hosted adapter for Normal-mode Simulink simulation.
% Keep dynamic profile structs and YAML/JSON loading in MATLAB; the
% Simulink boundary exposes only fixed-size numeric outputs. Reset is
% asserted by the block on every simulation start, preventing stale state.
    persistent cfg state lastTime lastOutput
    if reset || isempty(cfg)
        cfg = adcs.config([], runConfigPath);
        state = adcs.initial_state();
        lastTime = []; lastOutput = {};
    end
    % ROS subscribers initially output a zero-filled bus. Do not advance
    % dynamics or acknowledge a step until all required numerics are valid.
    ready = all(isfinite([qMeas(:); omega(:); pos(:); vel(:); t])) && ...
        norm(qMeas) > 1e-12 && norm(pos) > 1e-12 && norm(vel) > 1e-12;
    if haveTarget
        ready = ready && all(isfinite(targetEci(:))) && ...
            norm(targetEci(:)-pos(:)) > 1e-12;
    end
    fresh = false;
    if ~ready
        torque = zeros(1,3); qCmd = zeros(1,4); mode = 1;
        err = 0; momentum = state.wheelMomentum;
        saturated = false; energy = state.wheelEnergyJ;
        return
    end
    if ~isempty(lastTime) && t <= lastTime
        if t < lastTime
            error('adcs:simulink_step:TimeReversed', ...
                'Telemetry time moved backwards. Stop duplicate clock publishers and restart the run.');
        end
        [torque,qCmd,mode,err,momentum,saturated,energy] = lastOutput{:};
        return
    end
    fresh = true;
    in = struct('quaternion', qMeas, 'omega', omega, ...
        'position', pos, 'velocity', vel, 'targetEci', targetEci, ...
        'haveTarget', haveTarget, 't', t);
    out = adcs.twin_step(in, cfg, state);
    state = out.state;
    torque = out.torque;
    qCmd = out.quaternionCmd;
    names = {'NADIR', 'SLEW', 'SETTLE', 'IMAGE', 'SAFE'};
    mode = double(find(strcmp(names, state.mode), 1));
    err = out.boresightErrorRad;
    momentum = state.wheelMomentum;
    saturated = logical(out.wheelsSaturated);
    energy = state.wheelEnergyJ;
    lastTime = t;
    lastOutput = {torque,qCmd,mode,err,momentum,saturated,energy};
end
