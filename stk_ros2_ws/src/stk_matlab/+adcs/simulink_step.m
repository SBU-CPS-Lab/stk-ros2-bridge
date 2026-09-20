function [torque, qCmd, mode, err, momentum, saturated, energy, ready] = ...
    simulink_step(qMeas, omega, pos, vel, targetEci, haveTarget, t, reset, runConfigPath)
%SIMULINK_STEP MATLAB-hosted adapter for Normal-mode Simulink simulation.
% Keep dynamic profile structs and YAML/JSON loading in MATLAB; the
% Simulink boundary exposes only fixed-size numeric outputs. Reset is
% asserted by the block on every simulation start, preventing stale state.
    persistent cfg state
    if reset || isempty(cfg)
        cfg = adcs.config([], runConfigPath);
        state = adcs.initial_state();
    end
    % ROS subscribers initially output a zero-filled bus. Do not advance
    % dynamics or acknowledge a step until all required numerics are valid.
    ready = all(isfinite([qMeas(:); omega(:); pos(:); vel(:); t])) && ...
        norm(qMeas) > 1e-12 && norm(pos) > 1e-12 && norm(vel) > 1e-12;
    if haveTarget
        ready = ready && all(isfinite(targetEci(:))) && ...
            norm(targetEci(:)-pos(:)) > 1e-12;
    end
    if ~ready
        torque = zeros(1,3); qCmd = zeros(1,4); mode = 1;
        err = 0; momentum = state.wheelMomentum;
        saturated = false; energy = state.wheelEnergyJ;
        return
    end
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
end
