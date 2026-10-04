function [torque, qCmd, mode, err, momentum, saturated, energy, ready, fresh, omegaCmd] = ...
    simulink_step(qMeas, omega, pos, vel, targetEci, haveTarget, t, reset, runConfigPath, targetId)
%SIMULINK_STEP MATLAB-hosted adapter for Normal-mode Simulink simulation.
% Keep dynamic profile structs and YAML/JSON loading in MATLAB; the
% Simulink boundary exposes only fixed-size numeric outputs. Reset is
% asserted by the block on every simulation start, preventing stale state.
    if nargin < 10, targetId = ''; end
    targetId = char(targetId(:).');
    targetId = targetId(targetId ~= char(0));
    persistent cfg state lastTime lastOutput
    if reset || isempty(cfg)
        cfg = adcs.config([], runConfigPath);
        state = adcs.initial_state();
        lastTime = []; lastOutput = {};
        if strcmp(cfg.attitudeMismatchPolicy, 'e1_readback_ablation')
            warning('adcs:simulink_step:E1Ablation', ...
                'E1 treatment: mismatch guard disabled; measured attitude drives feedback. Not a nominal run.');
        end
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
        torque = zeros(1,3); qCmd = zeros(1,4); omegaCmd = zeros(1,3); mode = 1;
        err = 0; momentum = state.wheelMomentum;
        saturated = false; energy = state.wheelEnergyJ;
        return
    end
    if ~isempty(lastTime) && t <= lastTime
        if t < lastTime
            error('adcs:simulink_step:TimeReversed', ...
                'Telemetry time moved backwards. Stop duplicate clock publishers and restart the run.');
        end
        [torque,qCmd,mode,err,momentum,saturated,energy,omegaCmd] = lastOutput{:};
        return
    end
    if ~isempty(lastTime)
        % Continue both dynamics states from the same integration endpoint.
        % STK differentiates injected quaternions: that interval rate must
        % not replace the twin's instantaneous omegaNext.
        if abs((t-lastTime)-cfg.stepS) > max(1e-6, cfg.stepS*1e-5)
            error('adcs:simulink_step:TimeGap', ...
                'Expected the next lockstep sample after %.6g s; received %.6g s.', ...
                cfg.stepS, t-lastTime);
        end
        qEndpoint = lastOutput{2};
        qRead = qMeas(:)'/norm(qMeas);
        residual = min(norm(qRead-qEndpoint), norm(qRead+qEndpoint));
        ablation = strcmp(cfg.attitudeMismatchPolicy, 'e1_readback_ablation');
        if residual > 5e-5 && ~ablation % approximately 1e-4 rad disagreement
            error('adcs:simulink_step:AttitudeMismatch', ...
                ['STK attitude does not match the previous achieved command ' ...
                 '(quaternion residual %.6g). Check timestamps and reference frames.'], residual);
        end
        if ~ablation
            qMeas = qEndpoint;
        end
        omega = lastOutput{8};
    end
    fresh = true;
    in = struct('quaternion', qMeas, 'omega', omega, ...
        'position', pos, 'velocity', vel, 'targetEci', targetEci, ...
        'haveTarget', haveTarget, 't', t, 'targetId', targetId);
    out = adcs.twin_step(in, cfg, state);
    state = out.state;
    torque = out.torque;
    qCmd = out.quaternionCmd;
    omegaCmd = out.omegaNext;
    names = {'NADIR', 'SLEW', 'SETTLE', 'IMAGE', 'SAFE'};
    mode = double(find(strcmp(names, state.mode), 1));
    err = out.boresightErrorRad;
    momentum = state.wheelMomentum;
    saturated = logical(out.wheelsSaturated);
    energy = state.wheelEnergyJ;
    lastTime = t;
    lastOutput = {torque,qCmd,mode,err,momentum,saturated,energy,omegaCmd};
end
