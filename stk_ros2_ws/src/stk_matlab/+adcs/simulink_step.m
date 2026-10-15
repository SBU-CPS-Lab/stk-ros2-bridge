function [torque, qCmd, mode, err, momentum, saturated, energy, ready, fresh, omegaCmd, ...
    identity, identityLength] = simulink_step(qMeas, omega, pos, vel, targetEci, ...
    haveTarget, t, reset, runConfigPath, targetId, buildFingerprint)
%SIMULINK_STEP MATLAB-hosted adapter for Normal-mode Simulink simulation.
% Keep dynamic profile structs and YAML/JSON loading in MATLAB; the
% Simulink boundary exposes only fixed-size numeric outputs. Reset is
% asserted by the block on every simulation start, preventing stale state.
%
% identity (uint8, 1x128, zero-padded; identityLength bytes used) is
% "v1 <cfg> <build> <runtime>", fixed at reset: the sha256 of the run
% config loaded ('-' for none), the code fingerprint written into the
% model at build time (buildFingerprint, '-' if absent) and the one
% recomputed from disk now, both cut to 16 hex digits. The model
% publishes it on /acav/twin_identity; acav/twin_identity.py checks it.
%
% Free-run (run config execution_mode "free_run", E3): samples arrive at
% irregular wall-clock-driven times. With t0 the first ready sample and
% k the steps integrated so far, a sample at t integrates
%   n = floor((t - t0)/stepS + 1e-6) + 1 - k
% steps of stepS; n = 0 holds the last outputs. Counting from t0
% means rounding never accumulates; the 1e-6 step tolerance keeps a
% sample on the grid (0.3/0.1 = 2.99...96 in floating point) on its
% step. Sub-step j runs at tj = t0 + j*stepS on the newest sample
% stamped at or before tj (within the same tolerance) -- the previous
% one, held samples included, while catching up -- with its position
% and target propagated forward to tj (adcs.geometry_at). Holding one
% sample's geometry over every sub-step instead pulses the controller's
% reference rate and costs captures (P2c replay, results/_analysis/p2c).
% A sample on the step grid is used as received (dt = 0), so on-grid
% free-run equals lockstep bit for bit. The state reached, t0 + (k+n)*stepS, lies in (t, t+stepS]; the
% command is still stamped t + stepS by the model, and the difference
% is recorded. n > 1 occurrences, the largest n and the most negative
% stamp offset go to <run>.logs/twin_substeps.json when they change;
% adcs.simulink_step('substep_statistics') returns them.
    persistent cfg state lastTime lastOutput identityBytes identityCount
    persistent freeT0 freeSteps freeStats freeStatsPath freeLast
    if ischar(qMeas) && strcmp(qMeas, 'substep_statistics')
        torque = freeStats;
        return
    end
    if nargin < 10, targetId = ''; end
    if nargin < 11, buildFingerprint = ''; end
    targetId = char(targetId(:).');
    targetId = targetId(targetId ~= char(0));
    if reset || isempty(cfg)
        cfg = adcs.config([], runConfigPath);
        state = adcs.initial_state();
        lastTime = []; lastOutput = {};
        [identityBytes, identityCount] = identityOf(cfg, buildFingerprint);
        freeT0 = []; freeSteps = 0; freeLast = [];
        freeStats = struct('samples', 0, 'steps', 0, 'events', 0, ...
            'max_n', 0, 'min_stamp_offset_s', 0);
        freeStatsPath = substepStatsPath(runConfigPath);
        if strcmp(cfg.attitudeMismatchPolicy, 'e1_readback_ablation')
            warning('adcs:simulink_step:E1Ablation', ...
                'E1 treatment: mismatch guard disabled; measured attitude drives feedback. Not a nominal run.');
        end
    end
    identity = identityBytes;
    identityLength = identityCount;
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
    if strcmp(cfg.executionMode, 'free_run')
        if ~isempty(lastTime) && t < lastTime
            error('adcs:simulink_step:TimeReversed', ...
                'Telemetry time moved backwards. Stop duplicate clock publishers and restart the run.');
        end
        if isempty(freeT0)
            freeT0 = t;
        end
        freeStats.samples = freeStats.samples + 1;
        reached = floor((t - freeT0) / cfg.stepS + 1e-6) + 1;
        n = reached - freeSteps;
        current = struct('t', t, 'position', pos, 'velocity', vel, ...
            'targetEci', targetEci, 'haveTarget', haveTarget, 'targetId', targetId);
        if n <= 0
            freeLast = current;
            [torque,qCmd,mode,err,momentum,saturated,energy,omegaCmd] = lastOutput{:};
            return
        end
        if isempty(lastOutput)
            q = qMeas; w = omega;
        else
            q = lastOutput{2}; w = lastOutput{8};
        end
        for j = freeSteps:(freeSteps + n - 1)
            tj = freeT0 + j * cfg.stepS;
            source = current;
            if ~isempty(freeLast) && t > tj + 1e-6 * cfg.stepS
                source = freeLast;   % newest sample stamped at or before tj
            end
            [p, v, g] = adcs.geometry_at(source.position, source.velocity, ...
                source.targetEci, tj - source.t, cfg);
            in = struct('quaternion', q, 'omega', w, ...
                'position', p, 'velocity', v, 'targetEci', g, ...
                'haveTarget', source.haveTarget, 't', tj, ...
                'targetId', source.targetId);
            out = adcs.twin_step(in, cfg, state);
            state = out.state;
            q = out.quaternionCmd; w = out.omegaNext;
        end
        freeLast = current;
        freeSteps = reached;
        freeStats.steps = freeSteps;
        offset = (freeT0 + reached * cfg.stepS) - (t + cfg.stepS);
        changed = false;
        if n > 1
            freeStats.events = freeStats.events + 1;
            changed = true;
            if n > freeStats.max_n || freeStats.events == 1 || ...
                    any(freeStats.events == 10.^(1:9))
                fprintf('adcs.simulink_step: free-run sub-steps n=%d at t=%.3f s (%d occurrences, max n %d)\n', ...
                    n, t, freeStats.events, max(n, freeStats.max_n));
            end
        end
        if n > freeStats.max_n
            freeStats.max_n = n;
            changed = true;
        end
        if offset < freeStats.min_stamp_offset_s
            freeStats.min_stamp_offset_s = offset;
            changed = true;
        end
        if changed && ~isempty(freeStatsPath)
            writeSubstepStats(freeStatsPath, freeStats);
        end
        fresh = true;
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

function path = substepStatsPath(runConfigPath)
%SUBSTEPSTATSPATH <run>.logs/twin_substeps.json beside the run config,
% or '' when that logs folder does not exist (e.g. a test or a manual run).
    path = '';
    runConfigPath = char(runConfigPath);
    suffix = '.runconfig.json';
    if numel(runConfigPath) > numel(suffix) && ...
            strcmp(runConfigPath(end-numel(suffix)+1:end), suffix)
        logs = [runConfigPath(1:end-numel(suffix)) '.logs'];
        if isfolder(logs)
            path = fullfile(logs, 'twin_substeps.json');
        end
    end
end

function writeSubstepStats(path, stats)
    fid = fopen(path, 'w');
    if fid < 0
        return
    end
    fwrite(fid, uint8([jsonencode(stats) 10]), 'uint8');
    fclose(fid);
end

function [bytes, count] = identityOf(cfg, buildFingerprint)
    config = cfg.runConfigSha256;
    if isempty(config), config = '-'; end
    build = char(buildFingerprint);
    if numel(build) < 16
        build = '-';
    else
        build = build(1:16);
    end
    runtime = adcs.code_fingerprint();
    text = uint8(sprintf('v1 %s %s %s', config, build, runtime(1:16)));
    bytes = zeros(1, 128, 'uint8');
    count = numel(text);
    bytes(1:count) = text;
end
