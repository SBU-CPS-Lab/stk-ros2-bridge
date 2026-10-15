function test_free_run_twin()
%TEST_FREE_RUN_TWIN Free-run ZOH sub-stepping in adcs.simulink_step (E3).
%
% In free-run the bridge's clock follows the wall clock, so the twin
% receives samples at irregular times. It keeps an integer count k of
% steps already integrated since its first sample t0 and, at sample t,
% integrates n = floor((t - t0)/stepS + 1e-6) + 1 - k steps of stepS;
% n = 0 holds the last outputs. Counting from t0 means rounding never
% accumulates. n > 1 occurrences and the largest n are counted.
% Each sub-step at tj = t0 + j*stepS is driven by the newest sample
% stamped at or before tj (the previous one while catching up), its
% geometry propagated forward to tj by adcs.geometry_at.
% Lockstep (no execution_mode, or "lockstep") is unchanged.
    root = tempname;
    mkdir(root);
    cleanup = onCleanup(@() rmdir(root, 's'));
    pos = [6878137 0 0]; vel = [0 7500 0]; target = [6378137 0 0];
    still = @(t) deal(pos, vel, target);
    cfg = adcs.config();

    % -- configuration ---------------------------------------------------
    free = writeConfig(root, 'free', '{"run_id": "free", "execution_mode": "free_run"}');
    lock = writeConfig(root, 'lock', '{"run_id": "lock", "execution_mode": "lockstep"}');
    assert(strcmp(adcs.config([], free).executionMode, 'free_run'));
    assert(strcmp(adcs.config([], lock).executionMode, 'lockstep'));
    assert(strcmp(adcs.config().executionMode, 'lockstep'));
    bad = writeConfig(root, 'bad', '{"run_id": "bad", "execution_mode": "paced"}');
    assertError(@() adcs.config([], bad), 'adcs:config:ExecutionMode');
    e1 = writeConfig(root, 'e1', ['{"run_id": "e1", "execution_mode": "free_run", ' ...
        '"attitude_mismatch_policy": "e1_readback_ablation", "experiment": "E1", ' ...
        '"attitude_provider": "ECF Attitude Quaternions"}']);
    assertError(@() adcs.config([], e1), 'adcs:config:ExecutionMode');

    % -- the Earth rate has one source, and targets must be Earth-fixed ---
    assert(cfg.earthRateRadS == 7.292115e-5, 'Earth rate is the WGS 84 value');
    assertEarthRateDefinedOnce();
    assertTargetsMustBeEarthFixed(root);

    % -- adcs.geometry_at -------------------------------------------------
    p = [6878137.25 -1.5e-3 3.1e5]; v = [1.25 7500.5 -0.75]; g = [6378137 1e-9 -2.5];
    [p0, v0, g0] = adcs.geometry_at(p, v, g, 0, cfg);
    assert(isequal(p0, p) && isequal(v0, v) && isequal(g0, g), ...
        'dt = 0 must return the inputs bit-identically');
    dt = 0.37;
    [p1, v1, g1] = adcs.geometry_at(p, v, g, dt, cfg);
    assert(isequal(p1, p + v * dt), 'position: p + v dt');
    assert(isequal(v1, v), 'velocity is held');
    assert(isequal(g1, g + cross([0 0 cfg.earthRateRadS], g) * dt), ...
        'target: rotated with the Earth about z');

    % -- lockstep is unchanged, with or without the key -------------------
    a = runSequence('', (0:299) * cfg.stepS, still);
    b = runSequence(lock, (0:299) * cfg.stepS, still);
    assert(isequal(a, b), 'lockstep with execution_mode differs from without');

    % -- free-run on the step grid equals lockstep -----------------------
    c = runSequence(free, (0:299) * cfg.stepS, still);
    assert(isequal(a, c), 'free-run on the grid differs from lockstep');
    orbit = @(t) orbitGeometry(t);
    d = runSequence(lock, (0:299) * cfg.stepS, orbit);
    e = runSequence(free, (0:299) * cfg.stepS, orbit);
    assert(isequal(d, e), 'free-run on the grid differs from lockstep (moving geometry)');

    % -- irregular samples: sub-steps, holds, and the reference ----------
    % 0.35 catches up steps 2 and 3 from the 0.1 sample; the 0.36 hold
    % becomes the source of step 4 at the 0.5 sample.
    times = [0 0.1 0.35 0.36 0.5 0.3 + 0.3];   % last one: 0.6 by addition
    expectedN = [1 1 2 0 2 1];
    [outs, stats] = runSequence(free, times, orbit);
    reference = manualSteps(cfg, times, orbit, true);
    held = manualSteps(cfg, times, orbit, false);
    stepsDone = cumsum(expectedN);
    for k = 1:numel(times)
        if expectedN(k) == 0
            assert(isequal(outs{k}, outs{k-1}), 'n = 0 must hold the last outputs');
        else
            assert(norm(outs{k}.qCmd - reference{stepsDone(k)}.qCmd) < 1e-15, ...
                'sample %d: sub-stepped attitude differs from the reference', k);
        end
    end
    assert(norm(outs{5}.qCmd - held{stepsDone(5)}.qCmd) > 1e-9, ...
        'the reference must differ from held geometry, or this test proves nothing');
    assert(stats.events == 2 && stats.max_n == 2, 'n > 1 statistics');
    assert(stats.samples == numel(times) && stats.steps == stepsDone(end));

    % -- a sample exactly on the grid is never lost to rounding ----------
    [~, stats] = runSequence(free, [0 0.1 0.2 0.3], still);
    assert(stats.events == 0 && stats.steps == 4, '0.3/0.1 must count as step 3');

    % -- time still may not go backwards ---------------------------------
    clear adcs.simulink_step
    step(free, 0.0, true, pos, vel, target, [0 0 0 1], [0 0 0]);
    step(free, 0.2, false, pos, vel, target, [0 0 0 1], [0 0 0]);
    assertError(@() step(free, 0.1, false, pos, vel, target, [0 0 0 1], [0 0 0]), ...
        'adcs:simulink_step:TimeReversed');

    % -- statistics file beside a run's logs ------------------------------
    logged = writeConfig(root, 'E3-x', '{"run_id": "E3-x", "execution_mode": "free_run"}');
    mkdir(fullfile(root, 'E3-x.logs'));
    runSequence(logged, times, still);
    written = jsondecode(fileread(fullfile(root, 'E3-x.logs', 'twin_substeps.json')));
    assert(written.events == 2 && written.max_n == 2);
    assert(written.min_stamp_offset_s <= 0 && written.min_stamp_offset_s > -cfg.stepS);
    clear adcs.simulink_step

    fprintf(['PASS: free-run ZOH sub-stepping with causal propagated geometry, ' ...
             'holds, rounding, Earth rate, lockstep unchanged.\n']);
end

function [pos, vel, target] = orbitGeometry(t)
%ORBITGEOMETRY A circular orbit and an Earth-fixed target: curved enough
% that propagating the previous sample and the current one differ.
    a = 6878137; n = sqrt(3.986004418e14 / a^3); wE = 7.292115e-5;
    pos = a * [cos(n*t), 0.13 * sin(n*t), 0.99 * sin(n*t)];
    vel = a * n * [-sin(n*t), 0.13 * cos(n*t), 0.99 * cos(n*t)];
    target = 6378137 * [cos(wE*t + 0.02), sin(wE*t + 0.02), 0.05];
end

function [outs, stats] = runSequence(configPath, times, geometry)
    clear adcs.simulink_step
    q = [0 0 0 1]; w = [0 0 0];
    outs = cell(1, numel(times));
    for k = 1:numel(times)
        [pos, vel, target] = geometry(times(k));
        o = step(configPath, times(k), k == 1, pos, vel, target, q, w);
        outs{k} = o;
        if any(o.qCmd), q = o.qCmd; w = o.omegaCmd; end
    end
    stats = adcs.simulink_step('substep_statistics');
    clear adcs.simulink_step
end

function o = step(configPath, t, reset, pos, vel, target, q, w)
    [o.torque, o.qCmd, o.mode, o.err, o.momentum, o.saturated, o.energy, ...
        o.ready, ~, o.omegaCmd] = adcs.simulink_step(q, w, pos, vel, ...
        target, true, t, reset, configPath, '');
end

function reference = manualSteps(cfg, times, geometry, propagate)
%MANUALSTEPS Steps 0..K at tj = j*stepS (times(1) = 0), written out
% independently of adcs.simulink_step. propagate: each step uses the
% newest sample stamped at or before tj, moved to tj (p + v dt, target
% rotated with the Earth); otherwise the geometry of the sample that
% triggered the step is held.
    state = adcs.initial_state();
    q = [0 0 0 1]; w = [0 0 0];
    reference = {};
    done = 0;
    for k = 1:numel(times)
        reached = floor(times(k) / cfg.stepS + 1e-6) + 1;
        for j = done:reached-1
            tj = j * cfg.stepS;
            if propagate
                s = find(times(1:k) <= tj + 1e-6 * cfg.stepS, 1, 'last');
                [p, v, g] = geometry(times(s));
                ds = tj - times(s);
                if ds ~= 0
                    p = p + v * ds;
                    g = g + cross([0 0 cfg.earthRateRadS], g) * ds;
                end
            else
                [p, v, g] = geometry(times(k));
            end
            in = struct('quaternion', q, 'omega', w, 'position', p, ...
                'velocity', v, 'targetEci', g, 'haveTarget', true, ...
                't', tj, 'targetId', '');
            out = adcs.twin_step(in, cfg, state);
            state = out.state; q = out.quaternionCmd; w = out.omegaNext;
            reference{j+1} = struct('qCmd', q); %#ok<AGROW>
        end
        done = max(done, reached);
    end
end

function assertEarthRateDefinedOnce()
%ASSERTEARTHRATEDEFINEDONCE The twin's code spells the Earth rate in one
% place (adcs.config); everything else reads cfg.earthRateRadS.
    here = fileparts(mfilename('fullpath'));
    files = dir(fullfile(here, '+adcs', '**', '*.m'));
    holders = {};
    for k = 1:numel(files)
        text = fileread(fullfile(files(k).folder, files(k).name));
        if contains(text, '7.2921')
            holders{end+1} = files(k).name; %#ok<AGROW>
        end
    end
    assert(isequal(holders, {'config.m'}), ...
        'Earth rate literal found in: %s', strjoin(holders, ', '));
end

function assertTargetsMustBeEarthFixed(root)
    here = fileparts(mfilename('fullpath'));
    shipped = fileread(fullfile(fileparts(here), 'acav', 'config', 'acav_scenario.yaml'));
    % The targets' placement, not the ground station's, is what counts.
    targetLine = '  placement: ground_track_relative';
    first = strfind(shipped, targetLine);
    assert(numel(first) == 2, 'expected the targets'' and the station''s placement');
    moved = [shipped(1:first(1)-1) '  placement: inertial' ...
        shipped(first(1)+numel(targetLine):end)];
    assertError(@() adcs.config(writeText(root, 'inertial.yaml', moved)), ...
        'adcs:config:TargetsNotEarthFixed');
    station = [shipped(1:first(2)-1) '  placement: inertial' ...
        shipped(first(2)+numel(targetLine):end)];
    adcs.config(writeText(root, 'station.yaml', station));
    unnamed = [shipped(1:first(1)-1) shipped(first(1)+numel(targetLine):end)];
    assertError(@() adcs.config(writeText(root, 'unnamed.yaml', unnamed)), ...
        'adcs:config:TargetsNotEarthFixed');
    moon = strrep(shipped, 'central_body: Earth', 'central_body: Moon');
    assertError(@() adcs.config(writeText(root, 'moon.yaml', moon)), ...
        'adcs:config:TargetsNotEarthFixed');
end

function path = writeText(root, name, text)
    path = fullfile(root, name);
    fid = fopen(path, 'w'); fwrite(fid, unicode2native(text, 'UTF-8'), 'uint8'); fclose(fid);
end

function path = writeConfig(root, name, text)
    path = fullfile(root, [name '.runconfig.json']);
    fid = fopen(path, 'w'); fwrite(fid, uint8([text 10]), 'uint8'); fclose(fid);
end

function assertError(f, id)
    try
        f();
    catch err
        assert(strcmp(err.identifier, id), err.message);
        return
    end
    error('Expected %s', id);
end
