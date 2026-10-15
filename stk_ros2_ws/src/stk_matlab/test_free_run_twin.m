function test_free_run_twin()
%TEST_FREE_RUN_TWIN Free-run ZOH sub-stepping in adcs.simulink_step (E3).
%
% In free-run the bridge's clock follows the wall clock, so the twin
% receives samples at irregular times. It keeps an integer count k of
% steps already integrated since its first sample t0 and, at sample t,
% integrates n = floor((t - t0)/stepS + 1e-6) + 1 - k steps of stepS,
% holding the received position, velocity and target (zero-order hold);
% n = 0 holds the last outputs. Counting from t0 means rounding never
% accumulates. n > 1 occurrences and the largest n are counted.
% Lockstep (no execution_mode, or "lockstep") is unchanged.
    root = tempname;
    mkdir(root);
    cleanup = onCleanup(@() rmdir(root, 's'));
    pos = [6878137 0 0]; vel = [0 7500 0]; target = [6378137 0 0];
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

    % -- lockstep is unchanged, with or without the key -------------------
    a = runSequence('', (0:299) * cfg.stepS, pos, vel, target);
    b = runSequence(lock, (0:299) * cfg.stepS, pos, vel, target);
    assert(isequal(a, b), 'lockstep with execution_mode differs from without');

    % -- free-run on the step grid equals lockstep -----------------------
    c = runSequence(free, (0:299) * cfg.stepS, pos, vel, target);
    assert(isequal(a, c), 'free-run on the grid differs from lockstep');

    % -- irregular samples: sub-steps, holds, and the reference ----------
    times = [0 0.1 0.35 0.36 0.5 0.3 + 0.3];   % last one: 0.6 by addition
    expectedN = [1 1 2 0 2 1];
    [outs, stats] = runSequence(free, times, pos, vel, target);
    reference = manualSteps(cfg, 7, pos, vel, target);   % steps 0..6
    stepsDone = cumsum(expectedN);
    for k = 1:numel(times)
        if expectedN(k) == 0
            assert(isequal(outs{k}, outs{k-1}), 'n = 0 must hold the last outputs');
        else
            assert(norm(outs{k}.qCmd - reference{stepsDone(k)}.qCmd) < 1e-15, ...
                'sample %d: sub-stepped attitude differs from the reference', k);
        end
    end
    assert(stats.events == 2 && stats.max_n == 2, 'n > 1 statistics');
    assert(stats.samples == numel(times) && stats.steps == stepsDone(end));

    % -- a sample exactly on the grid is never lost to rounding ----------
    [~, stats] = runSequence(free, [0 0.1 0.2 0.3], pos, vel, target);
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
    runSequence(logged, times, pos, vel, target);
    written = jsondecode(fileread(fullfile(root, 'E3-x.logs', 'twin_substeps.json')));
    assert(written.events == 2 && written.max_n == 2);
    assert(written.min_stamp_offset_s <= 0 && written.min_stamp_offset_s > -cfg.stepS);
    clear adcs.simulink_step

    fprintf('PASS: free-run ZOH sub-stepping, holds, rounding, lockstep unchanged.\n');
end

function [outs, stats] = runSequence(configPath, times, pos, vel, target)
    clear adcs.simulink_step
    q = [0 0 0 1]; w = [0 0 0];
    outs = cell(1, numel(times));
    for k = 1:numel(times)
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

function reference = manualSteps(cfg, count, pos, vel, target)
    state = adcs.initial_state();
    q = [0 0 0 1]; w = [0 0 0];
    reference = cell(1, count);
    for j = 0:count-1
        in = struct('quaternion', q, 'omega', w, 'position', pos, ...
            'velocity', vel, 'targetEci', target, 'haveTarget', true, ...
            't', j * cfg.stepS, 'targetId', '');
        out = adcs.twin_step(in, cfg, state);
        state = out.state; q = out.quaternionCmd; w = out.omegaNext;
        reference{j+1} = struct('qCmd', q);
    end
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
