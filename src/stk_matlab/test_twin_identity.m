function test_twin_identity()
%TEST_TWIN_IDENTITY The twin's code fingerprint and run-config digest.
%
% adcs.code_fingerprint must equal acav/twin_identity.py's
% code_fingerprint for the same files: both suites pin this vector (see
% src/acav/test/test_twin_identity.py). adcs.config must report the
% sha256 of the exact run-config bytes it parsed, so the runner can tell
% which config the twin actually loaded.
%
% The fingerprint hashes file text with CRLF turned into LF, so a.m's
% CRLF below hashes as LF, and line endings never change it.
    vector = 'c241417d97fa57b740d30e77ee539ddc7d913076acfc4e909f200ca6e65c5ec5';
    root = tempname;
    cleanup = onCleanup(@() rmdir(root, 's'));
    files = {
        'stk_matlab/+adcs/a.m',                  uint8(['a' 13 10])
        'stk_matlab/+adcs/+internal/b.m',        uint8('b')
        'stk_matlab/simulink/build_acav_twin.m', uint8('build')
        'acav/config/acav_scenario.yaml',        uint8(['step_s: 0.1' 10])
        'stk_msgs/msg/X.msg',                    uint8(['int32 x' 10])
        % Decoys: not the twin's code.
        'stk_matlab/+adcs/notes.txt',            uint8('notes')
        'stk_matlab/other.m',                    uint8('other')
        'stk_matlab/simulink/build_twin_template.m', uint8('template')
        'acav/config/other.yaml',                uint8(['x: 1' 10])
        };
    for k = 1:size(files, 1)
        writeBytes(fullfile(root, files{k, 1}), files{k, 2});
    end

    assert(strcmp(adcs.code_fingerprint(root), vector), ...
        'code_fingerprint differs from the shared vector');

    % The same file with LF instead of CRLF: the same fingerprint. A lone
    % CR is not a line ending and is kept.
    writeBytes(fullfile(root, 'stk_matlab/+adcs/a.m'), uint8(['a' 10]));
    assert(strcmp(adcs.code_fingerprint(root), vector), ...
        'line endings must not change the fingerprint');
    writeBytes(fullfile(root, 'stk_matlab/+adcs/a.m'), uint8(['a' 13]));
    assert(~strcmp(adcs.code_fingerprint(root), vector), ...
        'a lone CR is content');
    writeBytes(fullfile(root, 'stk_matlab/+adcs/a.m'), uint8(['a' 13 10]));

    writeBytes(fullfile(root, 'stk_matlab/+adcs/+internal/b.m'), uint8('c'));
    assert(~strcmp(adcs.code_fingerprint(root), vector), ...
        'a changed file must change the fingerprint');

    delete(fullfile(root, 'acav/config/acav_scenario.yaml'));
    assertError(@() adcs.code_fingerprint(root), ...
        'adcs:code_fingerprint:Missing');

    % This checkout has one, and it covers the adapter.
    assert(numel(adcs.code_fingerprint()) == 64);

    % The run-config digest is of the bytes parsed.
    configPath = fullfile(root, 'r.runconfig.json');
    writeBytes(configPath, uint8(['{"run_id": "r"}' 10]));
    cfg = adcs.config([], configPath);
    assert(strcmp(cfg.runConfigSha256, ...
        'ea0ab5a1b67c36408f52f019e011c48fef54dad1f331ff35d7ec1261bc355fbb'), ...
        'run-config digest differs from sha256 of its bytes');
    assert(strcmp(cfg.runId, 'r'));
    nominal = adcs.config();
    assert(isempty(nominal.runConfigSha256) && isempty(nominal.runId));

    % Parsing is unchanged: gains still apply.
    writeBytes(configPath, uint8(['{"run_id": "g", "gains": ' ...
        '{"natural_frequency_rad_s": 1.2, "damping_ratio": 0.6}}' 10]));
    cfg = adcs.config([], configPath);
    assert(cfg.naturalFrequencyRadS == 1.2 && cfg.dampingRatio == 0.6);

    fprintf('PASS: code fingerprint vector, run-config digest.\n');

    % -- the identity the model publishes (adcs.simulink_step) ---------
    identityPath = fullfile(root, 'id.runconfig.json');
    writeBytes(identityPath, uint8(['{"run_id": "r"}' 10]));
    built = adcs.code_fingerprint();
    zero3 = zeros(1, 3);
    % v2 added the polling rate the model was built with (Simulink pacing,
    % build_acav_twin's PollingRateHz); v3 marks the fingerprint that
    % ignores line endings.
    [~,~,~,~,~,~,~,ready,~,~,identity,n,due] = adcs.simulink_step( ...
        zeros(1,4), zero3, zero3, zero3, zero3, false, 0, true, ...
        identityPath, '', built, 100);
    assert(~ready, 'startup sample must not be ready');
    assert(~due, 'the identity is not due before the twin is ready');
    assert(isa(identity, 'uint8') && isequal(size(identity), [1 128]));
    expected = sprintf('v3 %s %s %s 100Hz', ...
        'ea0ab5a1b67c36408f52f019e011c48fef54dad1f331ff35d7ec1261bc355fbb', ...
        built(1:16), built(1:16));
    assert(n == 107 && strcmp(char(identity(1:n)), expected), ...
        'identity text: %s', char(identity(1:n)));
    assert(all(identity(n+1:end) == 0));

    % A model built from other code reports that build fingerprint.
    [~,~,~,~,~,~,~,~,~,~,identity,n] = adcs.simulink_step( ...
        zeros(1,4), zero3, zero3, zero3, zero3, false, 0, true, ...
        identityPath, '', repmat('f', 1, 64), 100);
    assert(strcmp(char(identity(69:n)), ...
        sprintf('%s %s 100Hz', repmat('f', 1, 16), built(1:16))));

    % No run config, no build fingerprint, no polling rate: '-' fields
    % (the runner refuses the first two).
    [~,~,~,~,~,~,~,~,~,~,identity,n] = adcs.simulink_step( ...
        zeros(1,4), zero3, zero3, zero3, zero3, false, 0, true, '');
    assert(strcmp(char(identity(1:n)), sprintf('v3 - - %s -', built(1:16))));

    % A fractional rate is written without trailing zeros.
    [~,~,~,~,~,~,~,~,~,~,identity,n] = adcs.simulink_step( ...
        zeros(1,4), zero3, zero3, zero3, zero3, false, 0, true, ...
        identityPath, '', built, 12.5);
    assert(endsWith(char(identity(1:n)), ' 12.5Hz'));

    % The identity is fixed at reset: later calls keep it.
    adcs.simulink_step(zeros(1,4), zero3, zero3, zero3, zero3, false, ...
        0, true, identityPath, '', built, 100);
    writeBytes(identityPath, uint8(['{"run_id": "changed"}' 10]));
    [~,~,~,~,~,~,~,~,~,~,identity,n] = adcs.simulink_step( ...
        zeros(1,4), zero3, zero3, zero3, zero3, false, 0.1, false, ...
        identityPath, '', built, 100);
    assert(strcmp(char(identity(1:n)), expected));
    clear adcs.simulink_step

    % -- when the identity is published ----------------------------------
    % Once the twin is ready, then at most once per wall second (the run
    % orchestrator subscribes after the twin acks and waits up to 10 s);
    % again at once after a reset. The ack is not throttled.
    writeBytes(identityPath, uint8(['{"run_id": "r"}' 10]));
    pos = [6878137 0 0]; vel = [0 7500 0]; target = [6378137 0 0];
    dueAt = @(t, reset) dueOf(identityPath, built, pos, vel, target, t, reset);
    assert(dueAt(0.0, true), 'due on the first ready step');
    assert(~dueAt(0.1, false), 'not due again within a second');
    pause(1.05);
    assert(dueAt(0.2, false), 'due again after a wall second');
    assert(~dueAt(0.3, false));
    assert(dueAt(0.0, true), 'due at once after a reset');
    clear adcs.simulink_step

    fprintf('PASS: twin identity output.\n');
end

function due = dueOf(configPath, built, pos, vel, target, t, reset)
    persistent q w
    if reset || isempty(q), q = [0 0 0 1]; w = [0 0 0]; end
    [~,qCmd,~,~,~,~,~,ready,~,omegaCmd,~,~,due] = adcs.simulink_step( ...
        q, w, pos, vel, target, true, t, reset, configPath, '', built, 100);
    assert(ready);
    q = qCmd; w = omegaCmd;
end

function writeBytes(path, bytes)
    folder = fileparts(path);
    if ~isfolder(folder), mkdir(folder); end
    fid = fopen(path, 'w');
    fwrite(fid, bytes, 'uint8');
    fclose(fid);
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
