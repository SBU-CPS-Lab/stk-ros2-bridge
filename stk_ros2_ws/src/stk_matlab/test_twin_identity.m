function test_twin_identity()
%TEST_TWIN_IDENTITY The twin's code fingerprint and run-config digest.
%
% adcs.code_fingerprint must equal acav/twin_identity.py's
% code_fingerprint for the same files: both suites pin this vector (see
% src/acav/test/test_twin_identity.py). adcs.config must report the
% sha256 of the exact run-config bytes it parsed, so the runner can tell
% which config the twin actually loaded.
    vector = 'aecfd2825eaca8f111a228dab00853ec6b496cea8dd98f52773cf3156e476daa';
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
