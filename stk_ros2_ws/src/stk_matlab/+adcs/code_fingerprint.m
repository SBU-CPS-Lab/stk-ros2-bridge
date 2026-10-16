function fingerprint = code_fingerprint(srcRoot)
%CODE_FINGERPRINT sha256 identifying the twin's code on disk.
%
%   fp = adcs.code_fingerprint()          % this checkout's src directory
%   fp = adcs.code_fingerprint(srcRoot)
%
% A sha256 over "<path>\0<sha256 of the file>\n" lines, sorted by path,
% for +adcs/**/*.m, simulink/build_acav_twin.m, the scenario YAML the
% twin reads and the stk_msgs definitions its bus types come from.
% acav/twin_identity.py computes the same: both test suites pin one
% vector (test_twin_identity.m). Each file's text is hashed with CRLF
% turned into LF (a lone CR is kept), so a checkout's line endings
% (core.autocrlf) do not change the fingerprint. build_acav_twin writes it into the
% model; adcs.simulink_step recomputes it at simulation start, so a
% model built from other code is visible to the run orchestrator.
    if nargin < 1 || isempty(srcRoot)
        srcRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    end
    % Spelled the way dir() reports folders (e.g. long rather than 8.3
    % names), so relative paths can be cut from dir's own results.
    listing = dir(char(srcRoot));
    if isempty(listing)
        error('adcs:code_fingerprint:Missing', 'no folder %s', srcRoot);
    end
    srcRoot = listing(1).folder;

    relative = [ ...
        listFiles(srcRoot, fullfile('stk_matlab', '+adcs', '**', '*.m')), ...
        listFiles(srcRoot, fullfile('stk_matlab', 'simulink', 'build_acav_twin.m')), ...
        listFiles(srcRoot, fullfile('acav', 'config', 'acav_scenario.yaml')), ...
        listFiles(srcRoot, fullfile('stk_msgs', 'msg', '*.msg'))];
    for required = {'stk_matlab/simulink/build_acav_twin.m', ...
                    'acav/config/acav_scenario.yaml'}
        if ~any(strcmp(relative, required{1}))
            error('adcs:code_fingerprint:Missing', ...
                '%s not found under %s', required{1}, srcRoot);
        end
    end
    relative = sort(unique(relative));

    lines = uint8([]);
    for k = 1:numel(relative)
        bytes = readBytes(fullfile(srcRoot, relative{k}));
        lines = [lines, uint8(relative{k}), uint8(0), ...
            uint8(adcs.internal.sha256_hex(bytes)), uint8(10)]; %#ok<AGROW>
    end
    fingerprint = adcs.internal.sha256_hex(lines);
end

function relative = listFiles(srcRoot, pattern)
    entries = dir(fullfile(srcRoot, pattern));
    entries = entries(~[entries.isdir]);
    relative = cell(1, numel(entries));
    for k = 1:numel(entries)
        full = fullfile(entries(k).folder, entries(k).name);
        relative{k} = strrep(full(numel(srcRoot) + 2:end), '\', '/');
    end
end

function bytes = readBytes(path)
    fid = fopen(path, 'r');
    if fid < 0
        error('adcs:code_fingerprint:Unreadable', 'cannot read %s', path);
    end
    closer = onCleanup(@() fclose(fid));
    bytes = fread(fid, inf, '*uint8').';
    crlf = find(bytes(1:end-1) == 13 & bytes(2:end) == 10);
    bytes(crlf) = [];
end
