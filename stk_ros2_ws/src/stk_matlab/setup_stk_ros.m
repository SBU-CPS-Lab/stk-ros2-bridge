function setup_stk_ros(varargin)
%SETUP_STK_ROS Configure MATLAB to talk to the stk_bridge ROS 2 graph.
%
%   setup_stk_ros() adds this folder to the MATLAB path, generates the
%   custom message classes for stk_msgs (if not already generated), and
%   verifies the ROS 2 network can be reached.
%
%   setup_stk_ros('WorkspacePath', P) points at a specific colcon
%   workspace root instead of inferring it from this file's location.
%
% PREREQUISITES (see docs/02_matlab_setup.md for the full walkthrough):
%   1. ROS Toolbox installed, MATLAB release matched to ROS 2 Humble
%      (R2023b or R2024a -- see MathWorks' ROS Toolbox System
%      Requirements; a mismatched distro produces custom-message
%      failures that look like network problems, not version problems).
%   2. On Windows, ros2genmsg needs an MSVC toolchain discoverable by
%      MATLAB: install Visual Studio Build Tools with the "Desktop
%      development with C++" workload FIRST. This is a real, separate
%      prerequisite from ROS Toolbox itself and the error message if
%      it's missing does not say "install MSVC" -- it fails deep inside
%      message generation with a compiler-not-found error.
%   3. ROS_DOMAIN_ID must match whatever the bridge node's ROS 2
%      environment uses. If STK/the bridge run on Windows alongside
%      MATLAB (topology T1, the default), this is usually automatic; for
%      T2/T3 (bridge on Linux) set it explicitly as an environment
%      variable before starting MATLAB.

    p = inputParser;
    addParameter(p, 'WorkspacePath', '', @ischar);
    addParameter(p, 'Rebuild', false, @islogical);
    parse(p, varargin{:});

    thisFile = mfilename('fullpath');
    matlabDir = fileparts(thisFile);
    if isempty(p.Results.WorkspacePath)
        % stk_matlab/ sits at src/stk_matlab; the workspace root is two
        % levels up.
        wsRoot = fileparts(fileparts(matlabDir));
    else
        wsRoot = p.Results.WorkspacePath;
    end

    fprintf('stk_bridge MATLAB setup\n');
    fprintf('  workspace root: %s\n', wsRoot);

    addpath(matlabDir);
    fprintf('  added %s to path\n', matlabDir);

    msgPkgDir = fullfile(wsRoot, 'src', 'stk_msgs');
    if ~isfolder(msgPkgDir)
        error('setup_stk_ros:NoMsgPackage', ...
            ['stk_msgs source not found at %s.\nCheck WorkspacePath, ' ...
             'or pass it explicitly: setup_stk_ros(''WorkspacePath'', ...)'], ...
            msgPkgDir);
    end

    % ros2genmsg scans a *folder tree containing msg/srv/action
    % directories*, and (per MathWorks documentation) that folder must
    % NOT be named exactly like an existing ROS 2 package it will
    % generate a class for confusion with. Point it at the stk_msgs
    % package folder directly, matching the source-of-truth files
    % declared in stk_msgs/CMakeLists.txt -- see that file's note on
    % why .msg/.srv/.action stay the single source of truth for BOTH
    % the colcon build and this MATLAB build.
    genFolder = fullfile(matlabDir, 'generated_msgs');
    alreadyBuilt = isfolder(genFolder) && ...
        ~isempty(dir(fullfile(genFolder, 'matlab_msg_gen*')));

    if alreadyBuilt && ~p.Results.Rebuild
        fprintf(['  generated_msgs already exists; skipping ros2genmsg ' ...
                 '(pass ''Rebuild'', true to force).\n']);
    else
        fprintf('  running ros2genmsg on %s ...\n', msgPkgDir);
        fprintf(['  (this compiles C++ against your MATLAB-bundled ROS 2 ' ...
                 'client libraries; on Windows it requires MSVC -- see ' ...
                 'this function''s help text if it fails here)\n']);
        if ~isfolder(genFolder)
            mkdir(genFolder);
        end
        ros2genmsg(fileparts(msgPkgDir), 'FolderPath', genFolder); %#ok<*NASGU>
    end

    addpath(genpath(genFolder));
    fprintf('  added generated message path\n');

    fprintf('  checking ROS 2 network...\n');
    try
        topicList = ros2('topic', 'list'); %#ok<NASGU>
        fprintf('  ros2 network reachable.\n');
    catch causeErr
        warning('setup_stk_ros:NetworkCheckFailed', ...
            ['Could not query the ROS 2 network (%s). This is not fatal ' ...
             '-- the bridge node may simply not be running yet -- but if ' ...
             'it IS running, check ROS_DOMAIN_ID matches and, on a split ' ...
             'topology (T2/T3, see docs/00_architecture.md), that UDP ' ...
             'multicast is not blocked by a firewall.'], causeErr.message);
    end

    fprintf('\nSetup complete. Try:\n');
    fprintf('  client = stkros.BridgeClient();\n');
    fprintf('  state = client.telemetry.latest();\n');
end
