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

    % R2023b expects the parent directory containing the ROS packages.
    % It writes matlab_msg_gen/<architecture> beneath that directory;
    % there is no 'FolderPath' name-value option for an output folder.
    msgRoot = fileparts(msgPkgDir);
    genFolder = fullfile(msgRoot, 'matlab_msg_gen', computer('arch'));

    % ros2genmsg writes its output under src/, and part of that output is
    % matlab_msg_gen/<arch>/src/stk_msgs -- a copy of the very package it
    % was pointed at. colcon skips build/ and install/ by name but not
    % that one, so it then discovers stk_msgs twice and the workspace
    % build fails on a duplicate package name. A COLCON_IGNORE marker at
    % the top of the generated tree is the standard remedy; verified
    % against colcon on Humble.
    msgGenRoot = fullfile(msgRoot, 'matlab_msg_gen');
    if ~isfolder(msgGenRoot)
        mkdir(msgGenRoot);
    end
    markerFile = fullfile(msgGenRoot, 'COLCON_IGNORE');
    if ~isfile(markerFile)
        fclose(fopen(markerFile, 'w'));
    end
    installFolder = fullfile(genFolder, 'install');
    % VersionInfo.mat is written after a successful build/registration.
    versionInfo = fullfile(genFolder, 'src', 'VersionInfo.mat');
    alreadyBuilt = isfolder(installFolder) && isfile(versionInfo);

    % Existence is not enough. Editing a .msg and re-running this used
    % to print "Setup complete" while leaving every generated class on
    % the OLD definition -- so a Simulink model wired to a new field
    % failed with "unable to find selected signal", naming the model
    % rather than the stale package. Compare timestamps instead: any
    % .msg newer than the build marker means the generated code no
    % longer describes the messages in the tree.
    staleMsgs = {};
    if alreadyBuilt
        built = dir(versionInfo);
        definitions = dir(fullfile(msgRoot, '**', '*.msg'));
        for k = 1:numel(definitions)
            if definitions(k).datenum > built.datenum
                staleMsgs{end+1} = definitions(k).name; %#ok<AGROW>
            end
        end
    end

    if alreadyBuilt && isempty(staleMsgs) && ~p.Results.Rebuild
        fprintf(['  generated messages already built and up to date; ' ...
                 'skipping ros2genmsg (pass ''Rebuild'', true to ' ...
                 'force).\n']);
    else
        if ~isempty(staleMsgs)
            fprintf(['  %d message definition(s) changed since the last ' ...
                     'build (%s); regenerating.\n'], numel(staleMsgs), ...
                    strjoin(staleMsgs, ', '));
        end
        fprintf('  running ros2genmsg on %s ...\n', msgRoot);
        fprintf(['  (this compiles C++ against your MATLAB-bundled ROS 2 ' ...
                 'client libraries; on Windows it requires MSVC -- see ' ...
                 'this function''s help text if it fails here)\n']);
        ros2genmsg(msgRoot);
    end

    addpath(genpath(installFolder));
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
