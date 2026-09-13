function modelName = build_acav_twin(options)
%BUILD_ACAV_TWIN Construct the ACAV ADCS twin as a Simulink model.
%
%   build_acav_twin()
%   build_acav_twin('ModelName', "acav_twin", 'LockstepMode', true)
%
% The model is kept as CODE, not as a checked-in .slx. Simulink's binary
% format does not diff or merge, so a reviewer cannot see what changed
% in a control law and a git conflict in one is unresolvable. Run this
% to generate the model; treat the .slx as a build artifact and edit
% this function, not the model.
%
% Structure
% ---------
%   ROS2 Subscribe  /stk/<ns>/attitude     -> AttitudeState
%   ROS2 Subscribe  /stk/<ns>/orbit        -> OrbitState
%   ROS2 Subscribe  /acav/setpoint         -> stk_msgs/TargetSetpoint,
%                                              the scheduler's guidance
%                                              target (C4)
%           |
%   Stateflow chart "MissionMode"  NADIR -> SLEW -> SETTLE -> IMAGE,
%           |                      SAFE reachable from anywhere
%   MATLAB Function "GuidanceControl"  -> adcs.twin_step
%           |
%   Reaction wheels + rigid-body dynamics (adcs.wheels,
%   adcs.rigid_body_step) -- present so the twin can run open-loop for
%   development; in the closed loop STK is the truth and these are
%   bypassed by the ForcedTruth switch.
%           |
%   ROS2 Publish   /stk/cmd/attitude       -> AttitudeCommand
%   ROS2 Publish   /acav/twin_status       -> stk_msgs/TwinStatus (mode +
%                                              boresight error, for the
%                                              scheduler's capture scoring
%                                              and SAFE detection)
%   ROS2 Publish   /stk/lockstep/ack       -> the barrier ack
%
% Why the chart AND adcs.mission_mode
% -----------------------------------
% The chart is what the paper shows and what makes the model genuinely
% hybrid. adcs.mission_mode is the same transition logic as a function,
% which is what test_acav_twin.m can actually exercise against the
% Python reference. They must agree; the chart's guards below are
% written to mirror that file line for line, and any change belongs in
% both.
%
% Why /acav/setpoint carries a POSITION, not an attitude
% -------------------------------------------------------
% stk_msgs/TargetSetpoint carries the target's inertial position and a
% has_target flag -- not a quaternion. The scheduler's whole
% contribution to E2 is WHEN it switches targets and WHETHER it
% attempts one at all (acav/scheduling.py); the guidance law that turns
% "point here" into a commanded attitude, accounting for this vehicle's
% own boresight axis and a velocity-referenced roll, stays entirely in
% adcs.twin_step. An earlier version of this file wired the SAME topic
% to stk_msgs/AttitudeCommand, which cannot carry a target position at
% all -- caught before anything downstream depended on it, since the
% signal wiring below was (and remains) unbuilt.
%
% What this does NOT do
% ---------------------
% It does not wire the blocks' signals together, and it does not verify
% that the generated model runs. Both need a MATLAB-side ros2genmsg
% pass against stk_msgs (see setup_stk_ros) to even know the generated
% bus object's field names, which cannot be done in this development
% environment (see docs/C3_adcs_twin.md). Wiring blind, without being
% able to check a single connection, would risk shipping something that
% LOOKS complete and silently is not -- worse than leaving it explicit.
% Connect AttitudeSub/OrbitSub/SetpointSub through Bus Selector blocks
% into GuidanceControl's (qMeas, omega, pos, vel, targetEci, haveTarget,
% t) inputs, and GuidanceControl's (mode, err) outputs into a
% TwinStatusPub via a Bus Creator, on the deploy machine, once messages
% are generated there.

    arguments
        options.ModelName (1,1) string = "acav_twin"
        options.Namespace (1,1) string = "sss_1"
        options.Prefix (1,1) string = "stk"
        options.LockstepMode (1,1) logical = true
        options.SampleTime (1,1) double = 0.1
        options.SetpointTopic (1,1) string = "/acav/setpoint"
        options.TwinStatusTopic (1,1) string = "/acav/twin_status"
    end

    modelName = char(options.ModelName);
    requireMessages();

    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
    new_system(modelName);
    cleanup = onCleanup(@() discardIfUnsaved(modelName));
    load_system('ros2lib');

    set_param(modelName, 'SolverType', 'Fixed-step', ...
        'FixedStep', num2str(options.SampleTime), ...
        'StopTime', 'inf');

    % There is NO model-level ROS time-source parameter. Checked on
    % R2023a: the model has none, and the ROS 2 Current Time block
    % exposes only OutputFormat and SampleTime. Time follows /clock
    % because the generated node runs with use_sim_time true, which is
    % a launch-time setting, not a model one. An earlier version of this
    % function set an invented 'ROSTimeSource' parameter and warned when
    % it failed, which read as a release incompatibility rather than as
    % a parameter that never existed.

    base = sprintf('/%s/%s', options.Prefix, options.Namespace);

    % -- telemetry in ---------------------------------------------------
    addSubscribe(modelName, 'AttitudeSub', [40 40 200 100], ...
        [base '/attitude'], 'stk_msgs/AttitudeState');
    addSubscribe(modelName, 'OrbitSub', [40 130 200 190], ...
        [base '/orbit'], 'stk_msgs/OrbitState');
    addSubscribe(modelName, 'SetpointSub', [40 220 200 280], ...
        options.SetpointTopic, 'stk_msgs/TargetSetpoint');

    % -- mission mode chart ---------------------------------------------
    % In +adcs so it can be built and structurally checked without the
    % ROS blocks, which need generated messages. test_acav_twin asserts
    % that SAFE latches and that its guards run first.
    adcs.build_mission_chart(modelName, 'MissionMode', [270 220 470 340]);

    % -- guidance and control -------------------------------------------
    controlBlock = [modelName '/GuidanceControl'];
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        controlBlock, 'Position', [270 40 470 190]);
    setMatlabFunctionBody(controlBlock, guidanceControlSource());

    % -- command + status out ---------------------------------------------
    addPublish(modelName, 'AttitudeCmdPub', [540 40 720 100], ...
        sprintf('/%s/cmd/attitude', options.Prefix), ...
        'stk_msgs/AttitudeCommand');
    addPublish(modelName, 'TwinStatusPub', [540 130 720 190], ...
        options.TwinStatusTopic, 'stk_msgs/TwinStatus');

    if options.LockstepMode
        addPublish(modelName, 'LockstepAckPub', [540 220 720 280], ...
            sprintf('/%s/lockstep/ack', options.Prefix), ...
            'std_msgs/String');
        add_block('built-in/Note', [modelName '/AckOrderingNote'], ...
            'Position', [540 320 980 400], 'Text', ...
            ['Publish the ack AFTER the attitude command for this ' ...
             'step. The bridge releases the barrier on the ack, so ' ...
             'acking first lets STK advance past a step whose command ' ...
             'has not arrived -- which does not error, it just quietly ' ...
             'applies the command one step late for the whole run.']);
    end

    add_block('built-in/Note', [modelName '/TruthNote'], ...
        'Position', [40 420 980 500], 'Text', ...
        ['The command published to STK is the ACHIEVED attitude, not ' ...
         'the reference. Publishing the reference would reinstate the ' ...
         'perfect-pointing assumption this study exists to challenge, ' ...
         'and every capture would succeed.']);

    save_system(modelName, fullfile(fileparts(mfilename('fullpath')), ...
        [modelName '.slx']));
    clear cleanup
    fprintf(['build_acav_twin: wrote %s.slx\n' ...
             '  Run test_acav_twin first -- it checks the algorithms ' ...
             'this model calls against the Python reference.\n'], ...
        modelName);
end

% -----------------------------------------------------------------------

function requireMessages()
%REQUIREMESSAGES Fail before building anything if stk_msgs is missing.
%
% Without this the build dies partway through adding blocks, leaving a
% half-constructed model open, and the error names a mask callback
% rather than the actual problem. Observed exactly that: "Error
% evaluating 'MaskParameterCallback' ... Unrecognized message type
% stk_msgs/AttitudeState", which does not tell a reader to run
% setup_stk_ros.
    needed = ["stk_msgs/AttitudeState", "stk_msgs/OrbitState", ...
              "stk_msgs/AttitudeCommand", "stk_msgs/TargetSetpoint", ...
              "stk_msgs/TwinStatus"];
    try
        available = string(ros2("msg", "list"));
    catch err
        error('build_acav_twin:NoMessageList', ...
            ['could not list ROS 2 message types (%s).\nRun ' ...
             'setup_stk_ros first -- it generates stk_msgs for ' ...
             'MATLAB.'], err.message);
    end
    missing = needed(~ismember(needed, available));
    if ~isempty(missing)
        error('build_acav_twin:MissingMessages', ...
            ['%s is not registered with MATLAB.\n' ...
             'Run setup_stk_ros first; it calls ros2genmsg on ' ...
             'src/stk_msgs.\nOn Windows that needs MSVC Build Tools ' ...
             '("Desktop development with C++"), and the failure ' ...
             'message does not say so.'], strjoin(missing, ', '));
    end
end

function discardIfUnsaved(modelName)
%DISCARDIFUNSAVED Do not leave a half-built model open after an error.
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
end

% -----------------------------------------------------------------------

function addSubscribe(model, name, position, topic, messageType)
    block = [model '/' name];
    add_block('ros2lib/Subscribe', block, 'Position', position);
    set_param(block, 'topic', topic, 'messageType', messageType);
end

function addPublish(model, name, position, topic, messageType)
    block = [model '/' name];
    add_block('ros2lib/Publish', block, 'Position', position);
    set_param(block, 'topic', topic, 'messageType', messageType);
end

function setMatlabFunctionBody(blockPath, source)
    %SETMATLABFUNCTIONBODY Replace a MATLAB Function block's code.
    root = sfroot();
    block = root.find('-isa', 'Stateflow.EMChart', '-and', ...
        'Path', blockPath);
    if isempty(block)
        warning('build_acav_twin:NoChart', ...
            ['could not reach the MATLAB Function block''s source on ' ...
             'this release; paste the body of guidanceControlSource() ' ...
             'into %s by hand.'], blockPath);
        return
    end
    block.Script = source;
end

function source = guidanceControlSource()
    %GUIDANCECONTROLSOURCE The block body: a thin call into +adcs.
    %
    % Thin on purpose. Everything of substance lives in adcs.twin_step,
    % where test_acav_twin.m can reach it; a block full of algebra is a
    % block nothing can test.
    source = strjoin({
        'function [torque, qCmd, mode, err] = fcn(qMeas, omega, pos, vel, targetEci, haveTarget, t)'
        '%#codegen'
        '    persistent cfg state'
        '    if isempty(cfg)'
        '        cfg = adcs.config();'
        '        state = adcs.initial_state();'
        '    end'
        '    in = struct(''quaternion'', qMeas, ''omega'', omega, ...'
        '                ''position'', pos, ''velocity'', vel, ...'
        '                ''targetEci'', targetEci, ...'
        '                ''haveTarget'', haveTarget, ''t'', t);'
        '    out = adcs.twin_step(in, cfg, state);'
        '    state = out.state;'
        '    torque = out.torque;'
        '    qCmd = out.quaternionCmd;'
        '    mode = out.state.mode;'
        '    err = out.boresightErrorRad;'
        'end'
        }, newline);
end
