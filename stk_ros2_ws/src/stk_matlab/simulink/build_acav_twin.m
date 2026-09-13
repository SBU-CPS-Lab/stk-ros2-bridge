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
%   ROS2 Subscribe  /acav/setpoint         -> the scheduler's target (C4)
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
% What this does NOT do
% ---------------------
% It does not verify that the generated model runs. Simulink model
% construction is version-sensitive, and the honest position is the same
% one taken for STK: run it, and if a block name or parameter has moved
% in your release, the error will say which. See docs/C3_adcs_twin.md.

    arguments
        options.ModelName (1,1) string = "acav_twin"
        options.Namespace (1,1) string = "sss_1"
        options.Prefix (1,1) string = "stk"
        options.LockstepMode (1,1) logical = true
        options.SampleTime (1,1) double = 0.1
    end

    modelName = char(options.ModelName);
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
    new_system(modelName);
    load_system('ros2lib');

    set_param(modelName, 'SolverType', 'Fixed-step', ...
        'FixedStep', num2str(options.SampleTime), ...
        'StopTime', 'inf');
    try
        set_param(modelName, 'ROSTimeSource', 'ROSClock');
    catch
        warning('build_acav_twin:TimeSource', ...
            ['could not set ROSTimeSource on this release. The twin ' ...
             'MUST follow /clock in lockstep -- set it by hand under ' ...
             'Model Configuration Parameters > ROS 2, or the barrier ' ...
             'will run against wall-clock time and E3 will measure ' ...
             'the mismatch rather than determinism.']);
    end

    base = sprintf('/%s/%s', options.Prefix, options.Namespace);

    % -- telemetry in ---------------------------------------------------
    addSubscribe(modelName, 'AttitudeSub', [40 40 200 100], ...
        [base '/attitude'], 'stk_msgs/AttitudeState');
    addSubscribe(modelName, 'OrbitSub', [40 130 200 190], ...
        [base '/orbit'], 'stk_msgs/OrbitState');
    addSubscribe(modelName, 'SetpointSub', [40 220 200 280], ...
        '/acav/setpoint', 'stk_msgs/AttitudeCommand');

    % -- mission mode chart ---------------------------------------------
    buildMissionModeChart(modelName, [270 220 470 340]);

    % -- guidance and control -------------------------------------------
    controlBlock = [modelName '/GuidanceControl'];
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        controlBlock, 'Position', [270 40 470 190]);
    setMatlabFunctionBody(controlBlock, guidanceControlSource());

    % -- command out ------------------------------------------------------
    addPublish(modelName, 'AttitudeCmdPub', [540 40 720 100], ...
        sprintf('/%s/cmd/attitude', options.Prefix), ...
        'stk_msgs/AttitudeCommand');

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
    fprintf(['build_acav_twin: wrote %s.slx\n' ...
             '  Run test_acav_twin first -- it checks the algorithms ' ...
             'this model calls against the Python reference.\n'], ...
        modelName);
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

function buildMissionModeChart(model, position)
    %BUILDMISSIONMODECHART NADIR -> SLEW -> SETTLE -> IMAGE, plus SAFE.
    %
    % Guards mirror adcs/mission_mode.m exactly. Two properties there are
    % load-bearing and are reproduced here:
    %
    %   * the SAFE transitions have priority 1, so a wheel that saturates
    %     during IMAGE trips SAFE rather than being masked by an
    %     already-satisfied hold;
    %   * SAFE has no outgoing transition. An ADCS that re-armed itself
    %     the instant the trip cleared would oscillate in and out and
    %     make E5's deterministic replay meaningless.
    try
        chartPath = [model '/MissionMode'];
        add_block('sflib/Chart', chartPath, 'Position', position);
        root = sfroot();
        chart = root.find('-isa', 'Stateflow.Chart', '-and', ...
            'Path', chartPath);

        states = struct();
        names = {'NADIR', 'SLEW', 'SETTLE', 'IMAGE', 'SAFE'};
        for k = 1:numel(names)
            s = Stateflow.State(chart);
            s.Name = names{k};
            s.Position = [40 + 130 * mod(k - 1, 3), ...
                          40 + 110 * floor((k - 1) / 3), 100, 60];
            states.(names{k}) = s;
        end

        entry = Stateflow.Transition(chart);
        entry.Destination = states.NADIR;
        entry.DestinationOClock = 0;
        entry.SourceEndpoint = states.NADIR.Position(1:2) - [0 40];

        addTransition(chart, states.NADIR, states.SLEW, ...
            '[slewRequested]');
        addTransition(chart, states.IMAGE, states.SLEW, ...
            '[slewRequested]');
        addTransition(chart, states.SLEW, states.SETTLE, ...
            '[profileDone]');
        addTransition(chart, states.SETTLE, states.IMAGE, ...
            '[boresightError <= settleEntry]');

        % Priority 1: evaluated before anything else leaving the state.
        for k = 1:4
            t = addTransition(chart, states.(names{k}), states.SAFE, ...
                '[momentumFraction >= 1 || bodyRate > maxBodyRate]');
            t.ExecutionOrder = 1;
        end
    catch err
        warning('build_acav_twin:Stateflow', ...
            ['could not build the Stateflow chart (%s). The model is ' ...
             'still usable: adcs.twin_step calls adcs.mission_mode, ' ...
             'which is the same logic. Add the chart by hand if the ' ...
             'paper needs the figure.'], err.message);
    end
end

function t = addTransition(chart, from, to, label)
    t = Stateflow.Transition(chart);
    t.Source = from;
    t.Destination = to;
    t.LabelString = label;
end
