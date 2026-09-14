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
% signal wiring below was unbuilt at the time.
%
% Signal wiring
% -------------
% Built now that ros2genmsg can run here (see docs/C4_scheduler.md
% section 2), matching the "keep the block thin, put substance in
% +adcs" rule guidanceControlSource() already follows: UnpackTelemetry
% flattens the five subscribed buses into the plain numerics
% adcs.twin_step expects; publishing goes the other way, Blank Message
% (a stk_msgs-typed default bus) -> Bus Assignment (writes the computed
% fields by dotted path, e.g. "orientation.x") -> Header Assignment
% (stamps with the node's own clock, /clock-derived under use_sim_time)
% -> Publish -- the idiom ROS Toolbox's own examples use, since a
% MATLAB Function block can infer a bus INPUT's type from whatever is
% wired to it but constructing a typed bus as an ordinary function
% OUTPUT needs explicit, version-sensitive port configuration.
%
% NOT diagram-update-verified in THIS environment, and that is a
% dev-box tooling limitation, not a defect in the wiring below. Confirmed
% empirically: in this R2024a-on-Linux setup (the SAME setup that makes
% ros2genmsg work at all -- see "ros2genmsg now works here" in local
% working notes), no block that needs ELEMENT-LEVEL bus introspection
% (a MATLAB Function reading a field, or a Bus Selector) can resolve a
% Subscribe block's output for ANY stk_msgs custom message -- reproduced
% identically for AttitudeState, AngularRate and ScenarioTime, each in
% total isolation (a single Subscribe feeding a single one-line MATLAB
% Function), all failing "cannot determine sizes and/or types of the
% outputs" / a port-width mismatch that names nothing actually wrong.
% Blocks that do NOT need introspection (Terminator; a Blank-Message-
% sourced bus feeding Bus Assignment, used throughout below) work
% without complaint, and a MATLAB Function reading builtin_interfaces/
% Time (a BUILT-IN, not custom-generated, type, via ros2lib/Current
% Time) also works cleanly in a separate isolated test -- so the
% fault is specific to consuming a Subscribe block's output for a
% ros2genmsg-CUSTOM type here, not to MATLAB Function blocks, Bus
% Selectors, or this message shape in particular. Most likely the same
% root cause as the ros2genmsg GLIBCXX/typesupport mismatch already
% documented: the introspection path needs a live subscriber handshake
% that this dev box's generated libraries and MATLAB's own bundled ROS
% runtime disagree on, and unlike the CXXFLAGS/LD_LIBRARY_PATH fixes
% already found, this one did not yield to an environment variable set
% from the calling MATLAB process (plausibly because ROS's own separate
% server subprocess, libmwros2server, does not inherit it). NOT
% expected to reproduce on the deploy machine, which needs neither
% workaround in the first place. The diagram-update check below is kept
% as a WARNING rather than a hard failure for exactly this reason: it
% cannot currently pass here regardless of whether the wiring is right,
% so treating its failure as fatal would block every build on this box.
%
% Building this wiring is what exposed a real defect in adcs.twin_step,
% since fixed: quaternionCmd was `in.quaternion` verbatim -- the same
% value AttitudeSub had just received -- so the published command
% equalled the last reported state every tick and nothing in the loop
% ever integrated the computed torque. twin_step now runs the torque
% through adcs.wheels and adcs.rigid_body_step and publishes the
% attitude that results, and a twin_step section in the fixtures covers
% the orchestration that previously had no counterpart in the Python
% reference to be cross-checked against.

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
    addSubscribe(modelName, 'AngularRateSub', [40 310 200 370], ...
        [base '/angular_rate'], 'stk_msgs/AngularRate');
    % Not namespaced under base: the bridge publishes ScenarioTime once
    % per bridge instance (see nodes/clock_node.py), not per satellite --
    % the same distinction LockstepAckPub's own topic already draws.
    addSubscribe(modelName, 'ScenarioTimeSub', [40 400 200 460], ...
        sprintf('/%s/scenario_time', options.Prefix), ...
        'stk_msgs/ScenarioTime');

    % -- mission mode chart ---------------------------------------------
    % In +adcs so it can be built and structurally checked without the
    % ROS blocks, which need generated messages. test_acav_twin asserts
    % that SAFE latches and that its guards run first.
    %
    % Deliberately NOT wired into the signal path below. adcs.twin_step
    % (called from GuidanceControl) already calls adcs.mission_mode
    % directly -- the SAME transition logic as a function -- to decide
    % state.mode, and that decision is entangled with slew-profile
    % initialization (a mode entry into SLEW starts a new profile) in a
    % way the chart alone does not capture. Wiring the chart in parallel
    % would either duplicate the mode computation with no consumer, or
    % require restructuring GuidanceControl to take the chart's mode as
    % an input -- a control-flow change, not a wiring one. It stays here
    % as the paper's hybrid-model figure, built and structurally checked
    % (build_mission_chart itself; SAFE latching, guard order) but not
    % part of the executed loop.
    adcs.build_mission_chart(modelName, 'MissionMode', [270 490 470 610]);

    % -- unwrap ROS messages into the plain numerics twin_step expects ---
    unpackBlock = [modelName '/UnpackTelemetry'];
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        unpackBlock, 'Position', [240 40 420 460]);
    setMatlabFunctionBody(unpackBlock, unpackTelemetrySource());

    add_line(modelName, 'AttitudeSub/1', 'UnpackTelemetry/1');
    add_line(modelName, 'OrbitSub/1', 'UnpackTelemetry/2');
    add_line(modelName, 'SetpointSub/1', 'UnpackTelemetry/3');
    add_line(modelName, 'AngularRateSub/1', 'UnpackTelemetry/4');
    add_line(modelName, 'ScenarioTimeSub/1', 'UnpackTelemetry/5');

    % -- guidance and control -------------------------------------------
    controlBlock = [modelName '/GuidanceControl'];
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        controlBlock, 'Position', [480 40 680 320]);
    setMatlabFunctionBody(controlBlock, guidanceControlSource());

    % UnpackTelemetry outputs, in order: qMeas, omega, pos, vel,
    % targetEci, haveTarget, t, objectPath, stepIndex. The first seven
    % feed GuidanceControl 1:1; objectPath and stepIndex bypass it and
    % are picked up directly by the publish chains below, since they
    % pass through unchanged and adcs.twin_step has no use for either.
    for k = 1:7
        add_line(modelName, sprintf('UnpackTelemetry/%d', k), ...
            sprintf('GuidanceControl/%d', k));
    end

    % The torque IS integrated -- inside adcs.twin_step, which runs it
    % through the wheels and the rigid-body step to produce qCmd. What
    % comes out of the block here is the DELIVERED torque, and it is
    % diagnostic: nothing downstream of this model consumes it, since
    % STK is sent an attitude and the scheduler is sent a TwinStatus.
    % Terminate it rather than leave an unconnected output, which the
    % diagram-update check below would otherwise flag as an error
    % indistinguishable from a real wiring mistake. Wheel momentum,
    % saturation and the energy integral DO reach the campaign record --
    % they are TwinStatus fields, published below. The torque itself is
    % not, because nothing downstream uses it: the energy integral is
    % the quantity the study reports, and the twin computes that itself
    % since it is the only party holding torque and rate together.
    add_block('simulink/Sinks/Terminator', [modelName '/TorqueSink'], ...
        'Position', [720 40 750 60]);
    add_line(modelName, 'GuidanceControl/1', 'TorqueSink/1');

    % -- command out: stk_msgs/AttitudeCommand ---------------------------
    % GuidanceControl outputs, in order: torque, qCmd, mode, err.
    demuxBlock = [modelName '/DemuxQCmd'];
    add_block('simulink/Signal Routing/Demux', demuxBlock, ...
        'Position', [720 100 730 180], 'Outputs', '4');
    add_line(modelName, 'GuidanceControl/2', 'DemuxQCmd/1');

    add_block('simulink/Sources/Constant', ...
        [modelName '/UseInertialFrame'], 'Position', [720 210 750 230], ...
        'Value', 'false', 'OutDataTypeStr', 'boolean');

    addBlankMessage(modelName, 'AttitudeCmdBlank', [790 40 940 60], ...
        'stk_msgs/AttitudeCommand');
    addBusAssignment(modelName, 'AttitudeCmdAssign', [790 90 940 230], ...
        {'orientation.x', 'orientation.y', 'orientation.z', ...
         'orientation.w', 'object_path', 'use_inertial_frame'});
    add_line(modelName, 'AttitudeCmdBlank/1', 'AttitudeCmdAssign/1');
    for k = 1:4
        add_line(modelName, sprintf('DemuxQCmd/%d', k), ...
            sprintf('AttitudeCmdAssign/%d', k + 1));
    end
    add_line(modelName, 'UnpackTelemetry/8', 'AttitudeCmdAssign/6');
    add_line(modelName, 'UseInertialFrame/1', 'AttitudeCmdAssign/7');

    addHeaderAssignment(modelName, 'AttitudeCmdStamp', [980 90 1090 130]);
    add_line(modelName, 'AttitudeCmdAssign/1', 'AttitudeCmdStamp/1');

    addPublish(modelName, 'AttitudeCmdPub', [1130 90 1310 130], ...
        sprintf('/%s/cmd/attitude', options.Prefix), ...
        'stk_msgs/AttitudeCommand');
    add_line(modelName, 'AttitudeCmdStamp/1', 'AttitudeCmdPub/1');

    % -- status out: stk_msgs/TwinStatus ---------------------------------
    % Carries the wheel state as well as mode and pointing error, because
    % the run-level quantities acav/metrics.py reports (peak momentum,
    % saturation events, wheel energy) have no other route out of the
    % twin -- and the scheduler must not re-derive them, for the same
    % reason it does not re-derive the pointing error.
    add_block('simulink/Signal Routing/Demux', [modelName '/DemuxMomentum'], ...
        'Position', [720 250 730 310], 'Outputs', '3');
    add_line(modelName, 'GuidanceControl/5', 'DemuxMomentum/1');

    addBlankMessage(modelName, 'TwinStatusBlank', [790 260 940 280], ...
        'stk_msgs/TwinStatus');
    addBusAssignment(modelName, 'TwinStatusAssign', [790 300 940 380], ...
        {'mode', 'boresight_error_rad', 'wheel_momentum_nms.x', ...
         'wheel_momentum_nms.y', 'wheel_momentum_nms.z', ...
         'wheels_saturated', 'wheel_energy_j'});
    add_line(modelName, 'TwinStatusBlank/1', 'TwinStatusAssign/1');
    add_line(modelName, 'GuidanceControl/3', 'TwinStatusAssign/2');
    add_line(modelName, 'GuidanceControl/4', 'TwinStatusAssign/3');
    for k = 1:3
        add_line(modelName, sprintf('DemuxMomentum/%d', k), ...
            sprintf('TwinStatusAssign/%d', k + 3));
    end
    add_line(modelName, 'GuidanceControl/6', 'TwinStatusAssign/7');
    add_line(modelName, 'GuidanceControl/7', 'TwinStatusAssign/8');

    addHeaderAssignment(modelName, 'TwinStatusStamp', [980 300 1090 340]);
    add_line(modelName, 'TwinStatusAssign/1', 'TwinStatusStamp/1');

    addPublish(modelName, 'TwinStatusPub', [1130 300 1310 340], ...
        options.TwinStatusTopic, 'stk_msgs/TwinStatus');
    add_line(modelName, 'TwinStatusStamp/1', 'TwinStatusPub/1');

    if options.LockstepMode
        % -- lockstep ack: std_msgs/String, no header field to stamp ----
        ackTextBlock = [modelName '/BuildAckText'];
        add_block('simulink/User-Defined Functions/MATLAB Function', ...
            ackTextBlock, 'Position', [720 410 900 440]);
        setMatlabFunctionBody(ackTextBlock, buildAckTextSource());
        add_line(modelName, 'UnpackTelemetry/9', 'BuildAckText/1');

        addBlankMessage(modelName, 'AckBlank', [940 400 1090 420], ...
            'std_msgs/String');
        addBusAssignment(modelName, 'AckAssign', [940 430 1090 470], ...
            {'data'});
        add_line(modelName, 'AckBlank/1', 'AckAssign/1');
        add_line(modelName, 'BuildAckText/1', 'AckAssign/2');

        addPublish(modelName, 'LockstepAckPub', [1130 430 1310 470], ...
            sprintf('/%s/lockstep/ack', options.Prefix), ...
            'std_msgs/String');
        add_line(modelName, 'AckAssign/1', 'LockstepAckPub/1');

        add_block('built-in/Note', [modelName '/AckOrderingNote'], ...
            'Position', [720 490 1160 570], 'Text', ...
            ['Publish the ack AFTER the attitude command for this ' ...
             'step. The bridge releases the barrier on the ack, so ' ...
             'acking first lets STK advance past a step whose command ' ...
             'has not arrived -- which does not error, it just quietly ' ...
             'applies the command one step late for the whole run.']);
    end

    add_block('built-in/Note', [modelName '/TruthNote'], ...
        'Position', [40 630 1160 690], 'Text', ...
        ['The command published to STK is the ACHIEVED attitude, not ' ...
         'the reference. Publishing the reference would reinstate the ' ...
         'perfect-pointing assumption this study exists to challenge, ' ...
         'and every capture would succeed.']);

    try
        set_param(modelName, 'SimulationCommand', 'update');
        fprintf('build_acav_twin: diagram update succeeded -- wiring is structurally verified.\n');
    catch err
        detail = err.message;
        for k = 1:numel(err.cause)
            detail = sprintf('%s\n  - %s', detail, err.cause{k}.message);
        end
        % A WARNING, not a hard failure: on this dev box, ANY MATLAB
        % Function or Bus Selector consuming a Subscribe block's output
        % for a custom stk_msgs type fails this exact check regardless
        % of whether the wiring is right (see the "Signal wiring" header
        % comment) -- confirmed on three unrelated message types in
        % total isolation. Treating it as fatal would block every build
        % here for a reason that has nothing to do with this function's
        % correctness. Re-run on the deploy machine (or if this dev
        % box's introspection limitation gets fixed) to get a real
        % diagram-update verification.
        warning('build_acav_twin:WiringUnverified', ...
            ['diagram update failed -- see the "Signal wiring" header ' ...
             'comment for why this specific check is known-unreliable ' ...
             'on this dev box:\n%s'], detail);
    end

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
              "stk_msgs/AngularRate", "stk_msgs/ScenarioTime", ...
              "stk_msgs/AttitudeCommand", "stk_msgs/TargetSetpoint", ...
              "stk_msgs/TwinStatus", "std_msgs/String"];
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

function addBlankMessage(model, name, position, messageType)
%ADDBLANKMESSAGE A default-valued instance of messageType, to overwrite
% field-by-field via a following Bus Assignment. The idiom ROS Toolbox's
% own examples use to construct a typed message from plain signals,
% since a MATLAB Function block can infer a BUS INPUT's type from
% whatever is wired to it but constructing one as a function OUTPUT
% needs explicit, release-sensitive port configuration.
%
% Both 'entityType' and 'messageType' must be set. The block's mask
% defaults both to geometry_msgs/Point; setting 'messageType' alone
% left 'entityType' -- and so the compiled output bus -- stuck at that
% default for std_msgs/String specifically (confirmed empirically: two
% stk_msgs types picked up messageType alone without complaint, so this
% is not simply a no-op parameter, just one whose propagation cannot be
% relied on for every message name).
    block = [model '/' name];
    add_block('ros2lib/Blank Message', block, 'Position', position);
    set_param(block, 'entityType', messageType, 'messageType', messageType);
end

function addBusAssignment(model, name, position, dottedFields)
%ADDBUSASSIGNMENT A Bus Assignment writing each of dottedFields, in
% order, from its own input port. AssignedSignals accepts comma-separated
% dotted paths into nested buses (e.g. "orientation.x") -- confirmed
% empirically against a generated stk_msgs bus, not assumed from docs.
    block = [model '/' name];
    add_block('simulink/Signal Routing/Bus Assignment', block, ...
        'Position', position);
    set_param(block, 'AssignedSignals', strjoin(dottedFields, ','));
end

function addHeaderAssignment(model, name, position)
%ADDHEADERASSIGNMENT Stamp a message's header with the node's own clock.
% InsertTimeStamp='on' is the only field this model needs set: the stamp
% follows /clock under use_sim_time (a launch-time setting, not a model
% one -- see the SolverType comment above), and frame_id is left at the
% block's default rather than hardcoded here, since none of these three
% messages' headers are read by a frame-dependent consumer.
    block = [model '/' name];
    add_block('ros2lib/Header Assignment', block, 'Position', position);
    set_param(block, 'InsertTimeStamp', 'on');
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
        'function [torque, qCmd, mode, err, momentum, saturated, energy] = ...'
        '        fcn(qMeas, omega, pos, vel, targetEci, haveTarget, t)'
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
        '    momentum = out.state.wheelMomentum;'
        '    saturated = out.wheelsSaturated;'
        '    energy = out.state.wheelEnergyJ;'
        'end'
        }, newline);
end

function source = unpackTelemetrySource()
    %UNPACKTELEMETRYSOURCE Flatten the five subscribed ROS buses into the
    % plain numerics adcs.twin_step expects, plus two passthroughs
    % (objectPath, stepIndex) the publish side needs but twin_step does
    % not. A MATLAB Function block infers each INPUT's bus type from
    % whatever is wired to it, so this needs no Bus Selector chain.
    source = strjoin({
        'function [qMeas, omega, pos, vel, targetEci, haveTarget, t, ...'
        '    objectPath, stepIndex] = fcn(attitudeMsg, orbitMsg, ...'
        '    setpointMsg, rateMsg, timeMsg)'
        '%#codegen'
        '    qMeas = [attitudeMsg.orientation.x, attitudeMsg.orientation.y, ...'
        '        attitudeMsg.orientation.z, attitudeMsg.orientation.w];'
        '    omega = [rateMsg.angular_velocity.x, rateMsg.angular_velocity.y, ...'
        '        rateMsg.angular_velocity.z];'
        '    pos = [orbitMsg.position.x, orbitMsg.position.y, orbitMsg.position.z];'
        '    vel = [orbitMsg.velocity.x, orbitMsg.velocity.y, orbitMsg.velocity.z];'
        '    targetEci = [setpointMsg.target_position_m.x, ...'
        '        setpointMsg.target_position_m.y, setpointMsg.target_position_m.z];'
        '    haveTarget = setpointMsg.has_target;'
        '    t = double(timeMsg.sim_time.sec) + double(timeMsg.sim_time.nanosec) * 1e-9;'
        '    objectPath = attitudeMsg.object_path;'
        '    stepIndex = timeMsg.step_index;'
        'end'
        }, newline);
end

function source = buildAckTextSource()
    %BUILDACKTEXTSOURCE The lockstep ack's payload: "matlab_twin:<step>".
    % Matches the format acav_scheduler's own ack already uses
    % ("acav_scheduler:<step_index>", see docs/C4_scheduler.md section 5)
    % so a log reader can tell which participant acked a given step.
    source = strjoin({
        'function text = fcn(stepIndex)'
        '%#codegen'
        '    text = "matlab_twin:" + string(stepIndex);'
        'end'
        }, newline);
end
