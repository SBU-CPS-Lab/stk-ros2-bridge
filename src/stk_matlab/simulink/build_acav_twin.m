function modelName = build_acav_twin(options)
%BUILD_ACAV_TWIN Construct the ADCS twin used by the ACAV case study.
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
%   ROS2 Subscribe  /stk/<ns>/angular_rate -> AngularRate
%   ROS2 Subscribe  /stk/scenario_time     -> ScenarioTime
%   ROS2 Subscribe  /acav/setpoint         -> stk_msgs/TargetSetpoint,
%                                              the scheduler's target
%           |
%   MATLAB Function "UnpackTelemetry"  buses -> plain numerics
%           |
%   MATLAB Function "GuidanceControl"  -> adcs.simulink_step
%       -> adcs.twin_step: mode logic (adcs.mission_mode), guidance,
%          control, reaction wheels and rigid-body dynamics. The twin
%          owns the integrated attitude and body rate; STK provides
%          orbit and access and reads the attitude back.
%           |
%   ROS2 Publish   /stk/cmd/attitude       -> AttitudeCommand
%   ROS2 Publish   /acav/twin_status       -> stk_msgs/TwinStatus (mode,
%                                              boresight error, wheels)
%   ROS2 Publish   /acav/twin_identity     -> run config and code identity
%   ROS2 Publish   /stk/lockstep/ack       -> the barrier ack
%
% The Stateflow chart "MissionMode" (adcs.build_mission_chart) is built
% into the model as the specification of the mode logic and checked
% structurally, but it is not wired into the signal path: the executed
% mode logic is adcs.mission_mode, called from adcs.twin_step.
%
% Why /acav/setpoint carries a position, not an attitude
% -------------------------------------------------------
% stk_msgs/TargetSetpoint carries the target's inertial position and a
% has_target flag. A scheduler decides when to switch targets and
% whether to attempt one at all (acav/scheduling.py); the guidance law
% that turns "point here" into a commanded attitude, including this
% vehicle's boresight axis and a velocity-referenced roll, stays in
% adcs.twin_step.
%
% Signal wiring
% -------------
% Blocks stay thin; the substance is in +adcs. UnpackTelemetry flattens
% the subscribed buses into the numerics adcs.twin_step expects.
% Publishing goes the other way: Blank Message (a typed default bus) ->
% Bus Assignment (fields by dotted path, e.g. "orientation.x") ->
% Publish, the idiom ROS Toolbox's examples use, because constructing a
% typed bus as a MATLAB Function output needs version-sensitive port
% configuration. Header stamps come from addStamp (see there).
%
% Subscribe output 1 is the boolean IsNew flag; output 2 is the Msg bus.
% MATLAB Function inputs must consume Msg to access message fields.
% Diagram update below is mandatory before saving a usable model.
% GuidanceControl calls MATLAB-hosted adcs.simulink_step in Normal mode:
% YAML/JSON loading and dynamic slew-profile state are not code-generated.
% This model requires MATLAB during simulation; it is not a standalone
% target.

    arguments
        options.ModelName (1,1) string = "acav_twin"
        options.Namespace (1,1) string = "sss_1"
        options.Prefix (1,1) string = "stk"
        options.LockstepMode (1,1) logical = true
        options.SampleTime (1,1) double = 0.1
        % How often the model steps, per wall second (Simulink pacing).
        % Unpaced, Simulink steps as fast as it can and MATLAB uses about
        % one core. Every step polls the subscribers, so the period bounds
        % the latency this adds: on average half a period per lockstep step
        % and at most one period after each free-run sample. Keep it well
        % below 200 Hz, where Simulink was measured to busy-wait; at 100 Hz
        % the Windows sleep granularity limits it to about 66 steps/s. The
        % value is part of the twin identity, and the default part of the
        % code fingerprint.
        options.PollingRateHz (1,1) double {mustBePositive, mustBeFinite} = 100
        options.SetpointTopic (1,1) string = "/acav/setpoint"
        options.TwinStatusTopic (1,1) string = "/acav/twin_status"
        % Which run config and code the running twin uses; checked by the
        % run orchestrator (acav/twin_identity.py) before the scheduler
        % starts.
        options.TwinIdentityTopic (1,1) string = "/acav/twin_identity"
        %#ok<*STRNU>
        % Per-run gain and fault overrides, baked into the
        % GuidanceControl block so the generated model carries them.
        % Empty is the nominal vehicle. A campaign writes one of these per
        % run next to the run's output; see adcs.config's help for the
        % format.
        options.RunConfigPath (1,1) string = ""
    end

    modelName = char(options.ModelName);
    matlabDir = fileparts(fileparts(mfilename('fullpath')));
    addpath(matlabDir);
    rehash;
    if strlength(options.RunConfigPath) > 0
        candidate = char(options.RunConfigPath);
        if ~isfile(candidate)
            candidate = fullfile(fileparts(fileparts(matlabDir)), candidate);
        end
        [found, attributes] = fileattrib(candidate);
        if ~found
            error('build_acav_twin:MissingRunConfig', ...
                'Run configuration does not exist: %s', options.RunConfigPath);
        end
        options.RunConfigPath = string(attributes.Name);
    end
    requireMessages();

    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
    new_system(modelName);
    % Opening a saved model must not depend on the caller's current folder
    % or on an old MATLAB package cache from before the adapter was added.
    bootstrap = [ ...
        'acavModelFile = get_param(bdroot,''FileName''); ' ...
        'if ~isempty(acavModelFile), ' ...
        'addpath(fileparts(fileparts(acavModelFile))); end; rehash;'];
    set_param(modelName, 'PostLoadFcn', bootstrap, 'InitFcn', bootstrap);
    cleanup = onCleanup(@() discardIfUnsaved(modelName));
    load_system('ros2lib');

    set_param(modelName, 'SolverType', 'Fixed-step', ...
        'SimulationMode', 'normal', ...
        'FixedStep', num2str(options.SampleTime), ...
        'StopTime', 'inf');
    % Pacing: simulated seconds per wall second, so the model takes
    % PollingRateHz steps per wall second instead of spinning. Simulation
    % time is not used for anything else (every stamp comes from the
    % bridge's ScenarioTime), and lockstep steps are still ordered by
    % the barrier, so this changes timing only.
    set_param(modelName, 'EnablePacing', 'on', ...
        'PacingRate', options.PollingRateHz * options.SampleTime);

    % The model never reads a ROS clock. Every outgoing stamp is derived
    % from the ScenarioTime message of the step being processed (see
    % addStamp): the Simulink ROS node is not run with use_sim_time, so
    % anything stamped from the node's own clock is wall-clock time.
    %
    % The command stamp is offset by the twin's own integration step,
    % read from the same config twin_step uses at run time.
    stepNs = round(adcs.config([], char(options.RunConfigPath)).stepS * 1e9);

    % The code this model is built from, written into GuidanceControl. At
    % simulation start adcs.simulink_step recomputes it from disk; the
    % two differ when the model is stale relative to the code.
    buildFingerprint = adcs.code_fingerprint();

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
    % Not wired into the signal path below. adcs.twin_step (called from
    % GuidanceControl) calls adcs.mission_mode, the same transition logic
    % as a function, and the mode decision is entangled with slew-profile
    % initialization (entering SLEW starts a new profile) in a way the
    % chart alone does not capture. Wiring the chart in would mean
    % restructuring GuidanceControl to take the chart's mode as an input.
    % The chart is therefore the built and checked specification (SAFE
    % latching, guard order), not part of the executed loop.
    adcs.build_mission_chart(modelName, 'MissionMode', [270 490 470 610]);

    % -- unwrap ROS messages into the plain numerics twin_step expects ---
    unpackBlock = [modelName '/UnpackTelemetry'];
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        unpackBlock, 'Position', [240 40 420 460]);
    setMatlabFunctionBody(unpackBlock, unpackTelemetrySource());

    add_line(modelName, 'AttitudeSub/2', 'UnpackTelemetry/1');
    add_line(modelName, 'OrbitSub/2', 'UnpackTelemetry/2');
    add_line(modelName, 'SetpointSub/2', 'UnpackTelemetry/3');
    add_line(modelName, 'AngularRateSub/2', 'UnpackTelemetry/4');
    add_line(modelName, 'ScenarioTimeSub/2', 'UnpackTelemetry/5');

    % -- guidance and control -------------------------------------------
    controlBlock = [modelName '/GuidanceControl'];
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        controlBlock, 'Position', [480 40 680 320]);
    setMatlabFunctionBody(controlBlock, ...
        guidanceControlSource(options.RunConfigPath, buildFingerprint, ...
        options.PollingRateHz));

    % UnpackTelemetry outputs, in order: qMeas, omega, pos, vel,
    % targetEci, haveTarget, t, objectPath, stepIndex, pathLength,
    % pathReceived, stampSec, stampNsec, targetId. The first seven feed
    % GuidanceControl 1:1; the rest bypass it and are picked up directly
    % by the publish chains below, since adcs.twin_step has no use for
    % them. Target identity also feeds GuidanceControl input 8.
    for k = 1:7
        add_line(modelName, sprintf('UnpackTelemetry/%d', k), ...
            sprintf('GuidanceControl/%d', k));
    end

    add_line(modelName, 'UnpackTelemetry/14', 'GuidanceControl/8');

    % The torque IS integrated -- inside adcs.twin_step, which runs it
    % through the wheels and the rigid-body step to produce qCmd. What
    % comes out of the block here is the DELIVERED torque, and it is
    % diagnostic: nothing downstream of this model consumes it, since
    % STK is sent an attitude and the scheduler is sent a TwinStatus.
    % It is terminated rather than left unconnected, which the
    % diagram-update check below would flag. Wheel momentum, saturation
    % and the energy integral reach consumers as TwinStatus fields.
    add_block('simulink/Sinks/Terminator', [modelName '/TorqueSink'], ...
        'Position', [720 40 750 60]);
    add_line(modelName, 'GuidanceControl/1', 'TorqueSink/1');

    % -- command out: stk_msgs/AttitudeCommand ---------------------------
    % GuidanceControl outputs, in order: torque, qCmd, mode, err.
    demuxBlock = [modelName '/DemuxQCmd'];
    add_block('simulink/Signal Routing/Demux', demuxBlock, ...
        'Position', [720 100 730 180], 'Outputs', '4');
    add_line(modelName, 'GuidanceControl/2', 'DemuxQCmd/1');

    % TRUE: qCmd is an attitude relative to the INERTIAL frame -- twin_step
    % builds it from ICRF position, velocity and target vectors -- so the
    % bridge must write it with AddAttitude Quat, not CBFQuat. It reads
    % back through 'Attitude Quaternions' because bridge_params_acav.yaml
    % sets actuation.default_attitude_inertial. Sent as false, every
    % command would land in STK rotated by the Earth-rotation angle.
    add_block('simulink/Sources/Constant', ...
        [modelName '/UseInertialFrame'], 'Position', [720 210 750 230], ...
        'Value', 'true', 'OutDataTypeStr', 'boolean');

    addBlankMessage(modelName, 'AttitudeCmdBlank', [790 40 940 60], ...
        'stk_msgs/AttitudeCommand');
    addBusAssignment(modelName, 'AttitudeCmdAssign', [790 90 940 230], ...
        {'orientation.x', 'orientation.y', 'orientation.z', ...
         'orientation.w', 'object_path', 'use_inertial_frame', 'object_path_SL_Info.CurrentLength', ...
         'object_path_SL_Info.ReceivedLength'});
    add_line(modelName, 'AttitudeCmdBlank/1', 'AttitudeCmdAssign/1');
    for k = 1:4
        add_line(modelName, sprintf('DemuxQCmd/%d', k), ...
            sprintf('AttitudeCmdAssign/%d', k + 1));
    end
    add_line(modelName, 'UnpackTelemetry/8', 'AttitudeCmdAssign/6');
    add_line(modelName, 'UseInertialFrame/1', 'AttitudeCmdAssign/7');
    add_line(modelName, 'UnpackTelemetry/10', 'AttitudeCmdAssign/8');
    add_line(modelName, 'UnpackTelemetry/11', 'AttitudeCmdAssign/9');

    % Stamped with the time the commanded attitude is ACHIEVED: twin_step
    % integrates one cfg.stepS from this step's telemetry, so qCmd is the
    % attitude at t + stepS. The bridge writes it to STK at that stamp,
    % which is the next step's time -- exactly where it is read back.
    addStamp(modelName, 'AttitudeCmdStamp', [980 90 1090 130], ...
        'stk_msgs/AttitudeCommand');
    add_block('simulink/Sources/Constant', [modelName '/CmdStampOffsetNs'], ...
        'Position', [920 150 970 170], 'Value', num2str(stepNs));
    add_line(modelName, 'AttitudeCmdAssign/1', 'AttitudeCmdStamp/1');
    add_line(modelName, 'UnpackTelemetry/12', 'AttitudeCmdStamp/2');
    add_line(modelName, 'UnpackTelemetry/13', 'AttitudeCmdStamp/3');
    add_line(modelName, 'CmdStampOffsetNs/1', 'AttitudeCmdStamp/4');

    addPublish(modelName, 'AttitudeCmdPub', [1130 90 1310 130], ...
        sprintf('/%s/cmd/attitude', options.Prefix), ...
        'stk_msgs/AttitudeCommand');
    add_line(modelName, 'AttitudeCmdStamp/1', 'AttitudeCmdPub/1');
    % -- status out: stk_msgs/TwinStatus ---------------------------------
    % Carries the wheel state as well as mode and pointing error, because
    % the run-level quantities acav/metrics.py reports (peak momentum,
    % saturation events, wheel energy) have no other route out of the
    % twin; the scheduler does not re-derive them, as it does not
    % re-derive the pointing error.
    add_block('simulink/Signal Routing/Demux', [modelName '/DemuxMomentum'], ...
        'Position', [720 250 730 310], 'Outputs', '3');
    add_line(modelName, 'GuidanceControl/5', 'DemuxMomentum/1');

    addBlankMessage(modelName, 'TwinStatusBlank', [790 260 940 280], ...
        'stk_msgs/TwinStatus');
    addBusAssignment(modelName, 'TwinStatusAssign', [790 300 940 380], ...
        {'boresight_error_rad', 'wheel_momentum_nms.x', ...
         'wheel_momentum_nms.y', 'wheel_momentum_nms.z', ...
         'wheels_saturated', 'wheel_energy_j', 'saturation_events', ...
         'max_boresight_error_since_last_rad'});
    add_line(modelName, 'TwinStatusBlank/1', 'TwinStatusAssign/1');
    add_line(modelName, 'GuidanceControl/4', 'TwinStatusAssign/2');
    for k = 1:3
        add_line(modelName, sprintf('DemuxMomentum/%d', k), ...
            sprintf('TwinStatusAssign/%d', k + 2));
    end
    add_line(modelName, 'GuidanceControl/6', 'TwinStatusAssign/6');
    add_line(modelName, 'GuidanceControl/7', 'TwinStatusAssign/7');
    % What happened between two statuses (free-run sub-steps): the
    % cumulative saturation count and the largest error since the last.
    add_line(modelName, 'GuidanceControl/13', 'TwinStatusAssign/8');
    add_line(modelName, 'GuidanceControl/14', 'TwinStatusAssign/9');

    % Stamped with this step's own time: boresight_error_rad and the mode
    % describe the MEASURED attitude at t, which is what the scheduler's
    % hold-window check needs.
    addStamp(modelName, 'TwinStatusStamp', [980 300 1090 340], ...
        'stk_msgs/TwinStatus');
    add_block('simulink/Sources/Constant', [modelName '/StatusStampOffsetNs'], ...
        'Position', [920 410 970 430], 'Value', '0');
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [modelName '/EncodeMode'], 'Position', [950 360 1080 400]);
    setMatlabFunctionBody([modelName '/EncodeMode'], encodeModeSource());
    setMessageOutput([modelName '/EncodeMode'], 'stk_msgs/TwinStatus');
    add_line(modelName, 'TwinStatusAssign/1', 'EncodeMode/1');
    add_line(modelName, 'GuidanceControl/3', 'EncodeMode/2');
    add_line(modelName, 'EncodeMode/1', 'TwinStatusStamp/1');
    add_line(modelName, 'UnpackTelemetry/12', 'TwinStatusStamp/2');
    add_line(modelName, 'UnpackTelemetry/13', 'TwinStatusStamp/3');
    add_line(modelName, 'StatusStampOffsetNs/1', 'TwinStatusStamp/4');

    addPublish(modelName, 'TwinStatusPub', [1130 300 1310 340], ...
        options.TwinStatusTopic, 'stk_msgs/TwinStatus');
    add_line(modelName, 'TwinStatusStamp/1', 'TwinStatusPub/1');

    if options.LockstepMode
        % -- lockstep ack: std_msgs/String, no header field to stamp ----
        ackTextBlock = [modelName '/BuildAckText'];
        add_block('simulink/User-Defined Functions/MATLAB Function', ...
            ackTextBlock, 'Position', [720 410 900 440]);
        setMatlabFunctionBody(ackTextBlock, buildAckTextSource());
        setMessageOutput(ackTextBlock, 'std_msgs/String');
        add_line(modelName, 'UnpackTelemetry/9', 'BuildAckText/1');

        addBlankMessage(modelName, 'AckBlank', [940 400 1090 420], ...
            'std_msgs/String');
        add_line(modelName, 'AckBlank/1', 'BuildAckText/2');

        addPublish(modelName, 'LockstepAckPub', [1130 430 1310 470], ...
            sprintf('/%s/lockstep/ack', options.Prefix), ...
            'std_msgs/String');
        add_line(modelName, 'BuildAckText/1', 'LockstepAckPub/1');

        add_block('built-in/Note', [modelName '/AckOrderingNote'], ...
            'Position', [720 490 1160 570], 'Text', ...
            ['Publish the ack AFTER the attitude command for this ' ...
             'step. The bridge releases the barrier on the ack, so ' ...
             'acking first lets STK advance past a step whose command ' ...
             'has not arrived -- which does not error, it just quietly ' ...
             'applies the command one step late for the whole run.']);
    end

    % -- identity out: std_msgs/String, "v3 <cfg> <build> <runtime> <Hz>" -
    % Published on the first ready step and then about once per wall
    % second (GuidanceControl output 12, identityDue), so a late
    % subscriber sees it within seconds. GuidanceControl outputs 10 and
    % 11 are the zero-padded bytes and their length.
    identityTextBlock = [modelName '/BuildIdentityText'];
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        identityTextBlock, 'Position', [720 720 900 760]);
    setMatlabFunctionBody(identityTextBlock, buildIdentityTextSource());
    setMessageOutput(identityTextBlock, 'std_msgs/String');
    add_line(modelName, 'GuidanceControl/10', 'BuildIdentityText/1');
    add_line(modelName, 'GuidanceControl/11', 'BuildIdentityText/2');
    addBlankMessage(modelName, 'IdentityBlank', [940 710 1090 730], ...
        'std_msgs/String');
    add_line(modelName, 'IdentityBlank/1', 'BuildIdentityText/3');
    addPublish(modelName, 'TwinIdentityPub', [1130 720 1310 760], ...
        char(options.TwinIdentityTopic), 'std_msgs/String');
    add_line(modelName, 'BuildIdentityText/1', 'TwinIdentityPub/1');

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
        error('build_acav_twin:WiringInvalid', ...
            'Diagram update failed; model was not saved:\n%s', detail);
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
% Without this the build fails partway through adding blocks, leaving a
% half-constructed model open, with an error that names a mask callback
% ("Error evaluating 'MaskParameterCallback' ... Unrecognized message
% type stk_msgs/AttitudeState") instead of saying to run setup_stk_ros.
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
    % Lockstep uses reliable telemetry (qos.telemetry_reliability).
    % No newer step exists to replace a lost sample while the barrier waits.
    set_param(block, 'topic', topic, 'messageType', messageType, ...
        'QOSReliability', 'Reliable', 'QOSDurability', 'Volatile');
end

function addPublish(model, name, position, topic, messageType)
% Only publish commands, status and barrier acknowledgements after telemetry
% validation. Substituting an identity quaternion would invent a measurement.
    block = [model '/' name];
    add_block('built-in/Subsystem', block, 'Position', position);
    add_block('built-in/Inport', [block '/Message'], 'Position', [25 40 55 60]);
    add_block('built-in/EnablePort', [block '/Enable'], 'Position', [25 90 55 110]);
    add_block('ros2lib/Publish', [block '/Publish'], 'Position', [100 30 240 70]);
    set_param([block '/Publish'], 'topic', topic, 'messageType', messageType, ...
        'QOSReliability', 'Reliable', 'QOSDurability', 'Volatile');
    add_line(block, 'Message/1', 'Publish/1');
    controlPorts = get_param([model '/GuidanceControl'], 'PortHandles');
    publishPorts = get_param(block, 'PortHandles');
    port = 9; % Commands and status are sent only once per telemetry time.
    % The ack repeats on every ready step, for late listeners.
    if strcmp(name, 'LockstepAckPub'), port = 8; end
    % The identity: first ready step, then about once per wall second.
    if strcmp(name, 'TwinIdentityPub'), port = 12; end
    add_line(model, controlPorts.Outport(port), publishPorts.Enable);
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
% left 'entityType', and so the compiled output bus, at that default for
% std_msgs/String (for some stk_msgs types messageType alone was
% enough, so its propagation cannot be relied on).
    block = [model '/' name];
    add_block('ros2lib/Blank Message', block, 'Position', position);
    set_param(block, 'entityType', messageType, 'messageType', messageType);
end

function addBusAssignment(model, name, position, dottedFields)
%ADDBUSASSIGNMENT A Bus Assignment writing each of dottedFields, in
% order, from its own input port. AssignedSignals accepts comma-separated
% dotted paths into nested buses (e.g. "orientation.x"), checked against
% a generated stk_msgs bus.
    block = [model '/' name];
    add_block('simulink/Signal Routing/Bus Assignment', block, ...
        'Position', position);
    set_param(block, 'AssignedSignals', strjoin(dottedFields, ','));
end

function addStamp(model, name, position, messageType)
%ADDSTAMP Stamp a message's header from the telemetry's own sim time.
% Not ros2lib/Header Assignment with InsertTimeStamp: that stamps with
% the Simulink ROS node's clock, which does not follow /clock, so the
% stamps would be wall-clock time. The bridge takes an AttitudeCommand's
% stamp as the STK sample time and the scheduler takes a TwinStatus
% stamp as the sample's time, so a wall-clock stamp would put every
% sample outside the scenario.
    block = [model '/' name];
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        block, 'Position', position);
    setMatlabFunctionBody(block, stampSource());
    setMessageOutput(block, messageType);
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

function setMessageOutput(blockPath, messageType)
    root = sfroot;
    chart = root.find('-isa', 'Stateflow.EMChart', 'Path', blockPath);
    output = chart.find('-isa', 'Stateflow.Data', 'Scope', 'Output', 'Name', 'msg');
    output.DataType = ros.slros2.internal.bus.Util.rosMsgTypeToDataTypeStr(messageType);
end

function source = guidanceControlSource(runConfigPath, buildFingerprint, pollingHz)
    %GUIDANCECONTROLSOURCE The block body: a thin call into +adcs.
    %
    % Thin on purpose: everything of substance lives in adcs.twin_step,
    % where test_acav_twin.m can test it.
    %
    % runConfigPath is baked in as a literal rather than read from a
    % workspace variable at run time: the model is a build artifact
    % regenerated per run anyway, and a model whose behaviour depended
    % on the base workspace would make two runs of the same .slx differ.
    %
    % buildFingerprint (adcs.code_fingerprint at build time) is baked in
    % the same way, so the identity the twin publishes can show that the
    % model is stale relative to the code on disk.
    if nargin < 1
        runConfigPath = "";
    end
    if nargin < 2
        buildFingerprint = "";
    end
    if nargin < 3
        pollingHz = 100;
    end
    source = strjoin({
        'function [torque, qCmd, mode, err, momentum, saturated, energy, ready, fresh, ...'
        '        identity, identityLength, identityDue, saturationEvents, maxErrSinceLast] = ...'
        '        fcn(qMeas, omega, pos, vel, targetEci, haveTarget, t, targetId)'
        '%#codegen'
        '    coder.extrinsic(''adcs.simulink_step'');'
        '    persistent started'
        '    reset = isempty(started);'
        '    started = true;'
        '    torque = zeros(1,3); qCmd = [0 0 0 1]; mode = 1;'
        '    err = 0; momentum = zeros(1,3); saturated = false; energy = 0; ready = false; fresh = false;'
        '    omegaCmd = zeros(1,3); identity = zeros(1,128,''uint8''); identityLength = 0;'
        '    identityDue = false; saturationEvents = uint32(0); maxErrSinceLast = 0;'
        sprintf(['    [torque,qCmd,mode,err,momentum,saturated,energy,ready,fresh,omegaCmd,' ...
            'identity,identityLength,identityDue,saturationEvents,maxErrSinceLast] = ' ...
            'adcs.simulink_step(qMeas,omega,pos,vel,targetEci,haveTarget,t,reset,''%s'',targetId,''%s'',%s);'], ...
            strrep(char(runConfigPath), '''', ''''''), char(buildFingerprint), ...
            num2str(pollingHz, 15))
        'end'
        }, newline);
end

function source = unpackTelemetrySource()
    %UNPACKTELEMETRYSOURCE Flatten the five subscribed ROS buses into the
    % plain numerics adcs.twin_step expects, plus passthroughs the publish
    % side needs but twin_step does not. A MATLAB Function block infers
    % each INPUT's bus type from whatever is wired to it, so this needs no
    % Bus Selector chain.
    %
    % Two time quantities come out, and they are deliberately different:
    %   t                 seconds since the window start -- twin_step's
    %                     documented contract, and what injected fault epochs
    %                     (WheelFault.at_s) are measured against. Using
    %                     absolute epoch seconds here would make every
    %                     injected fault fire on the first step.
    %   stampSec/Nsec     this step's absolute sim time, exactly as the
    %                     bridge sent it, for stamping outgoing headers.
    %                     Integers throughout: no double round trip, so a
    %                     command stamp lands on the bridge's step grid.
    % The telemetry hold compares the three STK-derived headers against
    % the ScenarioTime stamp in integer sec/nanosec for the same reason.
    source = strjoin({
        'function [qMeas, omega, pos, vel, targetEci, haveTarget, t, ...'
        '    objectPath, stepIndex, pathLength, pathReceived, stampSec, stampNsec, targetId] = ...'
        '    fcn(attitudeMsg, orbitMsg, setpointMsg, rateMsg, timeMsg)'
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
        '    targetId = zeros(size(setpointMsg.target_id), ''uint8'');'
        '    n = double(setpointMsg.target_id_SL_Info.CurrentLength);'
        '    if setpointMsg.target_id_SL_Info.ReceivedLength > n'
        '        error(''Target ID exceeds the ROS bus string capacity.'');'
        '    end'
        '    targetId(1:n) = setpointMsg.target_id(1:n);'
        '    stampSec = double(timeMsg.sim_time.sec);'
        '    stampNsec = double(timeMsg.sim_time.nanosec);'
        '    t = (stampSec - double(timeMsg.scenario_epoch.sec)) + ...'
        '        (stampNsec - double(timeMsg.scenario_epoch.nanosec)) * 1e-9;'
        '    heldSec = double([attitudeMsg.header.stamp.sec, ...'
        '        orbitMsg.header.stamp.sec, rateMsg.header.stamp.sec]);'
        '    heldNsec = double([attitudeMsg.header.stamp.nanosec, ...'
        '        orbitMsg.header.stamp.nanosec, rateMsg.header.stamp.nanosec]);'
        '    offsetNs = (heldSec - stampSec) * 1e9 + (heldNsec - stampNsec);'
        '    if any(abs(offsetNs) > 1000)'
        '        qMeas = zeros(1,4); % Hold until all telemetry matches this step.'
        '    end'
        '    objectPath = attitudeMsg.object_path;'
        '    pathLength = attitudeMsg.object_path_SL_Info.CurrentLength;'
        '    pathReceived = attitudeMsg.object_path_SL_Info.ReceivedLength;'
        '    stepIndex = timeMsg.step_index;'
        'end'
        }, newline);
end

function source = buildAckTextSource()
    %BUILDACKTEXTSOURCE The lockstep ack's payload: "matlab_twin:<step>".
    % Matches the format acav_scheduler's own ack already uses
    % ("acav_scheduler:<step_index>", see docs/acav/scheduler.md)
    % so a log reader can tell which participant acked a given step.
    source = strjoin({
        'function msg = fcn(stepIndex, blank)'
        '%#codegen'
        '    msg = blank;'
        '    prefix = uint8(''matlab_twin:'');'
        '    digits = zeros(1,20,''uint8''); n = 0;'
        '    value = uint64(stepIndex);'
        '    while true'
        '        n = n + 1; digits(n) = uint8(rem(value,uint64(10))) + uint8(48);'
        '        value = idivide(value,uint64(10));'
        '        if value == 0, break; end'
        '    end'
        '    msg.data(:) = 0;'
        '    msg.data(1:numel(prefix)) = prefix;'
        '    for k = 1:n, msg.data(numel(prefix)+k) = digits(n-k+1); end'
        '    msg.data_SL_Info.CurrentLength = uint32(numel(prefix)+n);'
        '    msg.data_SL_Info.ReceivedLength = uint32(numel(prefix)+n);'
        'end'
        }, newline);
end

function source = buildIdentityTextSource()
    %BUILDIDENTITYTEXTSOURCE The twin identity as std_msgs/String data.
    % Fails the simulation, rather than truncating, if the message type's
    % string capacity is smaller than the identity: a cut identity would
    % be refused by the runner anyway, with a less useful reason.
    source = strjoin({
        'function msg = fcn(identity, identityLength, blank)'
        '%#codegen'
        '    msg = blank;'
        '    n = min(double(identityLength), numel(identity));'
        '    if n > numel(msg.data)'
        '        error(''Twin identity exceeds the std_msgs/String capacity.'');'
        '    end'
        '    msg.data(:) = 0;'
        '    for k = 1:n, msg.data(k) = identity(k); end'
        '    msg.data_SL_Info.CurrentLength = uint32(n);'
        '    msg.data_SL_Info.ReceivedLength = uint32(n);'
        'end'
        }, newline);
end

function source = stampSource()
    %STAMPSOURCE header.stamp = (sec, nsec) + offsetNs, carried in int64
    % nanoseconds so the result is exact and lands on the bridge's step
    % grid rather than a double-rounding neighbour of it.
    source = strjoin({
        'function msg = fcn(msgIn, sec, nsec, offsetNs)'
        '%#codegen'
        '    msg = msgIn;'
        '    billion = int64(1000000000);'
        '    total = int64(nsec) + int64(offsetNs);'
        '    carry = idivide(total, billion, ''floor'');'
        '    msg.header.stamp.sec = int32(int64(sec) + carry);'
        '    msg.header.stamp.nanosec = uint32(total - carry * billion);'
        'end'
        }, newline);
end

function source = encodeModeSource()
    source = strjoin({
        'function msg = fcn(blank, mode)'
        '%#codegen'
        '    msg = blank;'
        '    labels = uint8([''NADIR '';''SLEW  '';''SETTLE'';''IMAGE '';''SAFE  '']);'
        '    lengths = [5 4 6 5 4];'
        '    idx = min(5,max(1,mode)); n = lengths(idx);'
        '    msg.mode(:) = 0;'
        '    for k = 1:n, msg.mode(k) = labels(idx,k); end'
        '    msg.mode_SL_Info.CurrentLength = uint32(n);'
        '    msg.mode_SL_Info.ReceivedLength = uint32(n);'
        'end'
        }, newline);
end
