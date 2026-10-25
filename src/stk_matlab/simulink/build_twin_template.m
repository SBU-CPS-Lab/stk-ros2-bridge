function modelName = build_twin_template(options)
%BUILD_TWIN_TEMPLATE Programmatically construct a starter Simulink model
% wired to the stk_bridge ROS 2 topics, using the ROS 2 Subscribe/
% Publish/Current Time blocks.
%
% No .slx is kept in source control, because Simulink's binary format
% does not diff or merge usefully; the model is reproducible code
% (this function). Run it to generate stk_twin_template.slx locally,
% treat the .slx as a build artifact, and edit this function, not the
% model, when the topic set changes.
%
%   modelName = build_twin_template()
%   build_twin_template('LockstepMode', false)   % free_run-friendly variant
%
% The generated model contains the ROS 2 endpoints of a twin:
%   ROS 2 Subscribe (attitude) -> [your control law] ->
%   ROS 2 Publish (attitude command), plus ROS 2 Publish (lockstep ack)
%
% "Your control law" is a placeholder block that receives the
% Subscribe block's message output (output 2; output 1 is the boolean
% IsNew). The model does not compile until the placeholder is replaced
% by a bus-aware control law that fills an stk_msgs/AttitudeCommand
% (Bus Selector -> logic -> Blank Message -> Bus Assignment).
% simulink/build_acav_twin.m is a complete, working example of such a
% model; examples/example_03_lockstep_stepping.m is the equivalent
% loop in plain MATLAB, useful for checking the ROS wiring first.

    arguments
        options.ModelName (1,1) string = "stk_twin_template"
        options.Namespace (1,1) string = "sss_1"
        options.Prefix (1,1) string = "stk"
        options.LockstepMode (1,1) logical = true
    end

    modelName = char(options.ModelName);
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
    new_system(modelName);
    open_system(modelName);

    set_param(modelName, 'EnableSTLDoubleBuf', 'on');   %#ok<*NASGU>
    % Some releases offer a ROS time-source model parameter; where it does
    % not exist, stamp messages from stk/scenario_time instead of the
    % ROS 2 Current Time block, which returns wall-clock time (see
    % docs/bridge/time_synchronization.md).
    try
        set_param(modelName, 'ROSTimeSource', 'ROSClock'); %#ok<TRYNC>
    catch
        warning('build_twin_template:TimeSourceParam', ...
            ['could not set ROSTimeSource on this MATLAB release; set ' ...
             'it manually via the ROS 2 Toolbox model configuration, ' ...
             'or Simulation > Model Configuration Parameters > ROS 2.']);
    end

    base = sprintf('/%s/%s', options.Prefix, options.Namespace);

    % -- Subscribe: attitude telemetry --------------------------------
    subBlock = [modelName '/AttitudeSub'];
    add_block('ros2lib/Subscribe', subBlock, ...
        'Position', [40 40 220 100]);
    set_param(subBlock, 'topic', [char(base) '/attitude']);
    set_param(subBlock, 'messageType', 'stk_msgs/AttitudeState');

    % -- Placeholder control law ---------------------------------------
    % Replace this block with the actual ADCS control law (see the header).
    gainBlock = [modelName '/ControlLaw_PLACEHOLDER'];
    add_block('simulink/Math Operations/Gain', gainBlock, ...
        'Position', [280 40 400 100], 'Gain', '1');

    % -- Publish: attitude command --------------------------------------
    pubBlock = [modelName '/AttitudeCmdPub'];
    add_block('ros2lib/Publish', pubBlock, ...
        'Position', [440 40 620 100]);
    set_param(pubBlock, 'topic', sprintf('/%s/cmd/attitude', options.Prefix));
    set_param(pubBlock, 'messageType', 'stk_msgs/AttitudeCommand');

    % -- Publish: lockstep ack (only if LockstepMode) --------------------
    if options.LockstepMode
        ackBlock = [modelName '/LockstepAckPub'];
        add_block('ros2lib/Publish', ackBlock, ...
            'Position', [440 160 620 220]);
        set_param(ackBlock, 'topic', sprintf('/%s/lockstep/ack', ...
            options.Prefix));
        set_param(ackBlock, 'messageType', 'std_msgs/String');

        note = [modelName '/LockstepNote'];
        add_block('simulink/Sinks/Terminator', note, ...
            'Position', [660 160 690 190]);

        annotation = sprintf(['Wire a "Constant" or "MATLAB Function" ' ...
            'block producing a std_msgs/String with data = ' ...
            '"matlab_twin:<step_index>" into LockstepAckPub.\n' ...
            'Publish the ack after AttitudeCmdPub fires for this step ' ...
            '(see CommandPublisher.acknowledgeStep).']);
        add_block('built-in/Note', [modelName '/Note1'], ...
            'Position', [440 260 900 320], 'Text', annotation);
    end

    add_line(modelName, 'AttitudeSub/2', 'ControlLaw_PLACEHOLDER/1', ...
        'autorouting', 'on');
    add_line(modelName, 'ControlLaw_PLACEHOLDER/1', 'AttitudeCmdPub/1', ...
        'autorouting', 'on');

    save_system(modelName, fullfile(fileparts(mfilename('fullpath')), ...
        [modelName '.slx']));

    fprintf(['build_twin_template: wrote %s.slx\n' ...
             '  Replace ControlLaw_PLACEHOLDER with your ADCS logic.\n' ...
             '  Run stk_bus_definitions() first if you want Bus-typed ' ...
             'signals instead of raw ROS structs on these lines.\n'], ...
        modelName);
end
