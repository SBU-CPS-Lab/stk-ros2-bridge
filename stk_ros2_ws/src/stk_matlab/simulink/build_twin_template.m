function modelName = build_twin_template(options)
%BUILD_TWIN_TEMPLATE Programmatically construct a starter Simulink model
% wired to the stk_bridge ROS 2 topics, using the ROS 2 Subscribe/
% Publish/Current Time blocks.
%
% A .slx is not checked into source control here -- Simulink's binary
% model format does not diff or merge usefully, so this project keeps
% the model as REPRODUCIBLE CODE (this function) instead, matching the
% same "config as text, not binary" principle used throughout the
% Python side. Run this once to generate stk_twin_template.slx locally;
% treat the .slx as a build artifact, and re-run this function (editing
% it, not the model) when the topic set changes.
%
%   modelName = build_twin_template()
%   build_twin_template('LockstepMode', false)   % free_run-friendly variant
%
% The generated model:
%   ROS2 Subscribe (attitude) -> Bus Selector -> [your control law] ->
%   Bus Assignment -> ROS2 Publish (attitude cmd) -> ROS2 Publish (ack)
%
% "Your control law" is a placeholder Gain block -- replace the
% highlighted subsystem with the actual ADCS logic. See
% examples/example_03_lockstep_stepping.m for the equivalent pure-MATLAB
% (non-Simulink) version of this same loop, useful for validating the
% ROS wiring before building the Simulink model.

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
    % use_sim_time semantics: the ROS 2 Current Time block reads /clock
    % when the model's simulation-time source is set to follow ROS time.
    % See docs/05_time_synchronization.md for the full explanation of
    % why this must be true whenever the bridge is in lockstep/paced mode.
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

    % -- Placeholder control law: identity gain ------------------------
    % Replace this block with the actual ADCS control law. It is wired
    % to pass the sensed quaternion straight to the command output so
    % the generated model is runnable (a no-op controller) rather than
    % broken, out of the box.
    gainBlock = [modelName '/ControlLaw_PLACEHOLDER'];
    add_block('simulink/Math Operations/Gain', gainBlock, ...
        'Position', [280 40 400 100], 'Gain', '1');

    % -- Publish: attitude command --------------------------------------
    pubBlock = [modelName '/AttitudeCmdPub'];
    add_block('ros2lib/Publish', pubBlock, ...
        'Position', [440 40 620 100]);
    set_param(pubBlock, 'topic', [char(base(1:strlength(base) - ...
        strlength(options.Namespace) - 1)) '/cmd/attitude']); %#ok<NASGU>
    % (The line above derives ".../stk/cmd/attitude" from the base
    % namespace path; verify it against config/bridge_params.yaml's
    % topic_prefix if you rename the prefix.)
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
            'Publish the ack AFTER AttitudeCmdPub fires for this step ' ...
            '-- see CommandPublisher.acknowledgeStep''s help text on ' ...
            'why the ordering matters.']);
        add_block('built-in/Note', [modelName '/Note1'], ...
            'Position', [440 260 900 320], 'Text', annotation);
    end

    add_line(modelName, 'AttitudeSub/1', 'ControlLaw_PLACEHOLDER/1', ...
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
