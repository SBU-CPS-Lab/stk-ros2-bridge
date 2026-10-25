%% Example 3: driving the bridge's lockstep barrier from MATLAB
%
% Demonstrates the full lockstep pattern: sense, compute, command,
% acknowledge -- in that order, matching core/time_sync.py's step
% sequence (advance -> publish -> await_barrier) and CommandPublisher.
% acknowledgeStep's ordering requirement (ack AFTER the command that
% applies to this step, never before).
%
% Two ways to drive lockstep:
%   A) time.auto_step: true (bridge_params.yaml default) -- the bridge
%      steps continuously on its own loop thread; MATLAB's only job is
%      to ack each step promptly. THIS example uses that mode.
%   B) time.auto_step: false -- nothing advances until something calls
%      the StepSimulation service; use client.Services.stepSimulation()
%      to drive it explicitly if you want MATLAB, not the bridge, to
%      own the pacing.
%
% Prerequisites: bridge_params.yaml time.mode: lockstep,
% barrier_participants including "matlab_twin" (the default).

setup_stk_ros();

client = stkros.BridgeClient("sss_1", "ParticipantName", "matlab_twin");
client.waitUntilConnected(30.0);
fprintf('Connected. Running %d lockstep steps.\n', 50);

lastAckedStep = -1;
targetQuaternion = [0, 0, 0.3826834, 0.9238795];

for iteration = 1:50
    % In auto_step mode the bridge is already advancing; this loop's
    % job is to notice a NEW step happened (via a fresh telemetry
    % sample) and ack it -- not to drive the stepping itself. Polling
    % at a period well under the bridge's step_s (0.1s default) avoids
    % missing steps; a production Simulink model instead reacts to each
    % ROS2 Subscribe firing, which is event-driven rather than polled.
    pause(0.02);

    attitude = client.Telemetry.latestAttitude();
    if isempty(attitude)
        continue
    end

    % The step index isn't on AttitudeState directly; a real controller
    % correlates against stk/scenario_time (ScenarioTime.msg's
    % step_index) rather than inferring it. Simplified here for
    % brevity -- see stk_msgs/ScenarioTime.msg.
    currentStep = lastAckedStep + 1;

    % -- sense --
    current = attitude.quaternion_xyzw;

    % -- compute (see example_02 for the toy control law used here) --
    command = current;   % PLACEHOLDER: replace with real ADCS logic

    % -- actuate --
    client.Commands.sendAttitude(attitude.stamp, command);

    % -- acknowledge (must come AFTER the command above) --
    client.Commands.acknowledgeStep("matlab_twin", currentStep);
    lastAckedStep = currentStep;

    if mod(iteration, 10) == 0
        fprintf('[%3d] acked step %d, sim_time=%.3f\n', ...
            iteration, currentStep, attitude.stamp);
    end
end

fprintf(['Done. If steps were being missed (visible as barrier ' ...
         'timeouts in the bridge log with policy=continue/abort), ' ...
         'reduce this loop''s pause() below bridge_params.yaml''s ' ...
         'time.step_s, or switch to an event-driven Simulink model ' ...
         'instead of a polled script for anything beyond bring-up.\n']);
