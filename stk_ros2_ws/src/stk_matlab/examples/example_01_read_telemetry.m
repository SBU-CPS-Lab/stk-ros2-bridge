%% Example 1: read SSS-1 telemetry from the bridge
%
% The minimum useful MATLAB script: connect, wait for data, print it.
% Run this FIRST against a live bridge before attempting anything
% closed-loop -- if this doesn't work, nothing downstream will either.
%
% Prerequisites:
%   1. stk_ros2_ws colcon-built, sourced (install/local_setup.ps1)
%   2. ros2 launch stk_bridge stk_bridge_mock.launch.py running
%      (mock backend needs no STK -- start here before pointing at a
%      real install; see docs/06_troubleshooting.md if this launch
%      itself does not come up healthy)
%   3. setup_stk_ros() has been run once in this MATLAB session

setup_stk_ros();

client = stkros.BridgeClient("sss_1");
fprintf('Waiting for the bridge to report CONNECTED...\n');
client.waitUntilConnected(30.0);
fprintf('Connected. Backend: %s, STK: %s\n', ...
    client.Status.backend_type, client.Status.stk_version);

% Telemetry arrives asynchronously via ROS 2 callbacks. In a plain
% script (not a Simulink model, which has its own scheduling), you must
% either call pause()/drawnow to let callbacks fire, or use a loop --
% MATLAB does not pump the ROS 2 executor otherwise.
for i = 1:10
    pause(0.5);

    orbit = client.Telemetry.latestOrbit();
    attitude = client.Telemetry.latestAttitude();

    if isempty(orbit)
        fprintf('[%d] no orbit sample yet\n', i);
        continue
    end

    altitude_km = (norm(orbit.position) - 6378137.0) / 1000.0;
    fprintf('[%d] t=%.2f  pos=[%.0f %.0f %.0f] m  |r|-Re=%.1f km  frame=%s%s\n', ...
        i, orbit.stamp, orbit.position, altitude_km, orbit.frame_id, ...
        ternary(orbit.is_extrapolated, '  (extrapolated)', ''));

    if ~isempty(attitude)
        fprintf('     attitude (x,y,z,w) = [%.4f %.4f %.4f %.4f]  frame=%s\n', ...
            attitude.quaternion_xyzw, attitude.frame_id);
    end
end

function s = ternary(cond, a, b)
    if cond, s = a; else, s = b; end
end
