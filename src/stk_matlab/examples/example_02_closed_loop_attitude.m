%% Example 2: a trivial closed-loop attitude "controller"
%
% Not real ADCS logic -- a proportional controller nulling the angle
% between the current attitude and a fixed target quaternion, run in
% FREE_RUN or PACED mode (not lockstep; see example_03 for that). This
% is the shortest path to confirming sense -> compute -> actuate ->
% sense-the-effect works end to end before building anything more
% sophisticated in Simulink.
%
% Prerequisites: same as example_01, plus config's time.mode should be
% 'free_run' or 'paced' for this script (lockstep requires driving the
% barrier explicitly -- see example_03).

setup_stk_ros();

client = stkros.BridgeClient("sss_1");
client.waitUntilConnected(30.0);

targetQuaternion = [0, 0, 0.3826834, 0.9238795];   % 45 deg about Z
kp = 0.5;   % proportional gain, dimensionless per this toy law
dt = 0.2;   % script loop period, seconds -- NOT the bridge's step_s;
            % see docs/bridge/time_synchronization.md on why these differ.

fprintf('Driving toward target quaternion [%.4f %.4f %.4f %.4f]\n', ...
    targetQuaternion);

for iteration = 1:100
    pause(dt);

    attitude = client.Telemetry.latestAttitude();
    if isempty(attitude)
        continue   % no sample yet; nothing to control against
    end

    current = attitude.quaternion_xyzw;
    errorQuaternion = quaternionError(current, targetQuaternion);
    errorAngleDeg = 2 * acosd(min(1, abs(errorQuaternion(4))));

    if mod(iteration, 10) == 0
        fprintf('[%3d] error = %.3f deg\n', iteration, errorAngleDeg);
    end
    if errorAngleDeg < 0.5
        fprintf('Converged after %d iterations.\n', iteration);
        break
    end

    % Toy law: slerp a small step toward the target rather than
    % integrating a real rate-based controller -- adequate for
    % confirming the pipeline, not for actual ADCS design.
    step = min(1.0, kp * dt);
    command = slerp(current, targetQuaternion, step);

    client.Commands.sendAttitude(posixtime(datetime('now', 'TimeZone', 'UTC')), ...
        command);
end

function qerr = quaternionError(q, qTarget)
    % qerr = qTarget * conj(q), all as (x,y,z,w)
    qc = [-q(1), -q(2), -q(3), q(4)];
    qerr = quaternionMultiply(qTarget, qc);
end

function q = quaternionMultiply(a, b)
    ax=a(1); ay=a(2); az=a(3); aw=a(4);
    bx=b(1); by=b(2); bz=b(3); bw=b(4);
    q = [aw*bx + ax*bw + ay*bz - az*by, ...
         aw*by - ax*bz + ay*bw + az*bx, ...
         aw*bz + ax*by - ay*bx + az*bw, ...
         aw*bw - ax*bx - ay*by - az*bz];
end

function q = slerp(q0, q1, t)
    dotp = dot(q0, q1);
    if dotp < 0
        q1 = -q1; dotp = -dotp;   % shortest path (q and -q are the same
                                   % orientation -- see conversions.py's
                                   % quaternion_angle_between for why this
                                   % double-cover handling matters)
    end
    dotp = min(1, max(-1, dotp));
    theta = acos(dotp) * t;
    q2 = q1 - q0 * dotp;
    q2 = q2 / norm(q2);
    q = q0 * cos(theta) + q2 * sin(theta);
    q = q / norm(q);
end
