function profile = slew_profile(qStart, qFinish, alpha, omegaMax)
%SLEW_PROFILE Rest-to-rest eigenaxis reference trajectory.
%
%   p = adcs.slew_profile(q0, q1, alpha, omegaMax)
%   [q, w] = p.referenceAt(t)
%
% Triangular below the crossover angle omegaMax^2/alpha, trapezoidal
% above; the two agree at the crossover. It is the same model
% acav/slew_model.py gives the scheduler, so that the scheduler's
% slew-time estimate and the twin's reference trajectory agree.
%
% The limits passed in are the derated ones (TwinConfig.profile_limits):
% at full authority the feedforward term alone demands all the wheel
% torque, leaving the feedback loop nothing to correct tracking error.
%
% Transcribed from acav/twin_reference.py::SlewProfile.

    qStart = adcs.quat.normalize(qStart);
    qFinish = adcs.quat.normalize(qFinish);
    [axis, totalAngle] = adcs.quat.toAxisAngle( ...
        adcs.quat.error(qFinish, qStart));

    crossover = omegaMax^2 / alpha;
    if totalAngle <= crossover
        duration = 2 * sqrt(totalAngle / alpha);
    else
        duration = omegaMax / alpha + totalAngle / omegaMax;
    end

    profile = struct( ...
        'start', qStart, 'finish', qFinish, 'axis', axis, ...
        'totalAngle', totalAngle, 'alpha', alpha, ...
        'omegaMax', omegaMax, 'duration', duration, ...
        'angleAt', @angleAt, 'referenceAt', @referenceAt);

    function [angle, rate] = angleAt(t)
        if totalAngle <= 0 || t <= 0
            angle = 0; rate = 0; return
        end
        if t >= duration
            angle = totalAngle; rate = 0; return
        end
        if totalAngle <= crossover
            half = duration / 2;
            if t <= half
                angle = 0.5 * alpha * t^2;
                rate = alpha * t;
            else
                remaining = duration - t;
                angle = totalAngle - 0.5 * alpha * remaining^2;
                rate = alpha * remaining;
            end
        else
            ramp = omegaMax / alpha;
            if t <= ramp
                angle = 0.5 * alpha * t^2;
                rate = alpha * t;
            elseif t <= duration - ramp
                angle = 0.5 * omegaMax * ramp + omegaMax * (t - ramp);
                rate = omegaMax;
            else
                remaining = duration - t;
                angle = totalAngle - 0.5 * alpha * remaining^2;
                rate = alpha * remaining;
            end
        end
    end

    function [qRef, omegaRef] = referenceAt(t)
        [angle, rate] = angleAt(t);
        qRef = adcs.quat.normalize(adcs.quat.multiply( ...
            qStart, adcs.quat.fromAxisAngle(axis, angle)));
        omegaRef = axis * rate;
    end
end
