function profile = slew_profile(qStart, qFinish, alpha, omegaMax)
%SLEW_PROFILE Rest-to-rest eigenaxis reference trajectory.
%
%   p = adcs.slew_profile(q0, q1, alpha, omegaMax)
%   [q, w] = p.referenceAt(t)
%
% Triangular below the crossover angle omegaMax^2/alpha, trapezoidal
% above; the two agree at the crossover. This is the SAME model
% acav/slew_model.py gives the scheduler, and sharing it is what makes
% E2's comparison meaningful -- two independently written slew models
% would have E2 measure their disagreement rather than the effect under
% study (see the C1 spec).
%
% NOTE the limits passed in are the DERATED ones
% (TwinConfig.profile_limits): at full authority the feedforward term
% alone demands every newton-metre the wheels have, leaving the feedback
% loop nothing to correct tracking error with. Measured: the largest
% scenario slew then settled 9.8 s past the analytic bound against an
% 8 s budget.
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
