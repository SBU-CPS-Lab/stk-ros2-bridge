function [delivered, momentumOut, saturated] = wheels(torqueBody, dt, ...
        momentumIn, maxTorque, momentumCapacity, health)
%WHEELS Three orthogonal reaction wheels aligned with the body axes.
%
%   [tau, h, sat] = adcs.wheels(tauCmd, dt, h0, tauMax, hMax, health)
%
% Torque- and momentum-limited. The momentum limit is the one that
% matters: it trips SAFE and it is the injected saturation fault. Saturation is a
% hard clamp, not a soft roll-off, because that makes the failure abrupt
% and therefore detectable -- a gentle roll-off degrades pointing
% gradually and reads as a tuning problem.
%
% The wheel absorbs the reaction, so its momentum changes by -tau*dt. A
% sign error here makes momentum unload where it should build and the
% SAFE trip never fires.
%
% health is a per-axis derate, 1 = healthy, 0 = dead (a 'failure' fault
% drives one axis to zero).
%
% Transcribed from acav/twin_reference.py::ReactionWheels; agreement is
% asserted by test_acav_twin.m against generated fixtures.

    if nargin < 6 || isempty(health)
        health = [1 1 1];
    end
    torqueBody = torqueBody(:).';
    momentumIn = momentumIn(:).';

    delivered = zeros(1, 3);
    momentumOut = zeros(1, 3);
    saturated = false;

    for axis = 1:3
        limit = maxTorque * health(axis);
        request = max(-limit, min(limit, torqueBody(axis)));
        if abs(torqueBody(axis)) > limit + 1e-15
            saturated = true;
        end

        newMomentum = momentumIn(axis) - request * dt;
        if abs(newMomentum) > momentumCapacity
            clamped = sign(newMomentum) * momentumCapacity;
            % Only the part the wheel could absorb reaches the body. A
            % saturated wheel can still torque back towards zero --
            % clamping both directions would make saturation permanent
            % and turn every E5 run into a termination rather than a
            % fault injection.
            if dt > 0
                request = (momentumIn(axis) - clamped) / dt;
            else
                request = 0;
            end
            newMomentum = clamped;
            saturated = true;
        end

        delivered(axis) = request;
        momentumOut(axis) = newMomentum;
    end
end
