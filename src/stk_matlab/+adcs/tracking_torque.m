function torque = tracking_torque(q, omega, qRef, omegaRef, kp, kd)
%TRACKING_TORQUE Quaternion feedback with rate feedforward.
%
%   tau = adcs.tracking_torque(q, w, qRef, wRef, Kp, Kd)
%
%   tau = -Kp .* sign(qe_w) .* qe_v  -  Kd .* (w - wRef)
%
% The sign(qe_w) term selects the short way round: without it the
% controller can take the long way round a 350 deg error, which still
% converges but triples the settling time. The feedforward lets the
% loop track the reference profile rather than lag it, which keeps
% achieved slew times near the analytic bound.
%
% Kp and Kd are per axis. The bus inertia spans 0.041 to 0.123 kg m^2;
% one scalar gain would make z three times faster than x and the
% response direction-dependent for no physical reason.
%
% Transcribed from acav/twin_reference.py::tracking_torque.

    qe = adcs.quat.error(q, qRef);
    if qe(4) >= 0
        s = 1;
    else
        s = -1;
    end
    torque = -kp(:).' .* (s * qe(1:3)) ...
             - kd(:).' .* (omega(:).' - omegaRef(:).');
end
