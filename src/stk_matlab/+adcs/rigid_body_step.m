function [qOut, omegaOut] = rigid_body_step(q, omega, torqueBody, ...
        wheelMomentum, inertia, dt)
%RIGID_BODY_STEP One RK4 step of rigid-body attitude dynamics.
%
%   [q, w] = adcs.rigid_body_step(q, w, tau, hWheel, J, dt)
%
% Euler's equation with the wheel momentum included in the total, so
% gyroscopic coupling through a spinning wheel is present. It is small
% at these momenta, but omitting it would make the twin differ from a
% real vehicle in exactly the regime E5 explores.
%
% RK4 rather than Euler: at 0.1 s with a 1 rad/s bandwidth, forward
% Euler adds a slow energy drift that reads as a controller which never
% quite settles -- which is precisely the quantity this study reports,
% so the integrator must not contribute to it.
%
% inertia is [Jxx Jyy Jzz]; products of inertia are neglected, as
% acav_scenario.yaml documents.
%
% Transcribed from acav/twin_reference.py::RigidBody.

    q0 = q(:).';
    w0 = omega(:).';

    [k1q, k1w] = derivative(q0, w0);
    [k2q, k2w] = derivative(q0 + k1q * dt/2, w0 + k1w * dt/2);
    [k3q, k3w] = derivative(q0 + k2q * dt/2, w0 + k2w * dt/2);
    [k4q, k4w] = derivative(q0 + k3q * dt,   w0 + k3w * dt);

    qOut = adcs.quat.normalize( ...
        q0 + dt/6 * (k1q + 2*k2q + 2*k3q + k4q));
    omegaOut = w0 + dt/6 * (k1w + 2*k2w + 2*k3w + k4w);

    function [qd, wd] = derivative(qq, ww)
        total = inertia(:).' .* ww + wheelMomentum(:).';
        gyroscopic = cross(ww, total);
        wd = (torqueBody(:).' - gyroscopic) ./ inertia(:).';
        qd = adcs.quat.derivative(qq, ww);
    end
end
