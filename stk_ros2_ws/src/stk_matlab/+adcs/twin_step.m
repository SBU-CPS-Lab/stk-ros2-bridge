function out = twin_step(in, cfg, state)
%TWIN_STEP One guidance-and-control step of the ACAV twin.
%
%   out = adcs.twin_step(in, cfg, state)
%
% Everything the Simulink model does between reading telemetry and
% publishing a command, in one testable function. The model is then
% wiring: a MATLAB Function block calling this, a Stateflow chart owning
% the mode, and ROS 2 Subscribe/Publish blocks either side.
%
% Keeping the algorithm here rather than in the block means it can be
% exercised by test_acav_twin.m against the Python reference, and that a
% gain change is a diff rather than a re-export of a binary model.
%
% in     .quaternion   [x y z w] body attitude w.r.t. inertial (from STK)
%        .omega        [wx wy wz] body rates, rad/s
%        .position     [x y z] inertial position, m
%        .velocity     [x y z] inertial velocity, m/s
%        .targetEci    [x y z] inertial target position, m (0 if none)
%        .haveTarget   logical
%        .t            seconds since the window start
% cfg    from adcs.config()
% state  .mode         'NADIR' | 'SLEW' | 'SETTLE' | 'IMAGE' | 'SAFE'
%        .enteredAt    seconds
%        .wheelMomentum [3]
%        .profile      [] or a struct from adcs.slew_profile
%        .slewStarted  seconds
%
% out    .torque       commanded body torque, N m (what the wheels get)
%        .quaternionCmd the attitude to send STK -- the ACHIEVED one, not
%                       the reference: the whole study turns on STK
%                       computing access from what the vehicle actually
%                       did, so publishing the reference would quietly
%                       reinstate the perfect-pointing assumption the
%                       paper exists to challenge.
%        .state        updated state
%        .boresightErrorRad
%        .modeChanged

    % -- guidance: where should the boresight be pointing right now? ---
    if in.haveTarget
        los = in.targetEci(:).' - in.position(:).';
        qDesired = adcs.quat.align(cfg.boresightBody, los / norm(los), ...
            [1 0 0], in.velocity);
    else
        nadir = -in.position(:).' / norm(in.position);
        qDesired = adcs.quat.align(cfg.boresightBody, nadir, ...
            [1 0 0], in.velocity);
    end

    boresightErrorRad = adcs.quat.boresightError(in.quaternion, ...
        qDesired, cfg.boresightBody);
    rateMagnitude = norm(in.omega);
    momentumFraction = max(abs(state.wheelMomentum)) ...
        / cfg.momentumCapacityNms;

    % -- has a new slew been asked for? --------------------------------
    % A slew is requested when the commanded attitude has moved further
    % than the settle-entry threshold: below that the tracking loop can
    % absorb it without a profile, and re-planning a profile every step
    % would restart the ramp continuously and never converge.
    slewRequested = in.haveTarget && ...
        boresightErrorRad > cfg.settleEntryRad && ...
        ~strcmp(state.mode, 'SLEW');

    profileDone = isempty(state.profile) || ...
        (in.t - state.slewStarted) >= state.profile.duration;

    previousMode = state.mode;
    [state.mode, state.enteredAt] = adcs.mission_mode(state.mode, ...
        state.enteredAt, in.t, boresightErrorRad, rateMagnitude, ...
        momentumFraction, slewRequested, profileDone, cfg);
    modeChanged = ~strcmp(previousMode, state.mode);

    if modeChanged && strcmp(state.mode, 'SLEW')
        state.profile = adcs.slew_profile(in.quaternion, qDesired, ...
            cfg.profileAlphaRadS2, cfg.profileOmegaMaxRadS);
        state.slewStarted = in.t;
    end

    % -- reference trajectory -------------------------------------------
    switch state.mode
        case 'SLEW'
            [qRef, omegaRef] = state.profile.referenceAt( ...
                in.t - state.slewStarted);
        case 'SAFE'
            % Null the rates and stop pointing at anything. Holding the
            % last reference would keep the wheels loaded against a
            % target they cannot reach.
            qRef = in.quaternion;
            omegaRef = [0 0 0];
        otherwise
            qRef = qDesired;
            omegaRef = [0 0 0];
    end

    if strcmp(state.mode, 'SAFE')
        % Rate damping only: -Kd * omega, no attitude term.
        torque = -cfg.kd .* in.omega(:).';
    else
        torque = adcs.tracking_torque(in.quaternion, in.omega, ...
            qRef, omegaRef, cfg.kp, cfg.kd);
    end

    out = struct( ...
        'torque', torque, ...
        'quaternionCmd', in.quaternion, ...
        'state', state, ...
        'boresightErrorRad', boresightErrorRad, ...
        'modeChanged', modeChanged);
end
