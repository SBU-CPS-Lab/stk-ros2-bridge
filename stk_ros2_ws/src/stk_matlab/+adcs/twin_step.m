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
%        .targetId     stable target identity (optional for legacy callers)
%        .t            seconds since the window start
% cfg    from adcs.config()
% state  .mode         'NADIR' | 'SLEW' | 'SETTLE' | 'IMAGE' | 'SAFE'
%        .enteredAt    seconds
%        .wheelMomentum [3]
%        .wheelEnergyJ running integral of |tau . omega|, joules
%        .faultApplied whether E5's one-shot saturation has been applied
%        .profile      [] or a struct from adcs.slew_profile
%        .slewStarted  seconds
%
% out    .torque       body torque the wheels actually DELIVERED, N m --
%                       after torque and momentum saturation, so it is
%                       what moved the vehicle rather than what was asked
%        .quaternionCmd the attitude to send STK -- the ACHIEVED one, not
%                       the reference: the whole study turns on STK
%                       computing access from what the vehicle actually
%                       did, so publishing the reference would quietly
%                       reinstate the perfect-pointing assumption the
%                       paper exists to challenge.
%        .omegaNext    body rates after this step, rad/s
%        .state        updated state: wheel momentum and the running
%                       wheel-energy integral as well as the mode
%        .boresightErrorRad
%        .modeChanged
%        .wheelsSaturated
%
% Why this function integrates the dynamics
% -----------------------------------------
% It did not, and that was a real defect rather than a simplification:
% quaternionCmd was `in.quaternion` returned verbatim, so the twin read
% an attitude from STK, computed a torque, and published back the
% attitude it had just read. Nothing anywhere integrated that torque --
% the loop had no integrator at all, the vehicle could never slew, and
% every capture would have been decided by wherever it already happened
% to point. It survived C3 because this is the one function with no
% counterpart in acav/twin_reference.py, so no fixture could see it.
%
% The note that justified it -- "in the closed loop STK is the truth and
% the dynamics are bypassed" -- misread what STK does with injected
% attitude: AddAttitude CBFQuat stores and interpolates samples, it does
% not propagate attitude dynamics. STK is the truth for orbit, geometry
% and access; the vehicle's attitude RESPONSE is the twin's to compute,
% and it is computed here.

    % -- E5's injected fault -------------------------------------------
    % Applied to the state going INTO this step, before the momentum
    % guard is evaluated below, so the chart trips SAFE on the very step
    % the fault lands rather than one step later. A one-step lag would
    % be invisible in a plot and would make the injection epoch in the
    % record disagree with the epoch the run actually reacted to.
    %
    % One-shot: the wheel is filled once and ordinary physics takes over,
    % so it can still unload. Re-filling it every step would pin it at
    % capacity forever -- a termination rather than a fault injection,
    % which is the same reasoning behind adcs.wheels letting a saturated
    % wheel torque back towards zero.
    wheelFault = [];
    if isfield(cfg, 'wheelFault')
        wheelFault = cfg.wheelFault;
    end
    [wheelHealth, saturates] = adcs.wheel_fault(wheelFault, in.t);
    if saturates && ~state.faultApplied
        axisIndex = wheelFault.axis + 1;
        % Follow whichever way the wheel was already spinning, so one
        % that is unloading is not abruptly reversed; at rest it fills
        % positive. Either way the choice follows the trajectory, so a
        % replay makes it identically.
        if state.wheelMomentum(axisIndex) >= 0
            direction = 1;
        else
            direction = -1;
        end
        state.wheelMomentum(axisIndex) = ...
            direction * cfg.momentumCapacityNms;
        state.faultApplied = true;
    end

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

    % Track the moving line of sight, rather than damping its required
    % angular velocity to zero. Express the reference rate in actual body axes.
    % Different targets must never share a finite-difference rate history.
    % Position itself cannot identify a target: it moves continuously in ICRF.
    targetId = '';
    if in.haveTarget && isfield(in, 'targetId'), targetId = char(in.targetId); end
    targetKey = {logical(in.haveTarget), targetId};
    desiredRate = [0 0 0];
    if isfield(state, 'previousTargetKey') && isequal(state.previousTargetKey, targetKey) && ...
            isfield(state, 'previousDesired') && ~isempty(state.previousDesired) && ...
            in.t > state.previousDesiredTime
        delta = adcs.quat.multiply(adcs.quat.conjugate(state.previousDesired), qDesired);
        [axis, angle] = adcs.quat.toAxisAngle(delta);
        referenceRate = axis * (angle / (in.t-state.previousDesiredTime));
        desiredRate = adcs.quat.toBody(in.quaternion, ...
            adcs.quat.toInertial(qDesired, referenceRate));
    end
    state.previousTargetKey = targetKey;
    state.previousDesired = qDesired;
    state.previousDesiredTime = in.t;

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
            omegaRef = desiredRate;
    end

    if strcmp(state.mode, 'SAFE')
        % Rate damping only: -Kd * omega, no attitude term. The point of
        % SAFE is to stop the vehicle tumbling; commanding zero torque
        % would leave whatever rate tripped the guard exactly where it
        % was, and E5 exists to exercise that recovery. The Python
        % reference commanded zero here until this was reconciled --
        % see twin_reference.control_torque, now the single place the
        % choice is made on that side.
        torqueRequest = -cfg.kd .* in.omega(:).';
    else
        torqueRequest = adcs.tracking_torque(in.quaternion, in.omega, ...
            qRef, omegaRef, cfg.kp, cfg.kd);
    end

    % -- actuators, then dynamics ---------------------------------------
    % The wheels clamp the request in torque and in momentum, and only
    % what they could absorb reaches the body. Integrating the REQUEST
    % instead would give the vehicle authority it does not have and make
    % E5's saturation faults invisible.
    [torque, state.wheelMomentum, wheelsSaturated] = adcs.wheels( ...
        torqueRequest, cfg.stepS, state.wheelMomentum, cfg.maxTorqueNm, ...
        cfg.momentumCapacityNms, wheelHealth);

    [quaternionCmd, omegaNext] = adcs.rigid_body_step(in.quaternion, ...
        in.omega, torque, state.wheelMomentum, cfg.inertia, cfg.stepS);

    % Power against the rate the torque actually acted on -- the rate at
    % the START of the step, not the integrated one, matching the
    % left-hand rule the rest of the stepping uses. Using the rate after
    % integration would credit this step's torque with the motion it
    % produced and overstate the cost of every acceleration.
    state.wheelEnergyJ = state.wheelEnergyJ ...
        + abs(dot(torque, in.omega(:).')) * cfg.stepS;

    out = struct( ...
        'torque', torque, ...
        'quaternionCmd', quaternionCmd, ...
        'omegaNext', omegaNext, ...
        'state', state, ...
        'boresightErrorRad', boresightErrorRad, ...
        'modeChanged', modeChanged, ...
        'wheelsSaturated', wheelsSaturated);
end
