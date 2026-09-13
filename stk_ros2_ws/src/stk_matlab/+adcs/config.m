function cfg = config(scenarioPath)
%CONFIG Load the twin's parameters from acav_scenario.yaml.
%
%   cfg = adcs.config()                    % the shipped scenario
%   cfg = adcs.config('path/to/file.yaml')
%
% The YAML is THE contract (C1). Nothing in the twin may carry its own
% copy of a value the scheduler also uses: a quantity that can drift
% between them turns E2 into a measurement of their disagreement rather
% than of the effect under study.
%
% Values NOT in the YAML because they are the twin's own design -- the
% control gains and the profile authority -- are set here and are the
% quantities E4 sweeps. Their defaults were chosen by measurement, not
% taste: see acav/twin_reference.py::TwinConfig.

    if nargin < 1 || isempty(scenarioPath)
        here = fileparts(fileparts(mfilename('fullpath')));
        scenarioPath = fullfile(fileparts(here), 'acav', 'config', ...
            'acav_scenario.yaml');
    end
    if ~isfile(scenarioPath)
        error('adcs:config:NotFound', ...
            ['ACAV scenario spec not found at %s.\n' ...
             'This file is the contract every phase reads; pass its ' ...
             'path explicitly if the workspace is laid out ' ...
             'differently.'], scenarioPath);
    end

    raw = adcs.internal.read_scenario_yaml(scenarioPath);

    cfg = struct();
    cfg.scenarioPath = scenarioPath;
    cfg.inertia = [raw.ixx raw.iyy raw.izz];
    cfg.maxTorqueNm = raw.max_torque_Nm;
    cfg.momentumCapacityNms = raw.momentum_capacity_Nms;
    cfg.boresightBody = raw.boresight_body_axis;
    cfg.settleEntryRad = deg2rad(raw.settle_entry_deg);
    cfg.captureErrorRad = deg2rad(raw.capture_error_deg);
    cfg.settleHoldS = raw.settle_hold_s;
    cfg.maxBodyRateRadS = deg2rad(raw.max_body_rate_deg_s);
    cfg.stepS = raw.step_s;
    cfg.windowDurationS = raw.window_duration_s;
    cfg.settleEstimateS = raw.settle_estimate_s;

    % -- twin design parameters (E4 sweeps these) ---------------------
    cfg.naturalFrequencyRadS = 1.0;
    cfg.dampingRatio = 0.707;
    cfg.profileAuthority = 0.8;

    % -- derived ------------------------------------------------------
    wn = cfg.naturalFrequencyRadS;
    cfg.kp = cfg.inertia * wn^2;
    cfg.kd = 2 * cfg.dampingRatio * wn * cfg.inertia;

    % Actuator limits about the x axis: the largest inertia, and the
    % axis every cross-track slew in this study turns about. Same call
    % the scheduler makes.
    cfg.alphaRadS2 = cfg.maxTorqueNm / cfg.inertia(1);
    cfg.omegaMaxRadS = cfg.momentumCapacityNms / cfg.inertia(1);
    % Derated limits for the reference profile -- see slew_profile.m.
    cfg.profileAlphaRadS2 = cfg.alphaRadS2 * cfg.profileAuthority;
    cfg.profileOmegaMaxRadS = cfg.omegaMaxRadS * cfg.profileAuthority;
end
