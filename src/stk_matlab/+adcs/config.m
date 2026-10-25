function cfg = config(scenarioPath, runConfigPath)
%CONFIG Load the twin's parameters from acav_scenario.yaml.
%
%   cfg = adcs.config()                    % the shipped scenario
%   cfg = adcs.config('path/to/file.yaml')
%   cfg = adcs.config([], 'run.json')      % with per-run overrides
%
% The second argument is how a campaign varies the twin between runs:
% controller gains and an injected wheel fault are not properties of
% the scenario, so they arrive as a small JSON written per run by
% acav/campaign_runner.py:
%
%   {"gains": {"natural_frequency_rad_s": 1.2,
%              "damping_ratio": 0.707,
%              "profile_authority": 0.8},
%    "fault": {"axis": 0, "at_s": 123.456, "health": 0.0,
%              "kind": "saturation", "seed": 5}}
%
% Both keys are optional. The field names are the run record's, not
% this struct's, so that names are translated once, at this boundary.
%
% The YAML is the single source of scenario values. The twin keeps no
% copy of a value the scheduler also uses, so the two cannot drift
% apart.
%
% The twin's own design values (control gains and profile authority)
% are not in the YAML; their defaults are set here and documented in
% acav/twin_reference.py::TwinConfig.

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

    % -- twin design parameters ---------------------------------------
    cfg.naturalFrequencyRadS = 1.0;
    cfg.dampingRatio = 0.707;
    cfg.profileAuthority = 0.8;

    % -- per-run overrides (gains, wheel fault) -----------------------
    cfg.wheelFault = [];
    cfg.attitudeMismatchPolicy = 'strict';
    % Which run config was loaded: sha256 of the exact bytes parsed, and
    % its run_id. Empty for the nominal vehicle (no run config).
    cfg.runConfigSha256 = '';
    cfg.runId = '';
    % Earth's rotation rate, rad/s: the WGS 84 defining parameter omega
    % (NIMA TR8350.2, 3rd ed., 2000, Table 3.1). The one place the twin
    % spells it; adcs.geometry_at reads it from here. It rotates the
    % targets about the inertial z axis, which ignores precession and
    % nutation (the pole is a fraction of a degree off z): over the
    % seconds a free-run sub-step propagates, that is metres at most.
    cfg.earthRateRadS = 7.292115e-5;
    % That rotation is right only for targets fixed to the Earth. The
    % scenario places them on the ground (as STK targets), and anything
    % else must fail here rather than be propagated as if it were.
    if ~isfield(raw, 'central_body') || ~strcmp(raw.central_body, 'Earth') || ...
            ~isfield(raw, 'target_placement') || ...
            ~strcmp(raw.target_placement, 'ground_track_relative')
        error('adcs:config:TargetsNotEarthFixed', ...
            ['%s: the twin propagates targets with the Earth''s rotation, ' ...
             'which needs scenario.central_body "Earth" and ' ...
             'targets.placement "ground_track_relative" (Earth-fixed ' ...
             'ground targets).'], scenarioPath);
    end

    % 'lockstep' (one step per sample, the default) or 'free_run' (ZOH
    % sub-stepping, adcs.simulink_step), from the run config.
    cfg.executionMode = 'lockstep';
    if nargin >= 2 && ~isempty(runConfigPath)
        cfg = applyRunConfig(cfg, runConfigPath);
    end

    % -- derived ------------------------------------------------------
    wn = cfg.naturalFrequencyRadS;
    cfg.kp = cfg.inertia * wn^2;
    cfg.kd = 2 * cfg.dampingRatio * wn * cfg.inertia;

    % Actuator limits about the x axis: the largest inertia, and the
    % axis every cross-track slew turns about. Same call the scheduler
    % makes.
    cfg.alphaRadS2 = cfg.maxTorqueNm / cfg.inertia(1);
    cfg.omegaMaxRadS = cfg.momentumCapacityNms / cfg.inertia(1);
    % Derated limits for the reference profile -- see slew_profile.m.
    cfg.profileAlphaRadS2 = cfg.alphaRadS2 * cfg.profileAuthority;
    cfg.profileOmegaMaxRadS = cfg.omegaMaxRadS * cfg.profileAuthority;
end

function cfg = applyRunConfig(cfg, runConfigPath)
%APPLYRUNCONFIG Merge one run's gain and fault overrides into cfg.
%
% A missing file is an error, not a silent default: a run that asked
% for overrides and silently got the nominal values would record
% settings the twin never used.
    if ~isfile(runConfigPath)
        error('adcs:config:NoRunConfig', ...
            ['run config not found at %s. This file carries the ' ...
             'per-run gain and fault overrides; a run that silently ' ...
             'fell back to the nominal values would be recorded as ' ...
             'the treatment it was asked to be.'], runConfigPath);
    end

    % Read once, hash and parse the SAME bytes: the digest is how the run
    % orchestrator checks which config the twin actually loaded.
    fid = fopen(runConfigPath, 'r');
    bytes = fread(fid, inf, '*uint8').';
    fclose(fid);
    cfg.runConfigSha256 = adcs.internal.sha256_hex(bytes);
    raw = jsondecode(native2unicode(bytes, 'UTF-8'));
    if isfield(raw, 'run_id')
        cfg.runId = char(raw.run_id);
    end

    if isfield(raw, 'attitude_mismatch_policy')
        policy = char(raw.attitude_mismatch_policy);
        if ~strcmp(policy, 'strict')
            if ~strcmp(policy, 'e1_readback_ablation') || ...
                    ~isfield(raw, 'experiment') || ~strcmp(raw.experiment, 'E1') || ...
                    ~isfield(raw, 'attitude_provider') || isempty(raw.attitude_provider)
                error('adcs:config:MismatchPolicy', ...
                    'Read-back ablation is allowed only for an explicit E1 forced-provider treatment.');
            end
        end
        cfg.attitudeMismatchPolicy = policy;
    end

    if isfield(raw, 'execution_mode')
        mode = char(raw.execution_mode);
        if ~any(strcmp(mode, {'lockstep', 'free_run'}))
            error('adcs:config:ExecutionMode', ...
                'execution_mode must be lockstep or free_run, not %s.', mode);
        end
        if strcmp(mode, 'free_run') && ...
                strcmp(cfg.attitudeMismatchPolicy, 'e1_readback_ablation')
            error('adcs:config:ExecutionMode', ...
                'The E1 read-back ablation is defined for lockstep only.');
        end
        cfg.executionMode = mode;
    end

    if isfield(raw, 'gains') && ~isempty(raw.gains)
        gains = raw.gains;
        if isfield(gains, 'natural_frequency_rad_s')
            cfg.naturalFrequencyRadS = gains.natural_frequency_rad_s;
        end
        if isfield(gains, 'damping_ratio')
            cfg.dampingRatio = gains.damping_ratio;
        end
        if isfield(gains, 'profile_authority')
            cfg.profileAuthority = gains.profile_authority;
        end
    end

    if isfield(raw, 'fault') && ~isempty(raw.fault)
        fault = raw.fault;
        cfg.wheelFault = struct( ...
            'axis', fault.axis, ...          % zero-based; see wheel_fault.m
            'atS', fault.at_s, ...
            'health', 0, ...
            'kind', 'saturation');
        if isfield(fault, 'health')
            cfg.wheelFault.health = fault.health;
        end
        if isfield(fault, 'kind')
            cfg.wheelFault.kind = char(fault.kind);
        end
    end
end
