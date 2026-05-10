%% Sensor Gyroscope STIM210 (High-performance MEMS Gyro) Parameters Initialization
clear; clc;

%% Scale Factor(SF) parameters
% Accuracy: +/- 500 ppm (Datasheet p.7)
sf_error_ppm = 500;
stim210_sf = (sf_error_ppm * 1e-6) + 1;

%% Bias Instability parameters
% Instability: 0.3 deg/hour (Datasheet p.7)
bias_instability = 0.3;
stim210_bias_instability = bias_instability / 3600; % convert deg/hour to deg/sec

%% Noise parameters(ARW)
% ARW: 0.15 deg/sqrt(hour)(Datasheet p.7)
stim210_arw_deg_sqrt_hr = 0.15;
stim210_arw_deg_sqrt_sec = stim210_arw_deg_sqrt_hr / 60;
stim210_noise_power = stim210_arw_deg_sqrt_sec ^ 2;

%% Timing & Sampling
% Sampling frequency (Datasheet p.68)
stim210_fs = 2000; % HZ
% F = 1 / T
stim210_dt = 1 / stim210_fs;

%% Dynamic parameters
% bandwidth: 262 HZ (Datasheet p.7)
stim210_bw_hz = 262;
stim210_tau = 1 / (2 * pi * stim210_bw_hz); % Time constant [s]

%% Physical constraints(Saturation)
% measurement range: ±400 deg/s (Datasheet p.5)
stim210_range = 400;
