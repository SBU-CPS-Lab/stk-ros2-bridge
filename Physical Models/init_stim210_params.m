%% STIM210 Gyroscope Parameters Initialization
% Sensor: Sensonor STIM210 (High-performance MEMS Gyro)
clear; clc;

%% Scale Factor(SF) parameters
% Accuracy: +/- 500 ppm (Datasheet p.7)
sf_error_ppm = 500;
stim210_sf = (sf_error_ppm * 1e-6) + 1;

%% Bias Instability parameters
% Instability: 0.3 deg/hour (Datasheet p.5)s
bias_instability = 0.3;
stim210_bias_instability = bias_instability / 3600; % convert deg/hour to deg/sec
