%% STIM210 Gyroscope Parameters Initialization
% Sensor: Sensonor STIM210 (High-performance MEMS Gyro)

clear; clc;

%% Scale Factor(SF) parameters
% Accuracy: +/- 500 ppm (Datasheet p.7)
sf_error_ppm = 500;
stim210_sf = (sf_error_ppm * 1e-6) + 1;
