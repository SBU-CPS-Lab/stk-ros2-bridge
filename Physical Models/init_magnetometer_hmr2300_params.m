% Magnetometer HMR2300 params
hmr2300_range = 2; % +/-2 gauss
hmr2300_sf = diag([1.01, 0.99, 1.02]); % scale-factor matrix
hmr2300_resolution = 6.1e-5; % gauss per LSB (?67 µG)
hmr2300_bias = [0.005, 0.002, -0.003];    % bias (gauss)
hmr2300_noise_std = 5e-5;    % noise sigma (?50 µG)
hmr2300_sample_rate = 154;   % samples per second
hmr2300_Ts = 1 / hmr2300_sample_rate;
hmr2300_saturation = [-2, 2]; % gauss limits
