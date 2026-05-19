% Add current directory and all sub-directories to the main path
addpath(genpath(pwd));

% Add all the params of sub-models to top-level model workspace
run(fullfile(pwd, 'Sensors\init_gyroscope_stim210_params.m'));
run(fullfile(pwd, 'Sensors\init_magnetometer_hmr2300_params.m'));
