function test_twin_model_runs()
%TEST_TWIN_MODEL_RUNS Build the Simulink twin and run it for 1 s.
%
% A diagram update checks the wiring, not what happens when the model
% runs: d97800c passed it, then stopped at the first step of every live
% run ("Class mismatch for variable 'saturationEvents'. Expected
% 'uint32', Actual 'double'"). This runs the built model -- with no ROS
% traffic every step is "not ready", which still calls adcs.simulink_step
% and checks each output's class -- so such an error fails here.
%
% Needs the generated stk_msgs message types (setup_stk_ros) and takes
% about two minutes (the build).
    here = fileparts(mfilename('fullpath'));
    cd(here);
    setup_stk_ros;
    model = char(build_acav_twin());
    cleanup = onCleanup(@() close_system(model, 0));
    load_system(model);
    set_param(model, 'StopTime', '1');
    clear adcs.simulink_step
    try
        sim(model);
    catch err
        error('test_twin_model_runs:Stopped', ...
            'the twin model stopped while running: %s', err.message);
    end
    clear adcs.simulink_step
    fprintf('PASS: the built twin model runs.\n');
end
