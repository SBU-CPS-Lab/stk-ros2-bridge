function run_twin_session(runConfigPath, stopFile)
%RUN_TWIN_SESSION Build the twin for one run, run it until told to stop.
%
%   matlab -batch "run_twin_session('<run config>', '<stop file>')"
%
% Started by the run orchestrator (deploy_run_cli --twin-launch
% matlab-batch, acav/twin_launcher.py), one fresh MATLAB process per run,
% from this folder. It builds the model for runConfigPath
% (build_acav_twin), starts the simulation, and polls every 0.5 s until
% stopFile exists; the simulation stopping by itself is an error. Whatever
% happens -- a build error, a simulation error, the stop file -- it then
% stops the simulation and closes the model, and writes
% matlab_session.json beside the stop file:
%
%   matlab_version, release, pid, run_config, started_at, built_at,
%   running_at, stop_requested_at, stopped_at, exit ("clean_stop" or
%   "error"), error
%
% (times local, yyyy-mm-ddTHH:MM:SS.FFF). It exits non-zero unless the
% stop was clean. The runner keeps <run>.logs\matlab.log (-logfile) in
% every case. Simulink reports a run-time error to its Diagnostic Viewer,
% not to this script, so the viewer's messages are kept in
% simulink_diagnostics.txt beside the stop file, copied into
% matlab_session.json (simulink_diagnostics) and printed to matlab.log
% when the simulation stops by itself. The ack wait, the twin identity check and the free-run
% start gate stay with the runner.
    here = fileparts(mfilename('fullpath'));
    sessionPath = fullfile(fileparts(stopFile), 'matlab_session.json');
    session = struct('matlab_version', version, ...
        'release', version('-release'), 'pid', feature('getpid'), ...
        'run_config', runConfigPath, 'started_at', stamp(), ...
        'built_at', '', 'running_at', '', 'stop_requested_at', '', ...
        'stopped_at', '', 'exit', '', 'error', '', ...
        'simulink_diagnostics', '');
    writeSession(sessionPath, session);
    diagnosticsPath = fullfile(fileparts(stopFile), 'simulink_diagnostics.txt');
    diaryOn = false;
    model = '';
    try
        cd(here);
        setup_stk_ros;
        model = char(build_acav_twin('RunConfigPath', runConfigPath));
        load_system(model);
        session.built_at = stamp();
        writeSession(sessionPath, session);
        clear adcs.simulink_step
        if isfile(diagnosticsPath), delete(diagnosticsPath); end
        sldiagviewer.diary(diagnosticsPath, 'UTF-8');
        diaryOn = true;
        set_param(model, 'SimulationCommand', 'start');
        waitFor(@() strcmp(get_param(model, 'SimulationStatus'), 'running'), 120);
        session.running_at = stamp();
        writeSession(sessionPath, session);
        while ~isfile(stopFile)
            status = get_param(model, 'SimulationStatus');
            if ~strcmp(status, 'running')
                [session.simulink_diagnostics, diaryOn] = ...
                    readDiagnostics(diagnosticsPath, diaryOn);
                error('run_twin_session:Stopped', ...
                    'the simulation stopped by itself (status %s): %s', ...
                    status, session.simulink_diagnostics);
            end
            pause(0.5);
        end
        session.stop_requested_at = stamp();
        session.exit = 'clean_stop';
    catch err
        session.exit = 'error';
        session.error = getReport(err, 'extended', 'hyperlinks', 'off');
        if isempty(session.simulink_diagnostics)
            [session.simulink_diagnostics, diaryOn] = ...
                readDiagnostics(diagnosticsPath, diaryOn);
        end
        if ~isempty(session.simulink_diagnostics)
            fprintf('run_twin_session: Simulink diagnostics:\n%s\n', ...
                session.simulink_diagnostics);
        end
    end
    if diaryOn
        sldiagviewer.diary('off');
    end
    % Always: stop the simulation and close the model.
    try
        if ~isempty(model) && bdIsLoaded(model)
            if ~strcmp(get_param(model, 'SimulationStatus'), 'stopped')
                set_param(model, 'SimulationCommand', 'stop');
                waitFor(@() strcmp(get_param(model, 'SimulationStatus'), 'stopped'), 60);
            end
            close_system(model, 0);
        end
    catch err
        session.exit = 'error';
        session.error = strtrim([session.error newline ...
            getReport(err, 'extended', 'hyperlinks', 'off')]);
    end
    session.stopped_at = stamp();
    writeSession(sessionPath, session);
    if ~strcmp(session.exit, 'clean_stop')
        error('run_twin_session:NotClean', '%s', session.error);
    end
end

function [text, diaryOn] = readDiagnostics(path, diaryOn)
%READDIAGNOSTICS The Diagnostic Viewer messages logged so far ('' if none).
    if diaryOn
        sldiagviewer.diary('off');          % flushes the file
        diaryOn = false;
    end
    text = '';
    if isfile(path)
        text = strtrim(fileread(path));
    end
end

function text = stamp()
    text = datestr(now, 'yyyy-mm-ddTHH:MM:SS.FFF');
end

function waitFor(condition, timeoutS)
    t0 = tic;
    while ~condition()
        if toc(t0) > timeoutS
            error('run_twin_session:Timeout', 'not reached within %g s', timeoutS);
        end
        pause(0.2);
    end
end

function writeSession(path, session)
    fid = fopen(path, 'w');
    if fid < 0
        return
    end
    fwrite(fid, uint8([jsonencode(session, 'PrettyPrint', true) 10]), 'uint8');
    fclose(fid);
end
