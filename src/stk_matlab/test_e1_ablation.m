function test_e1_ablation()
    path = [tempname '.json']; cleanup = onCleanup(@() delete(path));
    raw = struct('experiment','E1','attitude_provider','ECF Attitude Quaternions', ...
        'attitude_mismatch_policy','e1_readback_ablation');
    writeConfig(path,raw);
    pos = [6878137 0 0]; vel = [0 7500 0]; target = [6378137 0 0];
    adcs.simulink_step([0 0 0 1],[0 0 0],pos,vel,target,true,0,true,path);
    qWrong = [1 0 0 0];
    [~,q] = adcs.simulink_step(qWrong,[0 0 0],pos,vel,target,true,.1,false,path);
    assert(abs(dot(q,qWrong))>.99,'Ablation must actually consume wrong read-back.');
    raw.experiment = 'E6'; writeConfig(path,raw);
    rejected = false;
    try, adcs.config([],path); catch err, rejected = strcmp(err.identifier,'adcs:config:MismatchPolicy'); end
    assert(rejected,'Ablation leaked into nominal E6.');
    test_simulink_adapter(); % Strict mismatch/time-gap guards still apply.
    fprintf('PASS: explicit E1 ablation; nominal mismatch protection retained.\n');
end
function writeConfig(path,raw)
    fid=fopen(path,'w'); closeFile=onCleanup(@() fclose(fid));
    fprintf(fid,'%s',jsonencode(raw));
end
