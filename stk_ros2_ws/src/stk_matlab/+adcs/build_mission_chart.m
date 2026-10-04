function chart = build_mission_chart(model, blockName, position)
%BUILD_MISSION_CHART The ACAV mission-mode chart, programmatically.
%
%   chart = adcs.build_mission_chart(modelName, 'MissionMode', pos)
%
%   NADIR -> SLEW -> SETTLE -> IMAGE, with SAFE reachable from anywhere
%   and latched once entered.
%
% Separate from build_acav_twin so it can be built and inspected without
% the ROS 2 blocks, which need custom messages generated first
% (setup_stk_ros). The chart is the part of the model with real logic in
% it; the rest is wiring.
%
% Guards mirror adcs/mission_mode.m, which is the same logic as a
% function and is what the fixtures actually exercise. They must agree,
% and a change belongs in both.
%
% Two properties are load-bearing:
%
%   * the SAFE transitions have execution order 1, so a wheel that
%     saturates during IMAGE trips SAFE rather than being masked by an
%     already-satisfied hold. Written the other way round the chart is
%     correct in every nominal run and silently fails to protect the
%     vehicle in exactly the campaign (E5) that exists to test it;
%   * SAFE has no outgoing transition. An ADCS that re-armed itself the
%     instant the trip cleared would oscillate in and out and make E5's
%     deterministic replay meaningless.

    arguments
        model (1,:) char
        blockName (1,:) char = 'MissionMode'
        position (1,4) double = [270 220 470 340]
    end

    chartPath = [model '/' blockName];
    add_block('sflib/Chart', chartPath, 'Position', position);

    root = sfroot();
    chart = root.find('-isa', 'Stateflow.Chart', '-and', 'Path', chartPath);
    if isempty(chart)
        error('adcs:build_mission_chart:NotFound', ...
            'the chart block was added but could not be found at %s', ...
            chartPath);
    end

    % Inputs the chart reads. Names match adcs.mission_mode's arguments
    % so the two can be compared by eye as well as by test.
    inputs = {'slewRequested', 'profileDone', 'boresightError', ...
              'bodyRate', 'momentumFraction'};
    for k = 1:numel(inputs)
        d = Stateflow.Data(chart);
        d.Name = inputs{k};
        d.Scope = 'Input';
    end
    for name = {'settleEntry', 'maxBodyRate'}
        d = Stateflow.Data(chart);
        d.Name = name{1};
        d.Scope = 'Constant';
    end
    modeOut = Stateflow.Data(chart);
    modeOut.Name = 'mode';
    modeOut.Scope = 'Output';

    names = {'NADIR', 'SLEW', 'SETTLE', 'IMAGE', 'SAFE'};
    states = struct();
    for k = 1:numel(names)
        s = Stateflow.State(chart);
        s.Name = names{k};
        s.LabelString = sprintf('%s\nen: mode = %d;', names{k}, k);
        s.Position = [60 + 200 * mod(k - 1, 3), ...
                      60 + 160 * floor((k - 1) / 3), 140, 70];
        states.(names{k}) = s;
    end

    % Default transition into NADIR.
    entry = Stateflow.Transition(chart);
    entry.Destination = states.NADIR;
    entry.DestinationOClock = 0;
    entry.SourceEndpoint = [states.NADIR.Position(1) + 30, ...
                            states.NADIR.Position(2) - 40];

    addTransition(chart, states.NADIR,  states.SLEW,   '[slewRequested]');
    addTransition(chart, states.IMAGE,  states.SLEW,   '[slewRequested]');
    addTransition(chart, states.SLEW,   states.SETTLE, '[profileDone]');
    addTransition(chart, states.SETTLE, states.SLEW, '[slewRequested]');
    addTransition(chart, states.SETTLE, states.IMAGE, ...
        '[boresightError <= settleEntry]');

    % Priority 1: evaluated before anything else leaving the state.
    guard = '[momentumFraction >= 1 || bodyRate > maxBodyRate]';
    for k = 1:4
        t = addTransition(chart, states.(names{k}), states.SAFE, guard);
        t.ExecutionOrder = 1;
    end
end

function t = addTransition(chart, from, to, label)
    t = Stateflow.Transition(chart);
    t.Source = from;
    t.Destination = to;
    t.LabelString = label;
end
