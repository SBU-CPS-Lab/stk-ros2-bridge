function [modeOut, enteredAt] = mission_mode(modeIn, enteredAtIn, nowS, ...
        boresightErrorRad, bodyRateRadS, momentumFraction, ...
        slewRequested, profileDone, cfg)
%MISSION_MODE The Stateflow chart's transition logic, as a function.
%
%   NADIR -> SLEW -> SETTLE -> IMAGE, with SAFE reachable from anywhere.
%
% Kept as a function as well as a chart so that (a) it is testable
% against the same fixtures as everything else, and (b) the chart and
% this agree by construction rather than by inspection.
%
% Ordering matters: the SAFE guards are evaluated first and
% unconditionally, so a wheel that saturates during IMAGE trips SAFE
% rather than being masked by an already-satisfied hold.
%
% SAFE latches. An ADCS that re-armed the instant the trip condition
% cleared would oscillate in and out of SAFE; recovery is an operator
% action.
%
% This function is what the twin executes (from adcs.twin_step); the
% Stateflow chart built by adcs.build_mission_chart specifies the same
% logic.
%
% Transcribed from acav/twin_reference.py::update_mode.

    modeOut = modeIn;
    enteredAt = enteredAtIn;

    if momentumFraction >= 1.0 || bodyRateRadS > cfg.maxBodyRateRadS
        if ~strcmp(modeIn, 'SAFE')
            modeOut = 'SAFE';
            enteredAt = nowS;
        end
        return
    end

    if strcmp(modeIn, 'SAFE')
        return
    end

    if slewRequested && (strcmp(modeIn, 'NADIR') || strcmp(modeIn, 'IMAGE') || strcmp(modeIn, 'SETTLE'))
        modeOut = 'SLEW';
        enteredAt = nowS;
        return
    end

    switch modeIn
        case 'SLEW'
            if profileDone
                modeOut = 'SETTLE';
                enteredAt = nowS;
            end
        case 'SETTLE'
            if boresightErrorRad <= cfg.settleEntryRad
                modeOut = 'IMAGE';
                enteredAt = nowS;
            end
    end
end
