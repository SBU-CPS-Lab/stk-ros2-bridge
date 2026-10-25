function [health, saturates] = wheel_fault(fault, t)
%WHEEL_FAULT The injected wheel fault's effect at time t. E5's treatment.
%
%   [health, saturates] = adcs.wheel_fault(fault, t)
%
% fault  [] for a healthy vehicle (every experiment except E5), or a
%        struct with:
%          .axis    0, 1 or 2 -- ZERO-BASED, because it comes straight
%                   from the run record that acav/fault_injection.py
%                   wrote. Converted to a MATLAB index here and nowhere
%                   else; doing it at each use site is how one call site
%                   ends up faulting the wrong wheel while every other
%                   one is right.
%          .atS     seconds from the window start
%          .health  derate applied to a FAILURE, 0 = dead
%          .kind    'saturation' | 'failure'
% t      seconds from the window start
%
% health     [1x3] per-axis derate for adcs.wheels
% saturates  true once a SATURATION fault's epoch has arrived; the
%            CALLER applies it once and remembers that it did, so this
%            stays a pure function of (fault, t) and a replay makes the
%            same decision at the same instant.
%
% The two kinds do different things, and which is which was measured
% rather than reasoned about:
%
%   'saturation'  fills the wheel to capacity. The chart's momentum
%                 guard trips SAFE. This is what C1 section 9 specifies
%                 for E5.
%   'failure'     derates the wheel so it delivers no torque. Pointing
%                 collapses -- 23 deg to 178 deg over a minute in the
%                 reference -- and NO guard fires, because the error is
%                 about the dead axis and the controller asks the other
%                 two wheels for almost nothing. A real gap in the
%                 vehicle's fault detection, worth reporting as such,
%                 but not what E5 was asked to inject.
%
% Transcribed from acav/fault_injection.py::WheelFault; agreement is
% asserted by test_acav_twin.m against generated fixtures.

    health = [1 1 1];
    saturates = false;
    if isempty(fault)
        return
    end

    kind = 'saturation';
    if isfield(fault, 'kind') && ~isempty(fault.kind)
        kind = char(fault.kind);
    end

    % Inclusive at the epoch, so a fault landing exactly on a step
    % boundary lands ON that step rather than depending on a
    % floating-point comparison -- the kind of one-step ambiguity that
    % makes two supposedly identical replays diverge.
    if t < fault.atS
        return
    end

    axisIndex = fault.axis + 1;     % zero-based in the record
    switch kind
        case 'failure'
            health(axisIndex) = fault.health;
        case 'saturation'
            saturates = true;
        otherwise
            error('adcs:wheel_fault:UnknownKind', ...
                ['unknown fault kind %s; expected ''saturation'' or ' ...
                 '''failure''. A kind this function does not recognise ' ...
                 'would otherwise inject nothing, and the run would be ' ...
                 'reported as an E5 treatment that behaved exactly ' ...
                 'like the control.'], kind);
    end
end
