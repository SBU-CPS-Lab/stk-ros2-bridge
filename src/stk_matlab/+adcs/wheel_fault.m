function [health, saturates] = wheel_fault(fault, t)
%WHEEL_FAULT The injected wheel fault's effect at time t.
%
%   [health, saturates] = adcs.wheel_fault(fault, t)
%
% fault  [] for a healthy vehicle, or a
%        struct with:
%          .axis    0, 1 or 2 -- ZERO-BASED, because it comes straight
%                   from the run record that acav/fault_injection.py
%                   wrote. Converted to a MATLAB index here and nowhere
%                   else.
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
% The two kinds behave differently (as measured with the reference):
%
%   'saturation'  fills the wheel to capacity. The momentum guard trips
%                 SAFE.
%   'failure'     derates the wheel so it delivers no torque. Pointing
%                 degrades (23 deg to 178 deg over a minute in the
%                 reference) and no guard fires, because the error is
%                 about the dead axis and the controller asks the other
%                 two wheels for almost nothing: a gap in this vehicle's
%                 fault detection.
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
    % boundary lands on that step rather than depending on a
    % floating-point comparison, which could make two replays diverge.
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
