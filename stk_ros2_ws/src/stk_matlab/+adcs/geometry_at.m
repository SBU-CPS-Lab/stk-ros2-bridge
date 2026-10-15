function [position, velocity, targetEci] = geometry_at(position, velocity, targetEci, dt, cfg)
%GEOMETRY_AT Move one sample's reference geometry forward by dt seconds.
%
%   [pos, vel, target] = adcs.geometry_at(pos, vel, target, dt, cfg)
%
% Position moves along the velocity (pos + vel*dt, velocity held);
% the target, Earth-fixed (adcs.config refuses anything else), turns
% with the Earth about the inertial z axis at cfg.earthRateRadS.
% First order: over a free-run sub-step's dt (well under a second
% typically, a few seconds at worst) the orbit's curvature moves the
% position by metres to ~100 m, a few hundredths of a degree of line
% of sight at the targets' ranges.
%
% dt = 0 returns the inputs unchanged, bit for bit, so a sample on the
% step grid (and every lockstep-cadence sample) reaches adcs.twin_step
% exactly as received. adcs.simulink_step uses this in free-run to give
% each sub-step the geometry at its own time.
    if dt == 0
        return
    end
    % As rows, then back to each input's shape: a column must never
    % broadcast against a row.
    position = reshape(position(:).' + velocity(:).' * dt, size(position));
    targetEci = reshape(targetEci(:).' + ...
        cross([0 0 cfg.earthRateRadS], targetEci(:).') * dt, size(targetEci));
end
