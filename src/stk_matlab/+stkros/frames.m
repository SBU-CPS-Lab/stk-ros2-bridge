function stkName = frames(rosFrameId)
%FRAMES Map a stk_bridge ROS frame_id to its STK name.
%
%   stkName = stkros.frames('icrf')  -> 'ICRF'
%   stkName = stkros.frames('fixed') -> 'Fixed'
%
% Mirrors core/frames.py's BUILTIN_FRAMES table on the Python/bridge
% side. Kept as a small static map here rather than querying the bridge
% for it on every call, because frame names do not change at runtime and
% a synchronous service round-trip per lookup would be wasteful on a
% hot path. If you add a frame in config/frames.yaml on the bridge side,
% add the matching entry here.
%
% On STK 12.6 'J2000' is not a Cartesian Position group member (see
% docs/bridge/frames_and_units.md). The bridge aliases it to
% 'MeanOfEpoch', and this table does the same.

    persistent map
    if isempty(map)
        map = containers.Map();
        map('icrf') = 'ICRF';
        map('j2000') = 'MeanOfEpoch';   % alias -- see note above
        map('mean_of_epoch') = 'MeanOfEpoch';
        map('mean_of_date') = 'MeanOfDate';
        map('true_of_date') = 'TrueOfDate';
        map('true_of_date_rotating') = 'TrueOfDateRotating';
        map('fixed') = 'Fixed';
        map('inertial') = 'Inertial';
        map('lvlh') = 'LVLH';
        map('vvlh') = 'VVLH';
        map('body') = 'Body';
    end

    key = lower(rosFrameId);
    if ~isKey(map, key)
        error('stkros:frames:UnknownFrame', ...
            'unknown frame_id ''%s''; known frames: %s', ...
            rosFrameId, strjoin(keys(map), ', '));
    end
    stkName = map(key);
end
