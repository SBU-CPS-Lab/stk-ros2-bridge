function values = read_scenario_yaml(path)
%READ_SCENARIO_YAML Pull the twin's fields out of acav_scenario.yaml.
%
% Deliberately NOT a general YAML parser. MATLAB has no bundled one, and
% adding a dependency to the deploy machine for this is not worth it.
% What is worth it is failing loudly when a field is missing rather than
% quietly substituting a default -- a twin running on a default inertia
% would produce plausible, wrong results, which is the failure mode this
% whole project is organised against.
%
% Only scalar `key: value` lines and the one inline list the twin needs
% are understood. If the YAML grows structure this cannot follow, the
% right answer is to export a flat parameter file from the Python side
% rather than to make this cleverer.

    text = fileread(path);
    lines = splitlines(string(text));

    wanted = ["ixx", "iyy", "izz", "max_torque_Nm", ...
              "momentum_capacity_Nms", "settle_entry_deg", ...
              "capture_error_deg", "settle_hold_s", ...
              "max_body_rate_deg_s", "step_s", "window_duration_s", ...
              "settle_estimate_s"];

    values = struct();
    section = "";
    for line = lines.'
        stripped = strtrim(line);
        if stripped == "" || startsWith(stripped, "#")
            continue
        end
        parts = split(stripped, ":");
        if numel(parts) < 2
            continue
        end
        key = strtrim(parts(1));
        rest = strtrim(strjoin(parts(2:end), ":"));
        rest = strtrim(extractBefore(rest + "#", "#"));
        if ~startsWith(line, [" ", char(9)])
            section = key;   % a top-level key opens a section
        end

        % Two strings, read by section because "placement" occurs more
        % than once: they say whether the targets are Earth-fixed.
        if section == "scenario" && key == "central_body"
            values.central_body = char(rest);
        elseif section == "targets" && key == "placement"
            values.target_placement = char(rest);
        elseif key == "boresight_body_axis"
            values.boresight_body_axis = parse_list(rest);
        elseif ismember(key, wanted) && ~isfield(values, key)
            number = str2double(rest);
            if ~isnan(number)
                values.(key) = number;
            end
        end
    end

    missing = wanted(~isfield(values, wanted));
    if ~isempty(missing)
        error('adcs:config:MissingFields', ...
            ['%s does not define: %s.\nThe twin will not substitute ' ...
             'defaults for these -- a twin running on a default ' ...
             'inertia produces plausible, wrong results.'], ...
            path, strjoin(missing, ', '));
    end
    if ~isfield(values, 'boresight_body_axis')
        error('adcs:config:MissingFields', ...
            '%s does not define sensor.boresight_body_axis', path);
    end
end

function v = parse_list(text)
    text = erase(erase(strtrim(text), "["), "]");
    v = str2double(split(text, ",")).';
end
