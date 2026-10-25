function busInfo = stk_bus_definitions()
%STK_BUS_DEFINITIONS Create Simulink.Bus objects matching stk_msgs, for
% use with the ROS 2 Subscribe/Publish blocks' "Output bus" option.
%
% Simulink's ROS 2 blocks can emit/consume either the raw ROS 2 struct or
% a Simulink.Bus with matching field layout; a Bus is generally easier
% to route through Simulink signal lines (Bus Selector, Scope, etc). This
% function builds the buses this project actually uses, base workspace,
% so build_twin_template.m can reference them.
%
% Field order and names here MUST match the corresponding .msg file's
% field order exactly, or Simulink's automatic bus-to-struct mapping
% silently misassigns fields -- there is no runtime name check for a
% struct built field-by-field like this. Cross-check against
% src/stk_msgs/msg/*.msg if you extend this.

    busInfo = struct();

    % -- OrbitState ---------------------------------------------------
    elems = Simulink.BusElement.empty(0, 1);
    elems(end+1) = makeElem('header_stamp_sec', 1, 'int32');
    elems(end+1) = makeElem('header_stamp_nanosec', 1, 'uint32');
    elems(end+1) = makeElem('header_frame_id', 1, 'string');
    elems(end+1) = makeElem('object_path', 1, 'string');
    elems(end+1) = makeElem('position', 3, 'double');
    elems(end+1) = makeElem('velocity', 3, 'double');
    elems(end+1) = makeElem('is_extrapolated', 1, 'boolean');
    busInfo.OrbitState = makeBus('STK_OrbitState', elems);

    % -- AttitudeState --------------------------------------------------
    elems = Simulink.BusElement.empty(0, 1);
    elems(end+1) = makeElem('header_stamp_sec', 1, 'int32');
    elems(end+1) = makeElem('header_stamp_nanosec', 1, 'uint32');
    elems(end+1) = makeElem('header_frame_id', 1, 'string');
    elems(end+1) = makeElem('object_path', 1, 'string');
    % Order is (x,y,z,w), matching geometry_msgs/Quaternion AND STK's
    % Q1..Q4 (see core/conversions.py's quaternion contract). Do not
    % reorder to (w,x,y,z): that silently swaps the scalar and vector
    % parts.
    elems(end+1) = makeElem('quaternion_xyzw', 4, 'double');
    elems(end+1) = makeElem('is_extrapolated', 1, 'boolean');
    busInfo.AttitudeState = makeBus('STK_AttitudeState', elems);

    % -- AngularRate ------------------------------------------------------
    elems = Simulink.BusElement.empty(0, 1);
    elems(end+1) = makeElem('header_stamp_sec', 1, 'int32');
    elems(end+1) = makeElem('header_stamp_nanosec', 1, 'uint32');
    elems(end+1) = makeElem('object_path', 1, 'string');
    elems(end+1) = makeElem('angular_velocity_radps', 3, 'double');
    busInfo.AngularRate = makeBus('STK_AngularRate', elems);

    % -- AttitudeCommand (Simulink -> bridge) ----------------------------
    elems = Simulink.BusElement.empty(0, 1);
    elems(end+1) = makeElem('header_stamp_sec', 1, 'int32');
    elems(end+1) = makeElem('header_stamp_nanosec', 1, 'uint32');
    elems(end+1) = makeElem('header_frame_id', 1, 'string');
    elems(end+1) = makeElem('object_path', 1, 'string');
    elems(end+1) = makeElem('quaternion_xyzw', 4, 'double');
    elems(end+1) = makeElem('use_inertial_frame', 1, 'boolean');
    busInfo.AttitudeCommand = makeBus('STK_AttitudeCommand', elems);

    % -- BridgeStatus -----------------------------------------------------
    elems = Simulink.BusElement.empty(0, 1);
    elems(end+1) = makeElem('state', 1, 'uint8');
    elems(end+1) = makeElem('backend_type', 1, 'string');
    elems(end+1) = makeElem('dropped_attitude_samples', 1, 'uint32');
    elems(end+1) = makeElem('dropped_position_samples', 1, 'uint32');
    elems(end+1) = makeElem('command_queue_depth', 1, 'uint32');
    elems(end+1) = makeElem('command_queue_capacity', 1, 'uint32');
    elems(end+1) = makeElem('last_backend_round_trip_ms', 1, 'double');
    busInfo.BridgeStatus = makeBus('STK_BridgeStatus', elems);

    fields = fieldnames(busInfo);
    for i = 1:numel(fields)
        assignin('base', busInfo.(fields{i}).Name, busInfo.(fields{i}));
    end
    fprintf('stk_bus_definitions: created %d Simulink.Bus objects in the base workspace\n', ...
        numel(fields));
end

function elem = makeElem(name, dims, dataType)
    elem = Simulink.BusElement;
    elem.Name = name;
    elem.Dimensions = dims;
    elem.DataType = dataType;
    elem.Complexity = 'real';
    elem.SampleTime = -1;
end

function bus = makeBus(name, elements)
    bus = Simulink.Bus;
    bus.Name = name;
    bus.Elements = elements;
end
