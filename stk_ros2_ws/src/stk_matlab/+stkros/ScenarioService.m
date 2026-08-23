classdef ScenarioService < handle
    %SCENARIOSERVICE Thin wrappers over stk_bridge's service calls.
    %
    % Mirrors interfaces/ros_services.py. Every method here blocks until
    % the bridge's COM-apartment worker returns a result (see
    % core/com_thread.py) and raises a MATLAB error with the SAME
    % message text STK/the bridge produced, rather than MATLAB's generic
    % "service call timed out" -- see ros_services.py's "Failure
    % convention" docstring on why responses always carry success/message
    % instead of the handler raising.

    properties (SetAccess = private)
        Prefix string = "stk"
    end

    properties (Access = private)
        Node
        Clients containers.Map
    end

    methods
        function obj = ScenarioService(options)
            arguments
                options.Prefix (1,1) string = "stk"
                options.Node = []
            end
            obj.Prefix = options.Prefix;
            if isempty(options.Node)
                obj.Node = ros2node("matlab_twin_services");
            else
                obj.Node = options.Node;
            end
            obj.Clients = containers.Map();
        end

        function [ok, message, epoch] = loadScenario(obj, scenarioPath, options)
            arguments
                obj
                scenarioPath (1,1) string
                options.CloseExisting (1,1) logical = true
            end
            client = obj.client("load_scenario", "stk_msgs/LoadScenario");
            req = ros2message(client);
            req.scenario_path = char(scenarioPath);
            req.close_existing_first = options.CloseExisting;
            resp = call(client, req, 120);
            ok = logical(resp.success);
            message = string(resp.message);
            epoch = double(resp.scenario_epoch.sec) + ...
                    double(resp.scenario_epoch.nanosec) * 1e-9;
        end

        function [ok, message] = resetScenario(obj, options)
            arguments
                obj
                options.ClearRealtimeData (1,1) logical = true
            end
            client = obj.client("reset_scenario", "stk_msgs/ResetScenario");
            req = ros2message(client);
            req.clear_realtime_data = options.ClearRealtimeData;
            resp = call(client, req, 60);
            ok = logical(resp.success);
            message = string(resp.message);
        end

        function [ok, message, objectPath] = createObject(obj, ...
                objectType, name, options)
            arguments
                obj
                objectType (1,1) string
                name (1,1) string
                options.ParentPath (1,1) string = ""
            end
            client = obj.client("create_object", "stk_msgs/CreateObject");
            req = ros2message(client);
            req.object_type = char(objectType);
            req.object_name = char(name);
            req.parent_path = char(options.ParentPath);
            resp = call(client, req, 30);
            ok = logical(resp.success);
            message = string(resp.message);
            objectPath = string(resp.object_path);
        end

        function [ok, message] = setPropagator(obj, objectPath, ...
                propagatorType, options)
            %SETPROPAGATOR propagatorType: "TwoBody"|"J4Perturbation"|
            % "HPOP"|"RealTime"|"External" (case-sensitive strings match
            % stk_msgs/SetPropagator.srv's constant NAMES, not the
            % Connect-command spellings -- this method does the mapping).
            arguments
                obj
                objectPath (1,1) string
                propagatorType (1,1) string
                options.LookAheadS (1,1) double = 3600.0
                options.LookBehindS (1,1) double = 3600.0
                options.ExternalEphemerisPath (1,1) string = ""
            end
            typeMap = containers.Map( ...
                {"TwoBody", "J4Perturbation", "HPOP", "RealTime", "External"}, ...
                {uint8(0), uint8(1), uint8(2), uint8(3), uint8(4)});
            if ~isKey(typeMap, char(propagatorType))
                error('stkros:ScenarioService:BadPropagatorType', ...
                    'unknown propagator type ''%s''; valid: %s', ...
                    propagatorType, strjoin(keys(typeMap), ', '));
            end
            client = obj.client("set_propagator", "stk_msgs/SetPropagator");
            req = ros2message(client);
            req.object_path = char(objectPath);
            req.propagator_type = typeMap(char(propagatorType));
            req.realtime_look_ahead_s = options.LookAheadS;
            req.realtime_look_behind_s = options.LookBehindS;
            req.external_ephemeris_path = char(options.ExternalEphemerisPath);
            resp = call(client, req, 60);
            ok = logical(resp.success);
            message = string(resp.message);
        end

        function [ok, message, newSimTime, stepIndex] = stepSimulation( ...
                obj, options)
            %STEPSIMULATION Manual single-step, LOCKSTEP mode only. See
            % examples/example_03_lockstep_stepping.m for the pattern of
            % pairing this with CommandPublisher.acknowledgeStep --
            % though with time.auto_step: true (bridge_params.yaml's
            % default) the bridge steps on its own and this service is
            % only needed for manual/debug stepping.
            arguments
                obj
                options.StepS (1,1) double = 0.0
            end
            client = obj.client("step_simulation", "stk_msgs/StepSimulation");
            req = ros2message(client);
            req.step_s = options.StepS;
            resp = call(client, req, 30);
            ok = logical(resp.success);
            message = string(resp.message);
            newSimTime = double(resp.new_sim_time.sec) + ...
                        double(resp.new_sim_time.nanosec) * 1e-9;
            stepIndex = double(resp.step_index);
        end

        function [ok, rawResponse, wasNack] = executeConnect(obj, command, ...
                options)
            %EXECUTECONNECT Raw Connect passthrough -- the escape hatch.
            % See stk_msgs/ExecuteConnect.srv and
            % interfaces/ros_services.py's warning-level log on every
            % call: this bypasses frame/unit/monotonicity validation.
            % Prefer a typed method above wherever one exists.
            arguments
                obj
                command (1,1) string
                options.TimeoutS (1,1) double = 60.0
            end
            client = obj.client("execute_connect", "stk_msgs/ExecuteConnect");
            req = ros2message(client);
            req.command = char(command);
            req.timeout_s = options.TimeoutS;
            resp = call(client, req, options.TimeoutS + 5);
            ok = logical(resp.success);
            rawResponse = string(resp.raw_response);
            wasNack = logical(resp.was_nack);
        end
    end

    methods (Access = private)
        function client = client(obj, name, type)
            key = char(name);
            if isKey(obj.Clients, key)
                client = obj.Clients(key);
                return
            end
            topic = sprintf("/%s/%s", obj.Prefix, name);
            client = ros2svcclient(obj.Node, topic, type);
            obj.Clients(key) = client;
        end
    end
end
