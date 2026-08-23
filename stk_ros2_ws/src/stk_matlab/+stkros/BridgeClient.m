classdef BridgeClient < handle
    %BRIDGECLIENT Single entry point bundling telemetry, commands, and
    % services for one satellite -- the object most MATLAB/Simulink code
    % should actually use.
    %
    % Example:
    %   client = stkros.BridgeClient("sss_1");
    %   pause(1.0);   % let ROS 2 discovery settle before first read
    %   orbit = client.Telemetry.latestOrbit();
    %   client.Commands.sendAttitude(client.now(), [0 0 0 1]);
    %
    % See examples/ for complete worked patterns: reading telemetry
    % (example_01), a closed attitude loop (example_02), and driving the
    % bridge's lockstep barrier from Simulink (example_03).

    properties (SetAccess = private)
        Namespace string
        Node
        Telemetry     % stkros.TelemetrySubscriber
        Commands      % stkros.CommandPublisher
        Services      % stkros.ScenarioService
        Status        % latest stk_msgs/BridgeStatus struct, or []
    end

    properties (Access = private)
        SubStatus
    end

    methods
        function obj = BridgeClient(namespace, options)
            arguments
                namespace (1,1) string
                options.Prefix (1,1) string = "stk"
                options.ParticipantName (1,1) string = "matlab_twin"
            end
            obj.Namespace = namespace;
            obj.Node = ros2node("matlab_twin_" + namespace);

            obj.Telemetry = stkros.TelemetrySubscriber(namespace, ...
                "Prefix", options.Prefix, "Node", obj.Node);
            obj.Commands = stkros.CommandPublisher(namespace, ...
                "Prefix", options.Prefix, ...
                "ParticipantName", options.ParticipantName, "Node", obj.Node);
            obj.Services = stkros.ScenarioService( ...
                "Prefix", options.Prefix, "Node", obj.Node);

            obj.SubStatus = ros2subscriber(obj.Node, ...
                sprintf("/%s/status", options.Prefix), ...
                "stk_msgs/BridgeStatus", @(msg) obj.onStatus(msg), ...
                "Reliability", "reliable", "Durability", "transientlocal");
        end

        function t = now(obj) %#ok<MANU>
            %NOW Current wall-clock time as seconds -- a convenience for
            % scripts. In LOCKSTEP mode the actual command timestamp
            % that matters is the bridge's simulation time, obtained
            % from client.Telemetry's message headers or from
            % client.Status, NOT wall time; see docs/05_time_synchronization.md.
            t = posixtime(datetime('now', 'TimeZone', 'UTC'));
        end

        function tf = isConnected(obj)
            %ISCONNECTED True if the last BridgeStatus reported
            % state == CONNECTED (1) or DEGRADED (2) -- both mean STK is
            % reachable; DEGRADED means a non-critical channel is
            % unhealthy (see core/health.py). STARTING/DISCONNECTED/
            % ERROR all return false.
            tf = ~isempty(obj.Status) && any(obj.Status.state == [1, 2]);
        end

        function waitUntilConnected(obj, timeoutS)
            %WAITUNTILCONNECTED Block until isConnected() or timeout.
            arguments
                obj
                timeoutS (1,1) double = 30.0
            end
            deadline = tic;
            while ~obj.isConnected()
                if toc(deadline) > timeoutS
                    error('stkros:BridgeClient:Timeout', ...
                        ['bridge did not report CONNECTED within %.0fs. ' ...
                         'Check the bridge node is running and ' ...
                         'ROS_DOMAIN_ID matches.'], timeoutS);
                end
                pause(0.1);
            end
        end
    end

    methods (Access = private)
        function onStatus(obj, msg)
            obj.Status = struct( ...
                'state', double(msg.state), ...
                'backend_type', string(msg.backend_type), ...
                'stk_version', string(msg.stk_version_string), ...
                'dropped_attitude_samples', double(msg.dropped_attitude_samples), ...
                'dropped_position_samples', double(msg.dropped_position_samples), ...
                'command_queue_depth', double(msg.command_queue_depth), ...
                'command_queue_capacity', double(msg.command_queue_capacity), ...
                'last_round_trip_ms', double(msg.last_backend_round_trip_ms), ...
                'last_error', string(msg.last_error));
        end
    end
end
