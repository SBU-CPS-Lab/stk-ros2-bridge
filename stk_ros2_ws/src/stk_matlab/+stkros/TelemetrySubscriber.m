classdef TelemetrySubscriber < handle
    %TELEMETRYSUBSCRIBER Subscribes to a satellite's stk_bridge topics
    % and exposes the latest sample of each as plain MATLAB structs.
    %
    % Mirrors interfaces/ros_publishers.py::SatellitePublishers on the
    % bridge side, one-to-one: same topic names, same "absence means no
    % data yet" contract. See that module's docstring for why a missing
    % channel is represented as [] rather than a stale/frozen value --
    % this class preserves that contract; it does NOT synthesize a
    % last-known-good fallback.
    %
    % Example:
    %   sub = stkros.TelemetrySubscriber("sss_1");
    %   pause(0.5);   % let ROS 2 discovery settle
    %   orbit = sub.latestOrbit();
    %   if ~isempty(orbit)
    %       fprintf('position (m): [%g %g %g]\n', orbit.position);
    %   end

    properties (SetAccess = private)
        Namespace string
        Prefix string = "stk"
    end

    properties (Access = private)
        Node
        SubOrbit
        SubAttitude
        SubRate
        SubLLA
        SubEclipse
        SubState
        LatestOrbit = []
        LatestAttitude = []
        LatestRate = []
        LatestLLA = []
        LatestEclipse = []
        LatestState = []
    end

    methods
        function obj = TelemetrySubscriber(namespace, options)
            arguments
                namespace (1,1) string
                options.Prefix (1,1) string = "stk"
                options.Node = []
            end
            obj.Namespace = namespace;
            obj.Prefix = options.Prefix;

            if isempty(options.Node)
                obj.Node = ros2node("matlab_twin_telemetry");
            else
                obj.Node = options.Node;
            end

            base = sprintf("/%s/%s", obj.Prefix, obj.Namespace);

            % QoS matches interfaces/qos.py's telemetry_qos(): best
            % effort, depth 1. Subscribing with mismatched QoS
            % (MATLAB defaults to reliable) silently drops every
            % message on some DDS implementations rather than erroring
            % -- an easy, quiet failure mode, hence the explicit profile.
            qos = struct('Reliability', 'besteffort', ...
                         'Durability', 'volatile', 'Depth', 1);

            obj.SubOrbit = ros2subscriber(obj.Node, base + "/orbit", ...
                "stk_msgs/OrbitState", @(msg) obj.onOrbit(msg), ...
                "Reliability", qos.Reliability, "Depth", qos.Depth);
            obj.SubAttitude = ros2subscriber(obj.Node, base + "/attitude", ...
                "stk_msgs/AttitudeState", @(msg) obj.onAttitude(msg), ...
                "Reliability", qos.Reliability, "Depth", qos.Depth);
            obj.SubRate = ros2subscriber(obj.Node, base + "/angular_rate", ...
                "stk_msgs/AngularRate", @(msg) obj.onRate(msg), ...
                "Reliability", qos.Reliability, "Depth", qos.Depth);
            obj.SubLLA = ros2subscriber(obj.Node, base + "/lla", ...
                "stk_msgs/LLAState", @(msg) obj.onLLA(msg), ...
                "Reliability", qos.Reliability, "Depth", qos.Depth);
            obj.SubEclipse = ros2subscriber(obj.Node, base + "/eclipse", ...
                "stk_msgs/EclipseState", @(msg) obj.onEclipse(msg), ...
                "Reliability", qos.Reliability, "Depth", qos.Depth);
            obj.SubState = ros2subscriber(obj.Node, base + "/state", ...
                "stk_msgs/SatelliteState", @(msg) obj.onState(msg), ...
                "Reliability", qos.Reliability, "Depth", qos.Depth);
        end

        function s = latestOrbit(obj)
            %LATESTORBIT Position (m) and velocity (m/s) as a struct, or
            % [] if no orbit sample has arrived yet. See this class's
            % header comment on why [] is returned rather than a stale
            % value.
            s = obj.LatestOrbit;
        end

        function s = latestAttitude(obj)
            %LATESTATTITUDE Quaternion [x y z w] as a struct, or [].
            s = obj.LatestAttitude;
        end

        function s = latestAngularRate(obj)
            s = obj.LatestRate;
        end

        function s = latestLLA(obj)
            s = obj.LatestLLA;
        end

        function s = latestEclipse(obj)
            s = obj.LatestEclipse;
        end

        function s = latestState(obj)
            %LATESTSTATE The composite SatelliteState message, requiring
            % both orbit and attitude to be present on the bridge side
            % (see ros_publishers.py::publish_state) -- so this being
            % non-empty is a stronger guarantee than orbit+attitude
            % individually being non-empty at possibly-different times.
            s = obj.LatestState;
        end
    end

    methods (Access = private)
        function onOrbit(obj, msg)
            obj.LatestOrbit = struct( ...
                'stamp', double(msg.header.stamp.sec) + ...
                         double(msg.header.stamp.nanosec) * 1e-9, ...
                'frame_id', string(msg.header.frame_id), ...
                'position', [msg.position.x, msg.position.y, msg.position.z], ...
                'velocity', [msg.velocity.x, msg.velocity.y, msg.velocity.z], ...
                'is_extrapolated', logical(msg.is_extrapolated));
        end

        function onAttitude(obj, msg)
            obj.LatestAttitude = struct( ...
                'stamp', double(msg.header.stamp.sec) + ...
                         double(msg.header.stamp.nanosec) * 1e-9, ...
                'frame_id', string(msg.header.frame_id), ...
                'quaternion_xyzw', [msg.orientation.x, msg.orientation.y, ...
                                    msg.orientation.z, msg.orientation.w], ...
                'is_extrapolated', logical(msg.is_extrapolated));
        end

        function onRate(obj, msg)
            obj.LatestRate = struct( ...
                'stamp', double(msg.header.stamp.sec) + ...
                         double(msg.header.stamp.nanosec) * 1e-9, ...
                'angular_velocity_radps', [msg.angular_velocity.x, ...
                    msg.angular_velocity.y, msg.angular_velocity.z]);
        end

        function onLLA(obj, msg)
            obj.LatestLLA = struct( ...
                'latitude_rad', msg.latitude, ...
                'longitude_rad', msg.longitude, ...
                'altitude_m', msg.altitude_m);
        end

        function onEclipse(obj, msg)
            % Matches stk_msgs/EclipseState's constants: 0=UMBRA,
            % 1=PENUMBRA, 2=SUNLIT.
            names = ["UMBRA", "PENUMBRA", "SUNLIT"];
            obj.LatestEclipse = struct( ...
                'state', double(msg.state), ...
                'name', names(double(msg.state) + 1));
        end

        function onState(obj, msg)
            obj.LatestState = struct( ...
                'orbit_position', [msg.orbit.position.x, ...
                    msg.orbit.position.y, msg.orbit.position.z], ...
                'orbit_velocity', [msg.orbit.velocity.x, ...
                    msg.orbit.velocity.y, msg.orbit.velocity.z], ...
                'attitude_quaternion_xyzw', [msg.attitude.orientation.x, ...
                    msg.attitude.orientation.y, msg.attitude.orientation.z, ...
                    msg.attitude.orientation.w]);
        end
    end
end
