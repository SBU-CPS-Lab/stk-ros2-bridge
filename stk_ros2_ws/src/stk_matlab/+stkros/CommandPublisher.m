classdef CommandPublisher < handle
    %COMMANDPUBLISHER Publishes position/attitude/rate commands and the
    % lockstep barrier acknowledgement to stk_bridge.
    %
    % Mirrors interfaces/ros_subscribers.py on the bridge side. See that
    % module's docstring for why commands are newest-wins coalesced on
    % the bridge (so publishing faster than the loop drains has no
    % unbounded-latency effect) and why attitude/rate ordering is the
    % bridge's job, not the publisher's -- this class does not need to
    % sequence its own calls.
    %
    % Example (single command):
    %   pub = stkros.CommandPublisher("sss_1");
    %   pub.sendAttitude(now_stk_time, [0 0 0.3827 0.9239]);   % (x,y,z,w)
    %
    % Example (lockstep control loop, see
    % examples/example_03_lockstep_stepping.m for the full pattern):
    %   pub.acknowledgeStep("matlab_twin", stepIndex);

    properties (SetAccess = private)
        Namespace string
        Prefix string = "stk"
        ParticipantName string = "matlab_twin"
    end

    properties (Access = private)
        Node
        PubPosition
        PubAttitude
        PubRate
        PubAck
    end

    methods
        function obj = CommandPublisher(namespace, options)
            arguments
                namespace (1,1) string
                options.Prefix (1,1) string = "stk"
                options.ParticipantName (1,1) string = "matlab_twin"
                options.Node = []
            end
            obj.Namespace = namespace;
            obj.Prefix = options.Prefix;
            obj.ParticipantName = options.ParticipantName;

            if isempty(options.Node)
                obj.Node = ros2node("matlab_twin_commander");
            else
                obj.Node = options.Node;
            end

            base = sprintf("/%s/cmd", obj.Prefix);

            % Command QoS matches interfaces/qos.py's command_qos():
            % RELIABLE, depth 10. Unlike telemetry, a dropped command is
            % a real gap in STK's append-only real-time channels -- see
            % core/actuation.py's module docstring.
            obj.PubPosition = ros2publisher(obj.Node, base + "/position", ...
                "stk_msgs/PositionCommand", "Reliability", "reliable", ...
                "Depth", 10);
            obj.PubAttitude = ros2publisher(obj.Node, base + "/attitude", ...
                "stk_msgs/AttitudeCommand", "Reliability", "reliable", ...
                "Depth", 10);
            obj.PubRate = ros2publisher(obj.Node, base + "/angular_velocity", ...
                "stk_msgs/AngularRate", "Reliability", "reliable", ...
                "Depth", 10);
            obj.PubAck = ros2publisher(obj.Node, ...
                sprintf("/%s/lockstep/ack", obj.Prefix), "std_msgs/String", ...
                "Reliability", "reliable", "Depth", 10);
        end

        function sendAttitude(obj, stampSeconds, quaternionXYZW, options)
            %SENDATTITUDE Send a real-time attitude command.
            %
            %   pub.sendAttitude(t, [qx qy qz qw]) sends via the
            %   bridge's default reference frame (see
            %   actuation.default_attitude_inertial in bridge_params.yaml
            %   -- CBFQuat/'fixed' unless configured otherwise; both
            %   command forms were confirmed to round-trip at 0.0000 deg
            %   error on a live STK 12.6 install, see docs/03...).
            %
            %   pub.sendAttitude(t, q, 'FrameId', 'icrf') selects the
            %   inertial form explicitly (AddAttitude Quat).
            arguments
                obj
                stampSeconds (1,1) double
                quaternionXYZW (1,4) double
                options.FrameId (1,1) string = ""
            end
            msg = ros2message(obj.PubAttitude);
            msg.header.stamp = stkros.CommandPublisher.toRosTime(stampSeconds);
            if options.FrameId ~= ""
                msg.header.frame_id = char(options.FrameId);
            end
            msg.object_path = obj.objectPath();
            msg.orientation.x = quaternionXYZW(1);
            msg.orientation.y = quaternionXYZW(2);
            msg.orientation.z = quaternionXYZW(3);
            msg.orientation.w = quaternionXYZW(4);
            send(obj.PubAttitude, msg);
        end

        function sendAngularVelocity(obj, stampSeconds, omegaRadPerSec)
            %SENDANGULARVELOCITY Body angular rate, rad/s.
            %
            % NOTE: STK's AddAttitude AngVels requires a PRIOR attitude
            % sample for this object or it is rejected (see
            % core/actuation.py). Send an attitude command before -- or
            % on the same step as -- the first rate command.
            arguments
                obj
                stampSeconds (1,1) double
                omegaRadPerSec (1,3) double
            end
            msg = ros2message(obj.PubRate);
            msg.header.stamp = stkros.CommandPublisher.toRosTime(stampSeconds);
            msg.object_path = obj.objectPath();
            msg.angular_velocity.x = omegaRadPerSec(1);
            msg.angular_velocity.y = omegaRadPerSec(2);
            msg.angular_velocity.z = omegaRadPerSec(3);
            send(obj.PubRate, msg);
        end

        function sendPosition(obj, stampSeconds, positionM, options)
            %SENDPOSITION Real-time position injection, metres, ICRF by
            % default.
            arguments
                obj
                stampSeconds (1,1) double
                positionM (1,3) double
                options.FrameId (1,1) string = "icrf"
                options.VelocityMps (1,3) double = [NaN NaN NaN]
            end
            msg = ros2message(obj.PubPosition);
            msg.header.stamp = stkros.CommandPublisher.toRosTime(stampSeconds);
            msg.header.frame_id = char(options.FrameId);
            msg.object_path = obj.objectPath();
            msg.position.x = positionM(1);
            msg.position.y = positionM(2);
            msg.position.z = positionM(3);
            if ~any(isnan(options.VelocityMps))
                msg.velocity.x = options.VelocityMps(1);
                msg.velocity.y = options.VelocityMps(2);
                msg.velocity.z = options.VelocityMps(3);
                msg.velocity_valid = true;
            else
                msg.velocity_valid = false;
            end
            send(obj.PubPosition, msg);
        end

        function acknowledgeStep(obj, participantName, stepIndex)
            %ACKNOWLEDGESTEP Release the bridge's lockstep barrier for
            % this step.
            %
            % Encoding is a plain "<participant>:<step_index>" string on
            % purpose -- see interfaces/ros_subscribers.py's _on_ack for
            % why a custom message was not used (trivially publishable
            % from a script or shell, no codegen dependency for this one
            % topic). Call this once per step, after this iteration's
            % control computation is complete and any resulting commands
            % have been published -- publishing the ack BEFORE the
            % commands would let the bridge advance past a step whose
            % control input has not landed yet.
            arguments
                obj
                participantName (1,1) string
                stepIndex (1,1) double
            end
            msg = ros2message(obj.PubAck);
            msg.data = char(sprintf("%s:%d", participantName, ...
                round(stepIndex)));
            send(obj.PubAck, msg);
        end
    end

    methods (Access = private)
        function path = objectPath(obj)
            % Reconstructs the canonical STK path from the topic
            % namespace. Matches ros_publishers.py::topic_namespace's
            % lower-cased, underscore-joined transform closely enough
            % for the common case (SSS-1 -> sss_1); scenarios with
            % ambiguous namespace collisions should set object_path
            % directly on the message instead of relying on this
            % heuristic -- see docs/03_message_reference.md.
            path = char("*/Satellite/" + upper(strrep(obj.Namespace, "_", "-")));
        end
    end

    methods (Static, Access = private)
        function t = toRosTime(seconds)
            t = struct('sec', int32(floor(seconds)), ...
                       'nanosec', uint32(round(mod(seconds, 1) * 1e9)));
        end
    end
end
