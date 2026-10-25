classdef quat
%QUAT Quaternion algebra for the ADCS twin, in the bridge's conventions.
%
% A transcription of acav/attitude.py. The conventions are fixed by
% fixtures generated from that module and replayed by test_acav_twin.m,
% so that the two implementations cannot silently disagree: a sign
% error in quaternion algebra does not crash, it steers the twin
% smoothly to the wrong attitude.
%
% Conventions, all asserted by fixtures:
%   order      [x y z w] -- vector part first, matching the bridge and
%              ROS geometry_msgs/Quaternion. STK's Q1..Q3 are the vector
%              part and Q4 the scalar.
%   direction  q rotates a vector FROM INERTIAL INTO BODY components.
%   product    Hamilton, with R(a (x) b) = R(b) R(a).
%   rates      omega is body-referenced, rad/s, and
%              qdot = 0.5 * q (x) [omega 0].
%
% All methods are static; this is a namespace, not an object.

    methods (Static)

        function q = identity()
            q = [0 0 0 1];
        end

        function q = normalize(q)
            n = norm(q);
            if n < 1e-12
                error('adcs:quat:Degenerate', ...
                    'quaternion has no magnitude');
            end
            q = q(:).' / n;
        end

        function q = conjugate(q)
            q = [-q(1) -q(2) -q(3) q(4)];
        end

        function c = multiply(a, b)
            %MULTIPLY Hamilton product a (x) b.
            c = [ a(4)*b(1) + b(4)*a(1) + a(2)*b(3) - a(3)*b(2), ...
                  a(4)*b(2) + b(4)*a(2) + a(3)*b(1) - a(1)*b(3), ...
                  a(4)*b(3) + b(4)*a(3) + a(1)*b(2) - a(2)*b(1), ...
                  a(4)*b(4) - a(1)*b(1) - a(2)*b(2) - a(3)*b(3) ];
        end

        function q = fromAxisAngle(axis, angleRad)
            axis = axis(:).' / norm(axis);
            h = 0.5 * angleRad;
            q = [axis * sin(h), cos(h)];
        end

        function [axis, angleRad] = toAxisAngle(q)
            %TOAXISANGLE Shortest-way-round axis and angle in [0, pi].
            %
            % The sign flip matters: q and -q are the same attitude, and
            % a controller handed the long-way-round error slews 350 deg
            % to reach a 10 deg target.
            q = adcs.quat.normalize(q);
            if q(4) < 0
                q = -q;
            end
            vn = norm(q(1:3));
            angleRad = 2 * atan2(vn, q(4));
            if vn < 1e-12
                axis = [1 0 0];
            else
                axis = q(1:3) / vn;
            end
        end

        function vBody = toBody(q, vInertial)
            %TOBODY Express an inertial vector in body axes.
            v = q(1:3);
            w = q(4);
            vInertial = vInertial(:).';
            vBody = vInertial * (w*w - dot(v, v)) ...
                  + v * (2 * dot(v, vInertial)) ...
                  + cross(v, vInertial) * (-2 * w);
        end

        function vInertial = toInertial(q, vBody)
            vInertial = adcs.quat.toBody(adcs.quat.conjugate(q), vBody);
        end

        function qd = derivative(q, omegaBody)
            %DERIVATIVE qdot = 0.5 * q (x) [omega 0].
            qd = 0.5 * adcs.quat.multiply(q, [omegaBody(:).' 0]);
        end

        function qe = error(q, qReference)
            %ERROR Rotation from the reference body frame to the actual.
            %
            % Its vector part is what the controller drives to zero, and
            % it is expressed in BODY axes -- which is where the torque
            % is applied.
            qe = adcs.quat.multiply(adcs.quat.conjugate(qReference), q);
        end

        function angleRad = pointingError(q, qReference)
            [~, angleRad] = adcs.quat.toAxisAngle( ...
                adcs.quat.error(q, qReference));
        end

        function angleRad = boresightError(q, qReference, boresightBody)
            %BORESIGHTERROR Angle between achieved and desired boresight.
            %
            % Distinct from pointingError, and the distinction decides
            % captures: rotation ABOUT the boresight does not move the
            % footprint, so a twin settled in boresight but not in roll
            % still images the target. STK's access computation cares
            % only about the boresight, so the capture criterion must
            % too; scoring total attitude error would fail targets the
            % sensor actually sees.
            if nargin < 3
                boresightBody = [0 0 1];
            end
            a = adcs.quat.toInertial(q, boresightBody);
            b = adcs.quat.toInertial(qReference, boresightBody);
            % atan2 of cross and dot, not acos of dot: acos loses half
            % its significant digits near zero, and this number is
            % compared against a 0.5 deg threshold.
            angleRad = atan2(norm(cross(a, b)), dot(a, b));
        end

        function q = align(primaryBody, primaryInertial, ...
                           secondaryBody, secondaryInertial)
            %ALIGN Point a body axis at a direction; pin the roll.
            %
            % TRIAD. primaryBody is aimed exactly at primaryInertial and
            % the secondary pair removes the remaining freedom about it.
            % Every commanded attitude of the twin is built this way.
            pB = primaryBody(:).'  / norm(primaryBody);
            pI = primaryInertial(:).' / norm(primaryInertial);
            sB = secondaryBody(:).' / norm(secondaryBody);
            sI = secondaryInertial(:).' / norm(secondaryInertial);

            if abs(dot(pI, sI)) > 1 - 1e-9
                error('adcs:quat:ParallelConstraint', ...
                    ['the primary and secondary inertial directions are ' ...
                     'parallel, so the rotation about the boresight is ' ...
                     'undefined. An arbitrary but stable roll would look ' ...
                     'fine and quietly change the footprint orientation.']);
            end

            b1 = pB;  b2 = cross(pB, sB); b2 = b2 / norm(b2);
            b3 = cross(b1, b2);
            i1 = pI;  i2 = cross(pI, sI); i2 = i2 / norm(i2);
            i3 = cross(i1, i2);

            % B' * I has, as its rows, the body axes expressed in
            % inertial components -- which is exactly the
            % inertial-to-body matrix. No further transpose: taking one
            % here produces the inverse rotation, a valid attitude that
            % points the boresight ~180 deg away. Checked against the
            % Python reference over random geometries.
            B = [b1; b2; b3];
            I = [i1; i2; i3];
            q = adcs.quat.fromRotation(B.' * I);
        end

        function q = fromRotation(R)
            %FROMROTATION Shepperd's method on an inertial-to-body matrix.
            %
            % Picks the largest of the four denominators, which stays
            % conditioned for every attitude including the 180 deg cases
            % where the naive formula divides by zero.
            tr = R(1,1) + R(2,2) + R(3,3);
            if tr > 0
                s = sqrt(tr + 1) * 2;
                w = 0.25 * s;
                x = (R(2,3) - R(3,2)) / s;
                y = (R(3,1) - R(1,3)) / s;
                z = (R(1,2) - R(2,1)) / s;
            elseif R(1,1) > R(2,2) && R(1,1) > R(3,3)
                s = sqrt(1 + R(1,1) - R(2,2) - R(3,3)) * 2;
                w = (R(2,3) - R(3,2)) / s;
                x = 0.25 * s;
                y = (R(2,1) + R(1,2)) / s;
                z = (R(3,1) + R(1,3)) / s;
            elseif R(2,2) > R(3,3)
                s = sqrt(1 + R(2,2) - R(1,1) - R(3,3)) * 2;
                w = (R(3,1) - R(1,3)) / s;
                x = (R(2,1) + R(1,2)) / s;
                y = 0.25 * s;
                z = (R(3,2) + R(2,3)) / s;
            else
                s = sqrt(1 + R(3,3) - R(1,1) - R(2,2)) * 2;
                w = (R(1,2) - R(2,1)) / s;
                x = (R(3,1) + R(1,3)) / s;
                y = (R(3,2) + R(2,3)) / s;
                z = 0.25 * s;
            end
            q = adcs.quat.normalize([x y z w]);
        end
    end
end
