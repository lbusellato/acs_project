classdef Robot < handle
    
    properties 
        urdf
        config
        dh
        links
        dof
        partial_T
        T
        JA
        JG
        q
        dq
        ddq
    end

    methods
        function obj = Robot(urdf_file, dh_table, links)
            obj.urdf = importrobot(urdf_file);
            obj.config = homeConfiguration(obj.urdf);
            obj.dh = dh_table;
            obj.dof = length(links);
            obj.q = sym('q', [3 1], 'real');
            obj.dq = sym('dq', [3 1], 'real');
            obj.ddq = sym('ddq', [3 1], 'real');   
            obj.links = links;
            obj.directKinematics;
            obj.jacobians;
        end

        % Computes the direct kinematics from the robot's DH table
        function directKinematics(obj)
            [rows ~] = size(obj.dh);
            % Pull each column from the DH table
            a     = obj.dh(:, 1);
            alpha = obj.dh(:, 2);
            d     = obj.dh(:, 3);
            theta = obj.dh(:, 4);
            % T also holds the intermediate transformation matrices
            Ti = sym(eye(4));
            T = sym(zeros(4,4,rows));
            % Direct kinematics
            for i = 1:rows
                % Partial transformation matrix
                t = [cos(theta(i)), -sin(theta(i))*cos(alpha(i)),  sin(theta(i))*sin(alpha(i)), a(i)*cos(theta(i));
                     sin(theta(i)),  cos(theta(i))*cos(alpha(i)), -cos(theta(i))*sin(alpha(i)), a(i)*sin(theta(i));
                     0,              sin(alpha(i)),                cos(alpha(i)),               d(i);
                     0,              0,                            0,                           1];
                % Update the global transformation matrix
                Ti = Ti * simplify(t);
                % Keep track of the partial matrix
                T(:,:,i) = Ti;
            end
            % Simplify the result if possible
            T = simplify(T);
            obj.T = T(:,:,end);
            obj.partial_T = T(:,:,:)
        end

        % Computes the inverse kinematics
        function config = inverseKinematics(obj, ee_position, joint3_position)
            loadValues;
            px = ee_position(1); py = ee_position(2); pz = ee_position(3) - d0;
            wx = joint3_position(1); wy = joint3_position(2);
            % q2
            q2p = -d2-real(sqrt(wx^2+wy^2-a1^2));
            q2n = -d2+real(sqrt(wx^2+wy^2-a1^2));
            % Pick the solution that respects the joint limits of the prismatic joint
            if obj.urdf.Bodies{2}.Joint.PositionLimits(1) <= q2p && ...
                    q2p <= obj.urdf.Bodies{2}.Joint.PositionLimits(2)
                q2 = q2p;
            else
                q2 = q2n;
            end
            % Compute all four solutions for q2 using q3 and the ee's position
            S3 = pz / a3;
            C3 = real(sqrt(1-S3^2));
            theta3 = [atan2(S3, C3), atan2(S3, -C3)];
            d23 = [-a3*C3-d2+real(sqrt(px^2+py^2-a1^2));
                   a3*C3-d2+real(sqrt(px^2+py^2-a1^2));
                  -a3*C3-d2-real(sqrt(px^2+py^2-a1^2));
                   a3*C3-d2-real(sqrt(px^2+py^2-a1^2))];
            % Get the index of the solution closest to the actual value of q2 computed before
            [~,closestIndex] = min(abs(d23 - q2));
            % Choose the angle corresponding to the closest solution as q3
            if closestIndex == 1 || closestIndex == 3
                q3 = theta3(1);
            else
                q3 = theta3(2);
            end
            % q1
            d = q2 + d2;
            C1 = (a1*wx - d*wy)/((q2 + d2)^2 + a1^2);
            S1 = (wx - a1*C1)/(q2 + d2);
            q1 = atan2(S1, C1);
            config = [q1, q2, q3];
        end

        % Computes the geometric jacobian from direct kinematics
        function J = geometricJacobian(obj)
            % JG will hold the jacobian matrix
            J = sym(zeros(6, obj.dof));
            % Get the ee's position
            p3 = obj.T(1:3,4);
            % Iteratively construct the matrix
            for i = 1:obj.dof
                % Get the i-th joint's z-axis versor
                zi = obj.partial_T(1:3,3,i);
                if strcmpi('prismatic', obj.urdf.Bodies{i}.Joint.Type)
                    % Apply the formula for a prismatic joint
                    J(:,i) = [zi; 0; 0; 0];
                else
                    % Apply the formula for a revolute joint
                    pi = obj.partial_T(1:3,4,i);
                    % Compute the skew matrix of zi for the cross product
                    Szi = skew(zi); 
                    J(:,i) = [Szi*(p3-pi); zi];
                end
            end
            % Simplify if possible
            J = simplify(J);
        end

        % Compute the transformation matrix ( JG = TA*JA ) 
        function t = Ta(obj)
            T = [0 cos(obj.q(1)) 0;
                 0 sin(obj.q(1)) 0;
                 1 0 0];
            t = [eye(3) zeros(3)
                  zeros(3) T];
            t = simplify(t);
        end

        % Compute the geometric and analytical jacobians
        function jacobians(obj)
            obj.JG = geometricJacobian(obj);
            obj.JA = simplify(pinv(obj.Ta)*obj.JG);
        end

        % Perform symbolic variable substitutions
        function out = setValues(obj, var, config)
            loadValues;
            if nargin == 2
                config = [obj.config.JointPosition].';
            end
            out = subs(var, obj.q, config);
            out = subs(out);
        end
    end
end