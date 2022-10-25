function [JPi, JOi] = partialJacobians(robotParams, com)
    % PARTIALJACOBIANS  Computes the partial jacobians from direct
    % kinematics. 
    %   [JPi, JOi] = PARTIALJACOBIANS(robotParams) where robotParams is a 
    % struct containing the robot's DH table.

    % Set up symbols for the joint variables
    syms d0 a1 d2 a3 q1 q2 q3

    % Compute the direct kinematics from the DH parameters
    [Ti] = myDirectKinematics(robotParams.DH_table_sym);
    % Rotation matrices
    R = cat(3, eye(3), Ti(1:3,1:3,2), Ti(1:3,1:3,3));
    % z-axis unit vectors
    z0 = [0; 0; 1];
    z = [R(:,:,1)*z0, R(:,:,2)*z0, R(:,:,3)*z0];
    % Skew matrices of the unit vectors
    Sz = cat(3, skew(z(:,1)), skew(z(:,2)), skew(z(:,3)));
    % Frame origin position vectors
    p = cat(3, zeros(3,1), Ti(1:3,4,2), Ti(1:3,4,3));
    % JPi and JOi will hold all of the partial jacobians
    JPi = sym(cat(3,zeros(3),zeros(3),zeros(3)));
    JOi = sym(cat(3,zeros(3),zeros(3),zeros(3)));
    % Cycle over the provided CoM vectors
    for i = 1:size(com, 2)
        JO = sym(zeros(3));
        JP = sym(zeros(3));
        % Cycle over the columns of the partial jacobians
        for j = 1:i
            if robotParams.joint_config(j) == 'P' % Prismatic joint
                JO(:,j) = zeros(3,1);
                JP(:,j) = z(:,j);
            else    % Revolute joint
                JO(:,j) = z(:,j);
                JP(:,j) = Sz(:,:,j)*(com(1:3,i)-p(:,i));
            end
        end
        JOi(:,:,i) = JO;
        JPi(:,:,i) = JP;
    end
    % Simplify the expressions, if possible
    JPi = simplify(JPi);
    JOi = simplify(JOi);
end