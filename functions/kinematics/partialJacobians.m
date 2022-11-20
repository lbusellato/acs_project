function [JPi, JOi] = partialJacobians(robotParams)
    % PARTIALJACOBIANS  Computes the partial jacobians from direct
    % kinematics. 
    %   [JPi, JOi] = PARTIALJACOBIANS(robotParams) where robotParams is a 
    % struct containing the robot's DH table.

    % Set up symbols for the joint variables
    syms d0 a1 d2 a3 q1 q2 q3

    % Compute the direct kinematics from the DH parameters
    [Ti] = myDirectKinematics(robotParams.DH_table_sym);
    % Rotation matrices
    R = cat(3, Ti(1:3,1:3,2), Ti(1:3,1:3,3), Ti(1:3,1:3,4));
    % z-axis unit vectors
    z0 = [0; 0; 1];
    z = [z0, R(:,:,1)*z0, R(:,:,2)*z0];
    % Skew matrices of the unit vectors
    Sz = cat(3, skew(z(:,1)), skew(z(:,2)), skew(z(:,3)));
    % Frame origin position vectors
    p = cat(3, Ti(1:3,4,2), Ti(1:3,4,3), Ti(1:3,4,4));
    % JPi and JOi will hold all of the partial jacobians
    JPi = sym(cat(3,zeros(3),zeros(3),zeros(3)));
    JOi = sym(cat(3,zeros(3),zeros(3),zeros(3)));
    % Cycle over the links
    for i = 1:robotParams.dof
        % Cycle over the columns of the partial jacobians
        for j = 1:i
            if robotParams.joint_config(j) == 'P' % Prismatic joint
                JOi(:,j,i) = zeros(3,1);
                JPi(:,j,i) = z(:,j);
            else    % Revolute joint
                JOi(:,j,i) = z(:,j);
                pli = R(:,:,i)*robotParams.coms(:,i)+p(:,i);
                if j == 1
                    p_j_1 = [0;0;d0];
                else
                    p_j_1 = p(:,j-1);
                end
                JPi(:,j,i) = Sz(:,:,j)*(pli - p_j_1);
            end
        end
    end
    % Simplify the expressions, if possible
    JPi = simplify(JPi);
    JOi = simplify(JOi);
end