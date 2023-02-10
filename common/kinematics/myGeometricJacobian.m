%--------------------------------------------------------------------------
%
%   myGeometricJacobian.m
%
% This script computes the geometric Jacobian matrix for the manipulator.
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------
function JG = myGeometricJacobian(robot)
    % JG will hold the jacobian matrix
    JG = sym(zeros(6, robot.dof));
    % Get the ee's position
    p3 = robot.T(1:3,4,end);
    % Iteratively construct the matrix
    for i = 1:robot.dof
        % Get the i-th joint's z-axis versor
        zi = robot.T(1:3,3,i);
        if robot.joint_config(i) == 'P'
            % Apply the formula for a prismatic joint
            JG(:,i) = [zi; 0; 0; 0];
        else
            % Apply the formula for a revolute joint
            pi = robot.T(1:3,4,i);
            % Compute the skew matrix of zi for the cross product
            Szi = skew(zi); 
            JG(:,i) = [Szi*(p3-pi); zi];
        end
    end
    % Simplify if possible
    JG = simplify(JG);
end