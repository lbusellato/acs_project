%--------------------------------------------------------------------------
%
% partialJacobians.m
%
% This script implements the computation of the partial Jacobian matrices
% for the manipulator.
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------
function [JP, JO] = partialJacobians(robot)
    JP = cat(3,sym(zeros(3)),sym(zeros(3)),sym(zeros(3)));
    JO = cat(3,sym(zeros(3)),sym(zeros(3)),sym(zeros(3)));
    p = [robot.T(1:3,4,1) robot.T(1:3,4,2) robot.T(1:3,4,3)];
    for i = 1:robot.dof
        for j = 1:i
            z = robot.T(1:3,3,j);
            if robot.joint_config(j) == 'R' % revolute
                JP(:,j,i) = cross(z,(robot.pli(:,i)-p(:,j)));
                JO(:,j,i) = z;
            else % prismatic
                JP(:,j,i) = z;
                JO(:,j,i) = sym(zeros(3,1));
            end
        end
    end
end