%--------------------------------------------------------------------------
%
%   directKinematics.m
%
% This script implements the computation of direct kinematics for the
% manipulator
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------
function [T, partialT] = directKinematics(robot)
    % Pull the parameters from the DH table
    a = robot.dh_table(:,1);
    alpha = robot.dh_table(:,2);
    d = robot.dh_table(:,3);
    theta = robot.dh_table(:,4);
    T = sym(zeros(4,4,robot.dof+2)); % Holds all the partial base-link transformations
    partialT = sym(zeros(4,4,robot.dof+2)); % Holds all the partial link-link transformations
    Ti = sym(eye(4)); % Holds the global matrix so far
    for i = 1:robot.dof+2 % Include world-base and last frame-ee fixed transformations
        % Transformation matrix from frame i-1 to frame i
        partialT(:,:,i) = getT(robot, i);
        Ti = simplify(Ti * partialT(:,:,i));
        T(:,:,i) = Ti;
    end
end