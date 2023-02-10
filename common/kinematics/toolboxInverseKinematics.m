%--------------------------------------------------------------------------
%
%   toolboxInverseKinematics.m
%
% This script uses MATLAB's IK solver to compute inverse kinematics for the
% manipulator, for the comparison in assignment 1.
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------
function config = toolboxInverseKinematics(robot)
    % Get the toolbox's transformation matrix from the ee to the base link
    toolbox_base_to_ee = getTransform(robot.urdf, robot.config, 'ee', 'base_link');
    % Set up the toolbox's numerical inverse kinematics solver
    ik = inverseKinematics('RigidBodyTree', robot.urdf);
    % Weights for the positions (0.25) and orientations (1)
    weights = [0.25 0.25 0.25 1 1 1];
    % Initial guess for the joint variables
    initialguess = robot.urdf.homeConfiguration;
    % Estimate joint variables with the toolbox
    [configSoln, ~] = ik('ee', toolbox_base_to_ee, weights, initialguess);
    config = [configSoln.JointPosition];
end