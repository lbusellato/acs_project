%--------------------------------------------------------------------------
%
% Assignment 1: DH parameters, direct and inverse kinematics, Jacobian
% matrices.
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------

%% SETUP

clc;
clearvars;
close all;
addpath(genpath('../common/'));
% Load the robot struct
robotStruct;
% Show the robot
show(robot.urdf, robot.config);
xlim([-1 1]);
ylim([-1 1]);
zlim([0 0.6]);
subs(robot.pl1,robot.q,robot.config.JointPosition);
scatter3(robot.pl1(1), robot.pl1(2),robot.pl1(3));

%% DIRECT KINEMATICS

fprintf('Base to ee transformation matrix:');
T0_ee = robot.T(:,:,5)
ee_pose = subs(T0_ee(1:3,4), robot.q, [0 0 0].');
fprintf('EE position using my direct kinematics: [%.2f %.2f %.2f]\n', ee_pose(1), ee_pose(2), ee_pose(3))
ee_pose = robot.toolboxT(1:3,4);
fprintf('EE position using the toolbox direct kinematics: [%.2f %.2f %.2f]\n\n', ee_pose(1), ee_pose(2), ee_pose(3))

%% INVERSE KINEMATICS

robot.config = randomConfiguration(robot.urdf);
fprintf('Actual configuration: [%.2f %.2f %.2f]\n', robot.config.JointPosition)
config = myInverseKinematics(robot);
fprintf('Configuration from my inverse kinematics: [%.2f %.2f %.2f]\n', config)
config = toolboxInverseKinematics(robot);
fprintf('Configuration from the toolbox inverse kinematics: [%.2f %.2f %.2f]\n\n', config)

%% JACOBIANS

fprintf('Analytical expression for JG:');
robot.JG
fprintf('My JG evaluated in the current robot config:');
JG = vpa(subs(robot.JG, robot.q, [robot.config.JointPosition].'),3)
fprintf("Toolbox's JG evaluated in the current robot config (note that the first 3 rows are swapped with the last 3):");
JG = vpa(geometricJacobian(robot.urdf, robot.config, 'ee'),3)
fprintf('\nAnalytical expression for JA:');
robot.JA