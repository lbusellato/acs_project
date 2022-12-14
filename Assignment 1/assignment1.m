%--------------------------------------------------------------------------
%
% Assignment 1: DH parameters, direct and inverse kinematics, Jacobian
% matrices.
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------

%% SETUP

close all;
clc;
clearvars;
addpath(genpath('../functions/'));
% Load the robot's URDF
robot_urdf = importrobot('../RPR_zyx.urdf');
% Load the robot parameter struct
robotStruct; clc;
% Show details of the robot (link/joint names and connections)
showdetails(robot_urdf)

% Set up a figure to display the robot
figure;
config = homeConfiguration(robot_urdf);
show(robot_urdf, config);
xlim([-1 1]);
ylim([-1 1]);
zlim([0 0.6]);

%% DIRECT KINEMATICS

% Move the robot to a random pose
config = randomConfiguration(robot_urdf);
% Check my direct kinematics against the toolbox's estimation
checkDirectKinematics(robot_urdf, robot, config);

%% INVERSE KINEMATICS

% Move the robot to a random pose
config = randomConfiguration(robot_urdf);
% Check the computed inverse kinematics against the toolbox's estimation
checkInverseKinematics(robot_urdf, robot, config);

%% JACOBIANS

% Move the robot to a random pose
config = randomConfiguration(robot_urdf);
% Compare my analytical and geometric Jacobian with the toolbox's
checkJacobians(robot_urdf, robot, config);