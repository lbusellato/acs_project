%--------------------------------------------------------------------------
%
% Assignment 3: Equations of motion.
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

%% Set up a figure to display the robot
figure;
config = homeConfiguration(robot_urdf);
show(robot_urdf, config);
xlim([-1 1]);
ylim([-1 1]);
zlim([0 0.6]);

%% Equations of motion
% Compute the B and C matrices and the g vector
B = inertialMatrix(robot)
C = christoffel(robot)
g = gravityVector(robot)
% Construct the equations
eqns = tau == B*ddq + C*dq;