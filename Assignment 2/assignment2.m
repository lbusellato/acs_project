%--------------------------------------------------------------------------
%
% Assignment 2: Kinetic energy and potential energy.
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
robot_urdf.DataFormat = 'row';
% Load the robot parameter struct
robotStruct; clc;

%% Set up a figure to display the robot
figure;
config = homeConfiguration(robot_urdf);
show(robot_urdf, config, 'Visuals','off'); hold on;
xlim([-1 1]);
ylim([-1 1]);
zlim([0 0.6]);

%% Kinetic energy
T = kineticEnergy(robot)

%% Potential energy
U = potentialEnergy(robot)