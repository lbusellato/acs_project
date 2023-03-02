%--------------------------------------------------------------------------
%
% Assignment 11: Compliance control
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------

%% SETUP

clc;
clearvars;
close all;
addpath(genpath('../common/'));
addpath(genpath('../simulink/'));
% Load the robot struct
robotStruct;
% Set the robot in the [0 -0.3 0] configuration
robot.config(2).JointPosition = -0.3;

%% PARAMETERS

% Pose reference
xd = k([0 -0.2 0]);     
% Environment pose
xr = k([0 -0.1 0]);
% Proportional and derivative gains
Kp = [50;50;50;50;50;50];
Kd = [10;10;10;10;10;10];
% Environment stiffness
K = 1*eye(6);

%% SIMULINK 

open('compliance');