%--------------------------------------------------------------------------
%
% Assignment 14: Force control with inner position loop
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
% PD control law
KP = 50*ones(6,1);
KD = 100*ones(6,1);
Md = diag([0.5;0.5;0.5;1;1;1]);
invMd = inv(Md);
KI = 5*ones(1,6);
KF = 10*ones(1,6);
% Environment pose
xr = k([0 -0.1 0]);
% Environment stiffness
K = 5*eye(6);
% Desired force
fd = [0 1 0 0 0 0]';

%% SIMULINK 

open('forceInnerPosition');