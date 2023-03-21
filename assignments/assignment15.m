%--------------------------------------------------------------------------
%
% Assignment 15: Parallel force/position control
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

%% PARAMETERS
% PD control law
KD = 60*ones(6,1);
KP = 40*ones(6,1);
Md = diag([1;1;1;1;1;1]);
KI = 15;
KF = 15;
invMd = inv(Md);
% Environment pose
xr = k([-pi/2 -0.2 -pi/6]);
% Environment stiffness
K = diag([1 0 1 0 0 0]);
% Desired force
fd = [0.5 0 0 0 0 0]';
% Desired pose
xd = k([-pi/2 -0.1 -pi/6]);

%% SIMULINK 

open('parallel');