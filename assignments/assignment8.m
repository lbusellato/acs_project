%--------------------------------------------------------------------------
%
% Assignment 8: 1-Dof adaptive control law
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

% Trajectory parameters
A = 1;
% Dynamic parameters
I = 0.5;
F = 0.15;
G = 1.85;
% Initial estimations
I_est = I - 0.01;
F_est = F - 0.01;
G_est = G - 0.01;
lambda = 200;
K = inv(diag([5000 100 25]));
% Derivative gain
Kd = 40;
Ts = 0.001;
% Initial conditions
qi = 0;
dqi = 0;

%% SIMULINK 

open('adaptive');