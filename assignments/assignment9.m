%--------------------------------------------------------------------------
%
% Assignment 9: Operational space PD control law with gravity compensation
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

% Constant pose reference
xd = k([-pi/3 -0.1 pi/2]);
% Proportional and derivative gains
Kp = [50;50;50;50;50;50];
Kd = [10;10;10;10;10;10];

%% SIMULINK 

open('opSpacePDControl');