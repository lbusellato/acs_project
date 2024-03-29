%--------------------------------------------------------------------------
%
% Assignment 6: Joint space PD control law with gravity compensation
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

% Constant position reference
qd = [pi/2 -0.2 pi]';
% Proportional and derivative gains
Kp = [50 500 50];
Kd = [15 150 15];

%% SIMULINK 

open('jointSpacePDControl');