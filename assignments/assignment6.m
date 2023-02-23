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
qd = [pi/3 -0.1 pi/4]';
% Proportional and derivative gains
Kp = 50;
Kd = 10;
% Kp = 150; % For sin reference
% Kd = 10; % For sin reference

%% SIMULINK 

open('jointSpacePDControl');