%--------------------------------------------------------------------------
%
% Assignment 7: Joint space inverse dynamics control law
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

% Trajectory waypoints
via1 = [pi/3 -pi/3 0 pi/2 pi/3 0];
via2 = [-0.2 -0.1 -0.2 0 -0.1 0];
via3 = [pi/3 pi/4 0 -pi/4 pi/4 0];
wpts = [via1; via2; via3];
time = [0.5 1 1.5 2 2.5 3];
velBound = zeros(size(wpts));
accBound = zeros(size(wpts));
% Proportional and derivative gains
Kp = 50;
Kd = 15;
% Kp = 500; % Small rising time
% Kd = 35; % Small rising time
% Set to 0 to use actual B, to 1 to use B hat
error = 0;

%% SIMULINK 

open('jointInvDyn');