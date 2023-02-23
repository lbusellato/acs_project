%--------------------------------------------------------------------------
%
% Assignment 5: Operational space dynamic model
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------

%% SETUP

clc;
clearvars;
close all;
addpath(genpath('../common/'));
% Load the robot struct
robotStruct;

%% OPERATIONAL SPACE DYNAMIC MODEL
robot.tauA = vpa(simplify(robot.BA*robot.ddx + robot.CA + robot.gA), 4); 
%robot.tauA % This takes a LONG time
