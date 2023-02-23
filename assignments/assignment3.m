%--------6------------------------------------------------------------------
%
% Assignment 3: Equations of motion
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

%% EQUATIONS OF MOTION
robot.tau = vpa(simplify(robot.B*robot.ddq + robot.C*robot.dq + robot.g), 4); 
robot.tau