%--------------------------------------------------------------------------
%
% Assignment 10: Operational space inverse dynamics control law
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
joint_wpts = [via1; via2; via3];
% Transform the waypoints in "wayposes"
op_wpts = zeros(6,size(joint_wpts,2));
for i = 1:size(joint_wpts,2)
    op_wpts(:,i) = k(joint_wpts(:,i));
end
time = [0.5 1 1.5 2 2.5 3];
velBound = zeros(size(op_wpts));
accBound = zeros(size(op_wpts));
% Proportional and derivative gains
Kp = 100;
Kd = 15;

%% SIMULINK 

open('opInvDyn');