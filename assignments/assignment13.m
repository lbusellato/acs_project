%--------------------------------------------------------------------------
%
% Assignment 13: Admittance control
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
Kp = [50;50;50;50;50;50];
Kd = [10;10;10;10;10;10];
Mt = [1 1 1 1 1 1];
KDt = [1 10 1 1 1 1];
KPt = [1 20 1 1 1 1];
% Environment pose
xr = k([0 -0.2 0]);
% Environment stiffness
K = 25*eye(6);
% Trajectory waypoints
via1 = [0 0];
via2 = [0 -0.25];
via3 = [0 0];
joint_wpts = [via1; via2; via3];
% Transform the waypoints in "wayposes"
op_wpts = zeros(6,size(joint_wpts,2));
for i = 1:size(joint_wpts,2)
    op_wpts(:,i) = k(joint_wpts(:,i));
end
time = [0 2.5];
velBound = zeros(size(op_wpts));
accBound = zeros(size(op_wpts));

%% SIMULINK 

open('admittance');