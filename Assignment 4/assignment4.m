%--------------------------------------------------------------------------
%
% Assignment4: Recursive Newton-Euler formulation.
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------

%% SETUP

close all;
clc;
clearvars;
addpath(genpath('../functions/'));
% Load the robot's URDF
robot_urdf = importrobot('../RPR_zyx.urdf');
% Load the robot parameter struct
robotStruct; clc;

%% Set up a figure to display the robot

figure;
config = homeConfiguration(robot_urdf);
show(robot_urdf, config);
xlim([-1 1]);
ylim([-1 1]);
zlim([0 0.6]);

%% Recursive Newton-Euler formulation
clc
syms q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3
q = [q1 q2 q3];
dq = [dq1 dq2 dq3];
ddq = [ddq1 ddq2 ddq3];
ddP0 = sym([0 0 0]).' - robot.g0;
tau = NE(robot, q, dq, ddq, ddP0)

% Compute the B and C matrices and the g vector
%B = inertialMatrix(robot);
%C = christoffel(B);
%g = gravityVector(robot)
% Construct the equations
%syms tau1 tau2 tau3 ddq1 ddq2 ddq3 dq1 dq2 dq3
%dq = [dq1 dq2 dq3].';
%ddq = [ddq1 ddq2 ddq3].';
%tau = [tau1 tau2 tau3].';
%eqns = tau == B*ddq + C*dq + g
