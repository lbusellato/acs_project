%--------------------------------------------------------------------------
%
% Assignment 4: Equations of motion - Recursive Newton-Euler formulation
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

%% EQUATIONS OF MOTION - RECURSIVE NEWTON EULER FORMULATION

zero = zeros(3,1);
g_RNE = RNE(robot, robot.q, zero, zero, robot.g0);
C_RNE = RNE(robot, robot.q, robot.dq, zero, zero);
B_RNE = sym(zeros(robot.dof));
for i = 1:robot.dof
    ei = zeros(robot.dof, 1);
    ei(i) = 1;
    B_RNE(:,i) = RNE(robot, robot.q, zero, ei, zero);
end
g_RNE = vpa(g_RNE,4);
C_RNE = vpa(C_RNE,4);
B_RNE = vpa(B_RNE,4);