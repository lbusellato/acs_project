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
% Compare Lagrangian and RNE in a random config
config = randomConfiguration(robot.urdf);
new = [[config.JointPosition]'; 0.1; 0.1; 0.1];
old = [robot.q;robot.dq];
clc;
disp("Gravity vector comparison:")
vpa([subs(g_RNE,old,new), subs(robot.g,old,new)],4)
disp("Coriolis and centrifugal acceleration matrix comparison:")
vpa([subs(C_RNE,old,new), subs(robot.C*robot.dq,old,new)],4)
disp("Inertia matrix comparison:")
vpa([subs(B_RNE,old,new), subs(robot.B,old,new)],4)