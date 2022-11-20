%--------------------------------------------------------------------------
%
% Assignment 5: Operational space dynamic model
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
robot_urdf.DataFormat = 'row';
robot_urdf.Gravity = [0 0 -9.81].';
% Load the robot parameter struct
robotStruct; clc;

%% Set up a figure to display the robot

figure;
config = homeConfiguration(robot_urdf);
show(robot_urdf, config, 'Visuals','off'); hold on;
xlim([-1 1]);
ylim([-1 1]);
zlim([0 0.6]);

%% Computation of B, C, g (in the joint space) and of the analytical Jacobian

JA = myAnalyticalJacobian(robot);
B = inertialMatrix(robot);
C = christoffel(B);
g = gravityVector(robot);

%% Pseudoinverse of the analytical Jacobian

assume([d0 a1 d2 a3 q1(t) q2(t) q3(t) h1 a2 h3 c2 b2 r1 r3 m1 m2 m3 dq1(t) ...
    dq2(t) dq3(t) ddq1(t) ddq2(t) ddq3(t)], 'real');
JA_inv = pinv(JA);

%% First derivative of the analytical Jacobian

dJA = diff(JA);
dJA = subs(dJA, [diff(q1(t), t), diff(q2(t), t), diff(q3(t), t)], [dq1(t), dq2(t), dq3(t)])

%% Computation of BA, CA and gA in the operational space

BA = inv(JA*inv(B)*JA.');
dq = [dq1(t) dq2(t) dq3(t)].';
CA = BA*JA*inv(B)*C*dq - BA*dJA*dq;
gA = BA*JA*inv(B)*g;











