%--------------------------------------------------------------------------
%
% Assignment 3: Equations of motion.
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------

%% SETUP

close all;
clc;
clearvars;
addpath(genpath('../functions/'));
% Load the robot parameter struct
robotStruct; clc;
% Load the robot's URDF
robot_urdf = importrobot('../RPR_zyx.urdf');
robot_urdf.DataFormat = 'row';
robot_urdf.Gravity = [0 0 -9.81].';

%% Set up a figure to display the robot
figure;
config = randomConfiguration(robot_urdf);
show(robot_urdf, config);
xlim([-1 1]);
ylim([-1 1]);
zlim([0 0.6]);

%% Inertial matrix
B = inertialMatrix(robot);
% Check against the toolbox's results
my_B = evalSym(B, robot, config)
toolbox_B = massMatrix(robot_urdf, config)

%% Coriolis and centrifugal acceleration matrix
C = christoffel(B);
% Check against the toolbox's results
my_C = subs(C*[dq1(t);dq2(t);dq3(t)], [dq1(t), dq2(t), dq3(t)], [0.5, 0.5, 0.5]);
my_C = evalSym(my_C, robot, config)
toolbox_C = velocityProduct(robot_urdf, config, [0.5, 0.5, 0.5])

%% Gravity term
config = randomConfiguration(robot_urdf);
g = gravityVector(robot);
% Check against the toolbox's results
my_g = vpa(subs(g.', robot.old, [robot.fixed_params,config]),4)
toolbox_g = gravityTorque(robot_urdf,config)
    
%% Equations of motion
% Construct the equations
syms tau1 tau2 tau3
dq = [dq1(t) dq2(t) dq3(t)].';
ddq = [ddq1(t) ddq2(t) ddq3(t)].';
tau = [tau1 tau2 tau3].';
eqns = tau == B*ddq + C*dq + g