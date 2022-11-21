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

%% Equations of motion

[eqns, B, C, g] = lagrange(robot);
% Check B against the toolbox's results
my_B = evalSym(B, robot, config)
toolbox_B = massMatrix(robot_urdf, config)
% Check C against the toolbox's results
dq = [0.1,0.1,0.1];
my_C = subs(C*[dq1(t);dq2(t);dq3(t)], [dq1(t), dq2(t), dq3(t)], dq);
my_C = evalSym(my_C, robot, config)
toolbox_C = velocityProduct(robot_urdf, config, dq)
% Check g against the toolbox's results
my_g = vpa(subs(g.', robot.old, [robot.fixed_params,config]),4)
toolbox_g = gravityTorque(robot_urdf, config)