%% SETUP

close all;
clc;
clearvars;
addpath(genpath('functions/'));
% Load the robot's URDF
robot_urdf = importrobot('RPR_zyx.urdf');
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

%%
clc
[JPi, JOi] = partialJacobians(robot)

%% 
clc
config = randomConfiguration(robot_urdf);
Bs = inertialMatrix(robot);
B = evalSym(Bs, robot, config)
Bt = massMatrix(robot_urdf, config)