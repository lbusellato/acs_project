%% SETUP
close all;
clc;
clearvars;

robot = importrobot('RPR_zyx.urdf');

showdetails(robot)

%% DISPLAY THE URDF
figure;
config = homeConfiguration(robot);
config(1).JointPosition = 0.0;
config(2).JointPosition = 0.0;
config(3).JointPosition = 0.0;
show(robot,config);
xlim([-1 1])
ylim([-1 1])
zlim([0 0.6])