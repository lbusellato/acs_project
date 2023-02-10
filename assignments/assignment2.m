%--------------------------------------------------------------------------
%
% Assignment 2: Kinetic and potential energy
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

%% KINETIC ENERGY

fprintf("Analytical expression of the kinetic energy:\n")
T = kineticEnergy(robot)
robot.config = randomConfiguration(robot.urdf);
fprintf("Kinetic energy evaluated with random q and null qdot: %.4f J\n", kineticEnergy(robot, [robot.config.JointPosition].', [0 0 0].'))

%% POTENTIAL ENERGY

fprintf("Analytical expression of the potential energy:\n")
U = potentialEnergy(robot)
robot.conig = homeConfiguration(robot.urdf);
fprintf("Potential energy evaluated in the home position: %.4f J\n", potentialEnergy(robot, [robot.config.JointPosition].'))
