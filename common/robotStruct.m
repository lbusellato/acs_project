%--------------------------------------------------------------------------
%
%   robotStruct.m
%
% This file defines the struct for the manipulator.
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------

addpath(genpath('../common/'));
% URDF for visualization
robot.urdf = importrobot('RPR_zyx.urdf');
robot.config = homeConfiguration(robot.urdf);
% Degrees of freedom
robot.dof = 3;
% Mechanical structure
robot.joint_config = ['R', 'P', 'R']; % prismatic-revolute-prismatic
robot.link_shape = ['C', 'P', 'C']; % cylinder-prism-cylinder
robot.joint_limits = [-pi pi;
                      -0.3 0;
                      -pi pi];
robot.link_dim = [0.35 0.02 0; % h1 r1e r1i
                  0.3 0.03 0.03; % a2 b2 c2
                  0.24 0.02 0]; %h3 h3e h3i
% Inertial information
robot.g0 = [0; 0; -9.81]; % gravity
robot.link_mass = [1.192 0.7317 0.8173]; % m1 m2 m3
% Joint space coordinates
robot.q = sym('q', [3 1], 'real');
robot.dq = sym('dq', [3 1], 'real');
robot.ddq = sym('ddq', [3 1], 'real');
% Denavit-Hartenberg parameter table
robot.dh_table = [ 0, 0, 0.15, 0;
           0.4, pi/2, 0, robot.q(1);
           0, -pi/2, 0.3+robot.q(2), pi/2;
           0.24, 0, 0, robot.q(3)-pi/2;
           0, pi/2, 0, pi/2];
syms d0 a1 d2 a3
robot.dh_table_sym = [ 0, 0, d0, 0;
           a1, pi/2, 0, robot.q(1);
           0, -pi/2, d2+robot.q(2), pi/2;
           a3, 0, 0, robot.q(3)-pi/2;
           0, pi/2, 0, pi/2];
% Kinematics
robot.T = directKinematics(robot);
robot.toolboxT = getTransform(robot.urdf, robot.config, 'ee');
robot.Ta = [ 1 0 0 0        0        0;
             0 1 0 0        0        0;
             0 0 1 0        0        0;
             0 0 0 0 cos(robot.q(1)) 0;
             0 0 0 0 sin(robot.q(1)) 0;
             0 0 0 1        0        0];
robot.JG = myGeometricJacobian(robot);
robot.JA = analyticalJacobian(robot);