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
                  0.24 0.02 0]; %h3 r3e r3i
% Joint space coordinates
robot.q = sym('q', [3 1], 'real');
robot.dq = sym('dq', [3 1], 'real');
robot.ddq = sym('ddq', [3 1], 'real');
% Operational space coordinates
robot.x = sym('x', [6 1], 'real');
robot.dx = sym('dx', [6 1], 'real');
robot.ddx = sym('ddx', [6 1], 'real');
% Initial values
robot.q0 = [0 0 0].';
robot.dq0 = [0 0 0].';
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
[robot.T, robot.partialT] = directKinematics(robot);
robot.toolboxT = getTransform(robot.urdf, robot.config, 'ee');
robot.Ta = [ 1 0 0 0        0        0;
             0 1 0 0        0        0;
             0 0 1 0        0        0;
             0 0 0 0 cos(robot.q(1)) 0;
             0 0 0 0 sin(robot.q(1)) 0;
             0 0 0 1        0        0]; % JG = TA*JA
robot.JG = myGeometricJacobian(robot);
robot.JA = analyticalJacobian(robot);
% Inertial information
robot.g0 = [0; 0; -9.81]; % gravity
robot.link_mass = [1.192 0.7317 0.8173]; % m1 m2 m3
%robot.link_mass = [0.9 0.5 0.7]; % For the inverse dynamics control law
robot.I1 = inertiaTensor(robot, 1);
robot.I2 = inertiaTensor(robot, 2);
robot.I3 = inertiaTensor(robot, 3);
robot.I = cat(3, robot.I1, robot.I2, robot.I3);
robot.pl1_0 = [-0.2 0 0].';   % CoM of link 1 wrt frame 1
robot.pl2_1 = [0 0.15 0].'; % CoM of link 2 wrt frame 2
robot.pl3_2 = [-0.12 0 0].'; % CoM of link 3 wrt frame 3
robot.pl1 = robot.T(1:3,1:3,2)*robot.pl1_0 + robot.T(1:3,4,2); % CoM of link 1 wrt base frame
robot.pl2 = robot.T(1:3,1:3,3)*robot.pl2_1 + robot.T(1:3,4,3); % CoM of link 2 wrt base frame
robot.pl3 = robot.T(1:3,1:3,4)*robot.pl3_2 + robot.T(1:3,4,4); % CoM of link 3 wrt base frame
robot.pli = [robot.pl1 robot.pl2 robot.pl3];
[robot.JP, robot.JO] = partialJacobians(robot);
% Dynamic model - joint space
robot.B = inertiaMatrix(robot); % Inertia matrix of the manipulator
robot.C = coriolis(robot); % Coriolis and centrifugal acceleration matrix
robot.g = gravity(robot);
robot.tau = sym('tau', [3 1], 'real');
% Dynamic model - operational space
robot.JAinv = pinv(robot.JA);
robot.JATinv = pinv(robot.JA.');
robot.dJA = DanalyticalJacobian(robot);
robot.BA = robot.JATinv*robot.B*robot.JAinv;
robot.CA = robot.JATinv*robot.C*robot.dq - robot.BA*robot.dJA*robot.dq;
robot.gA = robot.JATinv*robot.g;
robot.tauA = sym('tauA', [3 1], 'real');