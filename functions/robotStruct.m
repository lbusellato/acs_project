%--------------------------------------------------------------------------
% Set up a struct containing information about the robot
%--------------------------------------------------------------------------

% Set up symbols for the joint variables
syms d0 a1 d2 a3 q1 q2 q3 h1 a2 h3 c2 b2 r1 r3 m1 m2 m3

% Joint/link configuration and number of DOFs
robot.joint_config = ['R', 'P', 'R']; % prismatic-revolute-prismatic
robot.link_config = ['C', 'P', 'C']; % cylinder-prism-cylinder
robot.dof = 3;
% For subs
robot.old = [d0, a1, d2, a3, h1, a2, h3, c2, b2, r1, r3, m1, m2, m3, q1, q2, q3];

% Lower and upper joint limits
robot.jointLimits = [-pi pi;  % Joint 1 - revolute
                     -0.3 0;  % Joint 2 - prismatic
                     -pi pi]; % Joint 3 - revolute
% Geometric information about the links
robot.link_geometry = [h1, r1, 0;   % h r 0
                       c2, b2, a2; % c b a
                       h3, r3, 0];  % h r 0
robot.link2_volume = 0.35*pi*0.02^2;
robot.link3_volume = 0.3*0.03^2
robot.link4_volume = 0.24*pi*0.02^2;
% Inertial information about the links
robot.g0 = [0; 0; -9.81]; % gravity
robot.link_density = 2710; % aluminum, kg/m^3
% Masses
robot.link2_mass = vpa(robot.link_density * robot.link2_volume);
robot.link3_mass = vpa(robot.link_density * robot.link3_volume);
robot.link4_mass = vpa(robot.link_density * robot.link4_volume);
robot.link_masses = [m1, m2, m3];
% Positions of the CoMs wrt the joint frames
robot.link2_com = [-0.5*h1, 0, 0];
robot.link3_com = [0, 0.5*a2, 0];
robot.link4_com = [-0.5*h3, 0, 0];
% Fixed parameters for DH
robot.d0 = 0.15;
robot.a1 = 0.4;
robot.d2 = 0.3;
robot.a3 = 0.24;
% Link dimensions for inertia
robot.h1 = 0.35;
robot.r1 = 0.02;
robot.a2 = 0.3;
robot.b2 = 0.03;
robot.c2 = 0.03;
robot.h3 = 0.24;
robot.r3 = 0.02;
robot.fixed_params = [robot.d0, robot.a1, robot.d2, robot.a3, robot.h1, ...
    robot.a2, robot.h3, robot.c2, robot.b2, robot.r1, robot.r3, ...
    robot.link2_mass, robot.link3_mass, robot.link4_mass];
% Denavit-Hartenberg parameter table
robot.DH_table_sym = [ 0, 0, d0, 0;
                       a1, pi/2, 0, q1;
                       0, -pi/2, d2+q2, pi/2;
                       a3, 0, 0, q3-pi/2];