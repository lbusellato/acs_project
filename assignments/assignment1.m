%--------------------------------------------------------------------------
%
% Assignment 1: DH parameters, direct and inverse kinematics, Jacobian
% matrices.
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------

%% SETUP
    
% Set up a figure to display the robot
figure;
show(robot.urdf, robot.config);
xlim([-1 1]);
ylim([-1 1]);
zlim([0 0.6]);

%% DIRECT KINEMATICS

robot.config = homeConfiguration(robot.urdf);
disp("Analytic expression for direct kinematics:")
robot.T
% Compare with the toolbox's results
disp("My direct kinematics evaluated in the home configuration:")
myT = vpa(robot.setValues(robot.T), 2)
disp("Toolbox's direct kinematics evaluated in the home configuration:")
toolboxT = vpa(getTransform(robot.urdf, robot.config, 'ee'),2)

%% INVERSE KINEMATICS
robot.config = randomConfiguration(robot.urdf);
% Get the toolbox's transformation matrix from the ee to the base link
toolbox_base_to_ee = getTransform(robot.urdf, robot.config, 'ee', 'base_link');
ee_pos = toolbox_base_to_ee(1:3,4);
% Get the toolbox's transformation from the last revolute to the base link
toolbox_base_to_link4 = getTransform(robot.urdf, robot.config, 'Link4', 'base_link');
link4_pos = toolbox_base_to_link4(1:3,4);
% Set up the toolbox's numerical inverse kinematics solver
ik = inverseKinematics('RigidBodyTree', robot.urdf);
% Weights for the positions (0.25) and orientations (1)
weights = [0.25 0.25 0.25 1 1 1];
% Initial guess for the joint variables
initialguess = robot.urdf.homeConfiguration;
tform = getTransform(robot.urdf, robot.config, 'ee', 'base_link');
% Estimate joint variables with the toolbox
[configSoln, solnInfo] = ik('ee', toolbox_base_to_ee, weights, initialguess);
disp("Actual configuration:")
[robot.config.JointPosition]
disp("My inverse kinematics:")
robot.inverseKinematics(ee_pos, link4_pos)
disp("Toolbox's inverse kinematics:")
toolbox_ik = [configSoln(1).JointPosition;
            configSoln(2).JointPosition;
            configSoln(3).JointPosition].'

%% GEOMETRIC JACOBIAN

robot.config = homeConfiguration(robot.urdf);
disp("Analytic expression for the geometric jacobian:")
robot.JG
disp("My geometric jacobian evaluated in the home position:")
vpa(robot.setValues(robot.JG), 2)
disp("Toolbox's geometric jacobian:")
toolbox_JG = geometricJacobian(robot.urdf, robot.config, 'ee')

%% ANAYTICAL JACOBIAN

disp("Analytic expression for the analytical jacobian:")
robot.JA
disp("My analytical jacobian evaluated in the home position:")
vpa(robot.setValues(robot.JA), 2)
disp("Toolbox's analytical jacobian evaluated in the home position:")    
toolbox_JA = jacobian([robot.T(1:3,4); robot.q(1); pi/2 - robot.q(3); pi/2], robot.q);
toolbox_JA = vpa(robot.setValues(toolbox_JA), 2)