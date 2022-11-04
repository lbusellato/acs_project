function checkDirectKinematics(robot, robotParams, config)
    % CHECKDIRECTKINEMATICS  Computes the direct kinematics both with the 
    % toolbox and with "manual" computation.
    %   CHECKDIRECTKINEMATICS(robot, robotParams, config) where robot is a 
    % RigidBodyTree model, robotParams is a struct with the robot's 
    % parameters and config is a configuration of the robot.

    % Set up symbols for the joint variables
    syms d0 a1 a3 d2 q1 q2 q3 dq1 dq2 dq3

    % Get the toolbox's estimation of the position of the end-effector
    [q1_actual,q2_actual,q3_actual] = config.JointPosition;
    toolbox_base_to_ee = getTransform(robot, config, 'ee', 'base_link');
    toolbox_ee_position = toolbox_base_to_ee(1:3,4)
    % Compute the direct kinematics from the DH parameters
    [Ti] = myDirectKinematics(robotParams.DH_table_sym);
    T = Ti(:,:,4);
    % Get "my" direct kinematic estimation of the position of the end-effector
    new = [robotParams.fixed_params, q1_actual, q2_actual, q3_actual];
    my_base_to_ee = double(subs(T, robotParams.old, new));
    my_ee_position = my_base_to_ee(1:3,4)
end