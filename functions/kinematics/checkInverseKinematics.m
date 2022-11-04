function checkInverseKinematics(robot, robotParams, config)
    % CHECKINVERSEKINEMATICS  Computes the inverse kinematics both with the 
    % toolbox and with "manual" computation.
    %   CHECKINVERSEKINEMATICS(robot, robotParams, config) where robot is a 
    % RigidBodyTree model, robotParams is a struct with the robot's 
    % parameters and config is a configuration of the robot.

    % Get the toolbox's transformation matrix from the ee to the base link
    toolbox_base_to_ee = getTransform(robot, config, 'ee', 'base_link');
    ee_pose = toolbox_base_to_ee(1:3,4);
    % Get the toolbox's transformation from the last revolute to the base link
    toolbox_base_to_link4 = getTransform(robot, config, 'Link4', 'base_link');
    link4_pos = toolbox_base_to_link4(1:3,4);
    % Compute inverse kinematics "by hand"
    [q1, q2, q3] = myInverseKinematics(robotParams, ee_pose, link4_pos);
    my_IK = [q1; q2; q3]
    % Set up the toolbox's numerical inverse kinematics solver
    ik = inverseKinematics('RigidBodyTree', robot);
    % Weights for the positions (0.25) and orientations (1)
    weights = [0.25 0.25 0.25 1 1 1];
    % Initial guess for the joint variables
    initialguess = robot.homeConfiguration;
    tform = getTransform(robot, config, 'ee', 'base_link');
    % Estimate joint variables with the toolbox
    [configSoln, solnInfo] = ik('ee', toolbox_base_to_ee, weights, initialguess);
    toolbox_ik = [configSoln(1).JointPosition;
                configSoln(2).JointPosition;
                configSoln(3).JointPosition]
end