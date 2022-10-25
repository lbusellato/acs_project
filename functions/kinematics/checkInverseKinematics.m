function checkDirectKinematics(robot, config)

    % Set up the toolbox's numerical inverse kinematics solver
    ik = inverseKinematics('RigidBodyTree', robot);
    % Weights for the positions (0.25) and orientations (1)
    weights = [0.25 0.25 0.25 1 1 1];
    % Initial guess for the joint variables
    initialguess = robot.homeConfiguration;
    % Get the toolbox's transformation matrix from the ee to the base link
    tform = getTransform(robot, config, 'ee', 'base_link');
    % Estimate joint variables with the toolbox
    [configSoln, solnInfo] = ik('ee', tform, weights, initialguess);
    toolbox_ik = [configSoln(1).JointPosition;
                configSoln(2).JointPosition;
                configSoln(3).JointPosition]
end