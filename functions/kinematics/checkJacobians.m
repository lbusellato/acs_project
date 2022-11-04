function checkJacobians(robot, robotParams, config)
    % CHECKJACOBIANS  Computes the analytical and geometrical jacobian 
    % matrices both with the toolbox and with "manual" computation.
    %   CHECKJACOBIANS(robot, robotParams, config) where robot is a 
    % RigidBodyTree model, robotParams is a struct with the robot's 
    % parameters and config is a configuration of the robot.

    % Set up symbols for the joint variables
    syms d0 a1 d2 a3 q1 q2 q3
    
    % Compute the direct kinematics from the DH parameters
    [Ti] = myDirectKinematics(robotParams.DH_table_sym);
    % Actual values for joint variables
    new = [robotParams.fixed_params, config.JointPosition];
    % "My" analytical jacobian
    my_JA = myAnalyticalJacobian(robotParams);
    % Compare my analytical Jacobian with the toolbox's in the current config
    toolbox_JA = vpa(subs(jacobian(Ti(1:3,4,4), [q1, q2, q3]), robotParams.old, new), 4);
    % Add the constant lines relative to angular velocity
    toolbox_JA = [toolbox_JA; 1 0 0; 0 0 0; 0 0 1]
    my_JA = vpa(subs(my_JA, robotParams.old, new), 4)

    % "My" geometric jacobian
    my_JG = myGeometricJacobian(robotParams);
    % Compare my geometric Jacobian with the toolbox's in the current config
    toolbox_JG = geometricJacobian(robot, config, 'ee')
    my_JG = vpa(subs(my_JG, robotParams.old, new), 4)
end