function B = inertialMatrix(robot)
    % INERTIALMATRIX Computes the inertial matrix B.
    %   b = INERTIALMATRIX(robot) where robot is the robot parameter struct.
    
    % Set up symbols for the joint variables
    syms d0 a1 d2 a3 q1 q2 q3 h1 a2 h3 c2 b2 r1 r3 m1 m2 m3
    assume([h1 a2 h3], 'real');

    % Compute the positions of the CoMs wrt the base frame
    Ti = myDirectKinematics(robot.DH_table_sym);
    link2_com = Ti(:,:,2)*robot.link2_com';
    link3_com = Ti(:,:,3)*robot.link3_com';
    link4_com = Ti(:,:,4)*robot.link4_com';
    % Compute the partial Jacobians
    [JPi, JOi] = partialJacobians(robot, [link2_com, link3_com, link4_com]);
    % Compute the inertial tensors wrt the link frames
    link2_I = inertialTensor(robot, 1, robot.link2_com);
    link3_I = inertialTensor(robot, 2, robot.link3_com);
    link4_I = inertialTensor(robot, 3, robot.link4_com);
    link_I = cat(3, link2_I, link3_I, link4_I);
    % Get the rotation matrices from direct kinematics
    R = cat(3, Ti(1:3,1:3,2), Ti(1:3,1:3,3), Ti(1:3,1:3,4));
    % B is the global inertial matrix (i.e. B1+B2+B3)
    B = sym(zeros(3));
    % Cycle over all links
    for i = 1:robot.dof
        % Apply the formula
        Bi = robot.link_masses(i)*JPi(:,:,i).'*JPi(:,:,i) + ...
            JOi(:,:,i).'*R(:,:,i)*link_I(:,:,i)*R(:,:,i).'*JOi(:,:,i);
        B = B + Bi;
    end
    % Simplify if possible
    B = simplify(B);
    % Remove small coefficients
    B = subsZero(B);
end