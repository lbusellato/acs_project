function g = gravityVector(robot)
    % GRAVITYVECTOR Computes the gravity term vector.
    %   g = GRAVITYVECTOR(robot) where robot is the robot parameter struct.
    
    % Set up symbols for the joint variables
    syms d0 a1 a3 d2 q1 q2 q3 dq1 dq2 dq3 h1 a2 h3 c2 b2 r1 r3

    % Compute the positions of the CoMs wrt the base frame
    Ti = myDirectKinematics(robot.DH_table_sym);
    link2_com = Ti(1:3,1:3,2)*robot.link2_com.' + Ti(1:3,4,2);
    link3_com = Ti(1:3,1:3,3)*robot.link3_com.' + Ti(1:3,4,3);
    link4_com = Ti(1:3,1:3,4)*robot.link4_com.' + Ti(1:3,4,4);
    % Get the partial Jacobians
    [JPi, ~] = partialJacobians(robot, [link2_com, link3_com, link4_com]);
    % Compute the gravity term
    g = sym(zeros(3,1));
    for i = 1:robot.dof
        gi = 0;
        for j = 1:robot.dof
            gi = gi + robot.link_masses(i)*robot.g0.'*JPi(:,i,j);
        end
        g(i) = -gi;
    end
    % Simplify if possible
    g = simplify(g);
end