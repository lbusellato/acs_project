function g = gravityVector(robot)
    % GRAVITYVECTOR Computes the gravity term vector.
    %   g = GRAVITYVECTOR(robot) where robot is the robot parameter struct.
    
    % Set up symbols for the joint variables
    syms d0 a1 a3 d2 q1 q2 q3 dq1 dq2 dq3 h1 a2 h3 c2 b2 r1 r3

    % Get the partial Jacobians
    [JPi, ~] = partialJacobians(robot);
    % Compute the gravity term
    g = sym(zeros(3,1));
    for i = 1:robot.dof
        for j = 1:robot.dof
            g(i) = g(i) - robot.link_masses(i)*robot.g0.'*JPi(:,i,j);
        end
    end
    % Simplify if possible
    g = simplify(g);
end