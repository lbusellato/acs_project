function U = potentialEnergy(robot)
    % POTENTIALENERGY Computes the kinetic energy of the manipulator.
    %   U = POTENTIALENERGY(robot) where robot is the robot parameter struct.
    
    % Set up symbols for the joint variables
    syms d0 a1 a3 d2 q1 q2 q3 h1 a2 h3 c2 b2 r1 r3

    % Compute the positions of the CoMs wrt the base frame
    Ti = myDirectKinematics(robot.DH_table_sym);
    link2_com = Ti(1:3,1:3,2)*robot.link2_com.' + Ti(1:3,4,2);
    link3_com = Ti(1:3,1:3,3)*robot.link3_com.' + Ti(1:3,4,3);
    link4_com = Ti(1:3,1:3,4)*robot.link4_com.' + Ti(1:3,4,4);
    coms = [link2_com, link3_com, link4_com];
    % The total potential energy is the sum of each link's potential energy
    U = 0;
    for i = 1:robot.dof
        U = U - robot.link_masses(i)*robot.g0.'*coms(1:3, i);
    end
    % Simplify if possible
    U = simplify(U);
end