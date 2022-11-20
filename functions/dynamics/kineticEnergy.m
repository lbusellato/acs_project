function T = kineticEnergy(robot)
    % KINETICENERGY Computes the kinetic energy of the manipulator.
    %   T = KINETICENERGY(robot) where robot is the robot parameter struct.
    
    % Set up symbols for the joint variables
    syms d0 a1 d2 a3 q1(t) q2(t) q3(t) dq1(t) dq2(t) dq3(t) h1 a2 h3 c2 ...
        b2 r1 r3 m1 m2 m3
    dq = [dq1(t); dq2(t); dq3(t)];
    
    % Compute the inertial matrix B
    B = inertialMatrix(robot);
    % Compute the kinetic energy
    T = 0.5*dq.'*B*dq;
    T = simplify(T);
end