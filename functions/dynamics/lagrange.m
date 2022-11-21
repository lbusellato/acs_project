function [eqns, B, C, g] = lagrange(robot)
    % LAGRANGE  Computes the dynamic model of the robot.
    %   [eqns, B, C, g] = LAGRANGE(robot) where robot is a struct 
    % containing the robot's parameters
        
    % Set up symbols for the joint variables
    syms d0 a1 d2 a3 q1(t) q2(t) q3(t) h1 a2 h3 c2 b2 r1 r3 m1 m2 m3 ...
        dq1(t) dq2(t) dq3(t) ddq1(t) ddq2(t) ddq3(t) t
    assume([d0 a1 d2 a3 q1(t) q2(t) q3(t) h1 a2 h3 c2 b2 r1 r3 m1 m2 m3 ...
        dq1(t) dq2(t) dq3(t) ddq1(t) ddq2(t) ddq3(t)], 'real');

    B = inertialMatrix(robot);
    C = christoffel(B);
    g = gravityVector(robot);
    % Construct the equations
    syms tau1 tau2 tau3
    dq = [dq1(t) dq2(t) dq3(t)].';
    ddq = [ddq1(t) ddq2(t) ddq3(t)].';
    tau = [tau1 tau2 tau3].';
    eqns = tau == B*ddq + C*dq + g;
end