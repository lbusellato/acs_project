function [eqns, BA, CA, gA] = lagrangeOperational(robot)
    % LAGRANGEOPERATIONAL  Computes the dynamic model of the robot in the
    % operational space.
    %   [eqns, B, C, g] = LAGRANGEOPERATIONAL(robot) where robot is a struct 
    % containing the robot's parameters
        
    % Set up symbols for the joint variables
    syms d0 a1 d2 a3 q1(t) q2(t) q3(t) h1 a2 h3 c2 b2 r1 r3 m1 m2 m3 ...
        dq1(t) dq2(t) dq3(t) ddq1(t) ddq2(t) ddq3(t) t f1 f2 f3 mu1 mu2 ...
        mu3 fe1 fe2 fe3 mue1 mue2 mue3
    assume([d0 a1 d2 a3 q1(t) q2(t) q3(t) h1 a2 h3 c2 b2 r1 r3 m1 m2 m3 ...
        dq1(t) dq2(t) dq3(t) ddq1(t) ddq2(t) ddq3(t) t f1 f2 f3 mu1 mu2 ...
        mu3 fe1 fe2 fe3 mue1 mue2 mue3], 'real');
    dq = [dq1(t) dq2(t) dq3(t)].';
    ddq = [ddq1(t) ddq2(t) ddq3(t)].';
    
    % Computation of B, C, g, JA, dJA and TA
    [JA, TA] = myAnalyticalJacobian(robot);
    % First derivative of the analytical Jacobian
    dJA = subs(diff(JA), [diff(q1(t), t), diff(q2(t), t), diff(q3(t), t)], [dq1(t), dq2(t), dq3(t)]);
    % Pseudoinverse of the analytical Jacobian
    JA_inv = simplify(pinv(JA));
    % B, c, g
    B = inertialMatrix(robot);
    C = christoffel(B);
    g = gravityVector(robot);
    % Keep only the joint variables as symbolic variables
    JA = simplify(subs(JA, robot.old, [robot.fixed_params,q1(t),q2(t),q3(t)]));
    dJA = simplify(subs(dJA, robot.old, [robot.fixed_params,q1(t),q2(t),q3(t)]));
    TA = simplify(subs(TA, robot.old, [robot.fixed_params,q1(t),q2(t),q3(t)]));
    B = simplify(subs(B, robot.old, [robot.fixed_params,q1(t),q2(t),q3(t)]));
    C = simplify(subs(C, robot.old, [robot.fixed_params,q1(t),q2(t),q3(t)]));
    g = simplify(subs(g, robot.old, [robot.fixed_params,q1(t),q2(t),q3(t)]));
    % Computation of BA, CA, gA and the wrenches in the operational space
    BA = JA_inv.'*B*JA_inv;
    CA = (JA_inv.'*C - BA*dJA) * dq;
    gA = JA_inv.'*g;
    % Wrenches
    h = [f1 f2 f3 mu1 mu2 mu3].'; % EE wrench due to joint torques 
    he = [fe1 fe2 fe3 mue1 mue2 mue3].'; % External wrench
    u = simplify(TA.'*h);
    ue = simplify(TA.'*he);
    % Dynamic model in the operational space
    ddx = simplify(JA*ddq + dJA*dq);
    eqns = BA*ddx + CA + gA == u - ue;
end