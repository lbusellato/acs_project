function equationsOfMotion(B, C, g)
    
    % Set up symbols for the joint variables
    syms d0 a1 d2 a3 q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3 t1 t2 t3 h1 a2 h3 c2 b2 r1 r3
    assume([h1, a2, h3, c2, b2, r1, r3], 'real');
    q = [q1, q2, q3]';
    dq = [dq1, dq2, dq3]';
    ddq = [ddq1, ddq2, ddq3]';
    tau = [t1, t2, t3]';

    % Construct the equations
    eqns = tau == B*ddq + C*dq
end