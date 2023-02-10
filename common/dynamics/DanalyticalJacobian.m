function dJA = DanalyticalJacobian(robot)
    JA = robot.JA;
    % Introduce time dependency
    syms q1(t) q2(t) q3(t) t
    dJA = subs(JA, robot.q, [q1(t) q2(t) q3(t)].');
    % Take the derivative
    dJA = diff(dJA, t);
    % Replace the diffs with the actual symbols
    dJA = subs(dJA, [diff(q1(t),t) diff(q2(t),t) diff(q3(t),t)], [robot.dq(1) robot.dq(2) robot.dq(3)]);
    dJA = simplify(dJA);
end