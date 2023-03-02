%--------------------------------------------------------------------------
%
% DgeometricJacobian.m
%
% This script computes the first derivative of the robot's analytical
% Jacobian.
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------
function dJG = DgeometricJacobian(robot)
    JG = robot.JG;
    % Introduce time dependency
    syms q1(t) q2(t) q3(t) t
    dJG = subs(JG, robot.q, [q1(t) q2(t) q3(t)].');
    % Take the derivative
    dJG = diff(dJG, t);
    % Replace the diffs with the actual symbols
    dJG = subs(dJG, [diff(q1(t),t) diff(q2(t),t) diff(q3(t),t)], [robot.dq(1) robot.dq(2) robot.dq(3)]);
    dJG = simplify(dJG);
end