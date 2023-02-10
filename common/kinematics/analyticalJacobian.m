%--------------------------------------------------------------------------
%
%   analyticalJacobian.m
%
% This script implements the computation of the analytical Jacobian matrix
% for the manipulator.
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------
function JA = analyticalJacobian(robot)
    % Compute JA with the transformation matrix TA
    JA = simplify(pinv(robot.Ta)*robot.JG);
end