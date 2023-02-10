%--------------------------------------------------------------------------
%
% kineticEnergy.m
%
% This script implements the computation of the kinetic energy for the 
% manipulator.
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------
function T = kineticEnergy(robot, q, dq)
    T = vpa(simplify(0.5 * robot.dq.'*robot.B*robot.dq),4);
    if nargin == 3
        T = vpa(subs(T, [robot.q, robot.dq], [q, dq]),4);
    end
end