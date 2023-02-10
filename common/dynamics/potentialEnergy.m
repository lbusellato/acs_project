%--------------------------------------------------------------------------
%
% potentialEnergy.m
%
% This script implements the computation of the potential energy for the 
% manipulator.
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------
function U = potentialEnergy(robot, q)
    U = 0;
    for i = 1:robot.dof
        m = robot.link_mass(i);
        pl = robot.pli(:,i);
        U = U - m*robot.g0.'*pl;
    end
    U = vpa(simplify(U),4);
    if nargin == 2
        U = vpa(subs(U, robot.q, q),4);
    end
end