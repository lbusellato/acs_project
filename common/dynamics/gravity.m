%--------------------------------------------------------------------------
%
% gravity.m
%
% This script implements the computation of the gravity term for the 
% manipulator.
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------
function G = gravity(robot)
    G = sym(zeros(3,1));
    for i = 1:robot.dof
        g = 0;
        for j = 1:robot.dof
           g = g + robot.link_mass(j)*robot.g0.'*robot.JP(:,j,i);
        end
        G(i) = g;
    end
    G = vpa(simplify(G),4);
end