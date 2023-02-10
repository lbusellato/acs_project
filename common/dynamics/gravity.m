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
function g = gravity(robot)
    g = sym(zeros(3,1));
    for i = 1:robot.dof
        for j = 1:robot.dof
           m = robot.link_mass(j);
           jp = robot.JP(:,j,i);
           g(i) = g(i) - m*robot.g0.'*jp;
        end
    end
    g = vpa(simplify(g),4);
end