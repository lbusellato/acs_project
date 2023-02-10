%--------------------------------------------------------------------------
%
% inertiaMatrix.m
%
% This script implements the computation of the inertia matrix for the 
% manipulator.
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------
function B = inertiaMatrix(robot)
    B = sym(zeros(3));
    for i = 1:robot.dof
        m = robot.link_mass(i);
        JP = robot.JP(:,:,i);
        JO = robot.JO(:,:,i);
        Rbi = robot.T(1:3,1:3,i+1);
        I = robot.I(:,:,i);
        B = vpa(simplify(B + simplify(m*JP.'*JP + JO.'*Rbi*I*Rbi.'*JO)),4);
    end
end