%--------------------------------------------------------------------------
%
% coriolis.m
%
% This script implements the computation of the Coriolis and centrifugal
% acceleration matrix for the manipulator.
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------
function C = coriolis(robot)
    C = sym(zeros(3));
    dB1 = vpa(simplify(diff(robot.B, robot.q(1))*robot.dq(1)),4);
    dB2 = vpa(simplify(diff(robot.B, robot.q(2))*robot.dq(2)),4);
    dB3 = vpa(simplify(diff(robot.B, robot.q(3))*robot.dq(3)),4);
    dB = cat(3, dB1, dB2, dB3);
    for k = 1:robot.dof
        C(1,1) = C(1,1) + 0.5*(dB(1,1,k)+dB(1,k,1)-dB(1,k,1))*robot.dq(k);
    end
    for k = 1:robot.dof
        C(2,2) = C(2,2) + 0.5*(dB(2,2,k)+dB(2,k,2)-dB(2,k,2))*robot.dq(k);
    end
    for k = 1:robot.dof
        C(3,3) = C(3,3) + 0.5*(dB(3,3,k)+dB(3,k,3)-dB(3,k,3))*robot.dq(k);
    end
    for k = 1:robot.dof
        C(1,2) = C(1,2) + 0.5*(dB(1,2,k)+dB(1,k,2)-dB(2,k,1))*robot.dq(k);
    end
    C(2,1) = C(1,2); % Symmetry
    for k = 1:robot.dof
        C(1,3) = C(1,3) + 0.5*(dB(1,3,k)+dB(1,k,3)-dB(3,k,1))*robot.dq(k);
    end
    C(3,1) = C(1,3); % Symmetry
    C = vpa(simplify(C),4);
end