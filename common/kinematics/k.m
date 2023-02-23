%--------------------------------------------------------------------------
%
%   k.m
%
% This script computes the 3D pose from a joint configuration by applying
% direct kinematics.
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------
function x = k(q)
    q1 = q(1); q2 = q(2); q3 = q(3);
    % Direct kinematics
    x = [(2*cos(q1))/5 + (3*sin(q1))/10 + (6*cos(q3)*sin(q1))/25 + q2*sin(q1);
         (2*sin(q1))/5 - (3*cos(q1))/10 - (6*cos(q1)*cos(q3))/25 - q2*cos(q1);
         (6*sin(q3))/25 + 3/20;
         q1;
         pi/2 - q3;
         pi/2];
end