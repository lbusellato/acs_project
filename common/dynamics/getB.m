%--------------------------------------------------------------------------
%
% getB.m
%
% This script computes numerically the inertia matrix.
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------
function B = getB(q, error)
    q1 = q(1); q2 = q(2); q3 = q(3);
    if error == 0
        B=[ [0.7099*q2 + 0.05885*cos(q3) + 0.0474*cos(q3)^2 + 0.1962*q2*cos(q3) + 1.549*q2^2 + 0.496,          -0.6196,  0.03923*sin(q3)]
            [                                                                                -0.6196,            1.549, -0.09808*sin(q3)]
            [                                                                        0.03923*sin(q3), -0.09808*sin(q3),          0.04757]];             
    else
        B=[ [0.57*q2 + 0.0504*cos(q3) + 0.0406*cos(q3)^2 + 0.168*q2*cos(q3) + 1.2*q2^2 + 0.3857,          -0.48, 0.0336*sin(q3)]
            [                                                                             -0.48,            1.2, -0.084*sin(q3)]
            [                                                                    0.0336*sin(q3), -0.084*sin(q3),        0.04074]];
    end
end