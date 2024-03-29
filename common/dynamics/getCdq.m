%--------------------------------------------------------------------------
%
% getCdq.m
%
% This script computes numerically the velocity product.
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------
function Cdq = getCdq(q,dq)
    q1 = q(1); q2 = q(2); q3 = q(3);
    dq1 = dq(1); dq2 = dq(2); dq3 = dq(3);
    Cdq = [
  [0.5*dq2^2*(3.098*q2 + 0.1962*cos(q3) + 0.7099) - 0.5*dq3^2*(0.0474*sin(2.0*q3) + 0.05885*sin(q3) + 0.1962*q2*sin(q3)), 0.5*dq1*dq2*(3.098*q2 + 0.1962*cos(q3) + 0.7099), 0.03923*cos(q3)*dq3^2 - 0.5*dq1*(0.0474*sin(2.0*q3) + 0.05885*sin(q3) + 0.1962*q2*sin(q3))*dq3]
[                                                                     0.5*dq1*dq2*(3.098*q2 + 0.1962*cos(q3) + 0.7099),                                                0,                                                                                              0]
[                       0.03923*cos(q3)*dq3^2 - 0.5*dq1*(0.0474*sin(2.0*q3) + 0.05885*sin(q3) + 0.1962*q2*sin(q3))*dq3,                                                0,                                                                                              0]
    ]*dq;
end