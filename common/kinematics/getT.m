%--------------------------------------------------------------------------
%
%   getT.m
%
% This script computes the homogeneous transformation matrix between frame
% i and frame i-1.
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------
function T = getT(robot, i)
    a = robot.dh_table(i,1);
    alpha = robot.dh_table(i,2);
    d = robot.dh_table(i,3);
    theta = robot.dh_table(i,4);
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
             sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
             0,              sin(alpha),                cos(alpha),               d;
             0,              0,                            0,                1];
end