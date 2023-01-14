close all;
clc;
clearvars;
addpath(genpath('./common/'));
addpath(genpath('./assignments/'));
syms d0 a1 d2 a3 q1 q2 q3 t
% Denavit-Hartenberg parameter table
dh_table = [ 0, 0, d0, 0;
           a1, pi/2, 0, q1;
           0, -pi/2, d2+q2, pi/2;
           a3, 0, 0, q3-pi/2;
           0, pi/2, 0, pi/2];
links = [
    Link("C"),
    Link("P"),
    Link("C")
];
robot = Robot('RPR_zyx.urdf', dh_table, links);