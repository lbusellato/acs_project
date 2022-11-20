function [T] = getT(DH_row)
    % GETT Computes the homogeneous transformation matrix associated to a
    % provided DH table row.
    %   T = GETT(DH_row)
    
    % Set up symbols for the joint variables
    syms d0 a1 d2 a3 q1 q2 q3

    % Pull the values from the DH table
    a     = DH_row(1);
    alpha = DH_row(2);
    d     = DH_row(3);
    theta = DH_row(4);
    % Transformation matrix
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,              sin(alpha),                cos(alpha),               d;
         0,              0,                            0,                     1];
    % Simplify the result if possible
    T = simplify(T);
end