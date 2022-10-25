function [Ti] = myDirectKinematics(DH_table)
    % MYDIRECTKINEMATICS Computes the direct kinematics from a DH parameter
    % table.
    %   Ti = MYDIRECTKINEMATICS(robotParams) where robotParams is a 
    % struct containing the robot's DH table. Ti is a vector of
    % transformation matrices.
    
    % Set up symbols for the joint variables
    syms d0 a1 d2 a3 q1 q2 q3

    % Pull each column from the DH table
    a     = DH_table(:, 1);
    alpha = DH_table(:, 2);
    d     = DH_table(:, 3);
    theta = DH_table(:, 4);
    % T will hold the partial transformation matrix, while Ti holds all of them
    T = sym(eye(4));
    Ti = sym(cat(3, eye(4), eye(4), eye(4)));

    for i = 1:size(alpha, 1)
        % Partial transformation matrix
        t = [cos(theta(i)), -sin(theta(i))*cos(alpha(i)),  sin(theta(i))*sin(alpha(i)), a(i)*cos(theta(i));
             sin(theta(i)),  cos(theta(i))*cos(alpha(i)), -cos(theta(i))*sin(alpha(i)), a(i)*sin(theta(i));
             0,              sin(alpha(i)),                cos(alpha(i)),               d(i);
             0,              0,                            0,                           1];
        % Update the global transformation matrix
        T = T * simplify(t);
        % Keep track of the partial matrix
        Ti(:,:,i) = T;
    end    
    % Simplify the result if possible
    Ti = simplify(Ti);
end