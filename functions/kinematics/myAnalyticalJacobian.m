function JA = myAnalyticalJacobian(robotParams)
    % MYANALYTICALJACOBIAN  Computes the analytical jacobian from direct
    % kinematics.
    %   JA = MYANALYTICALJACOBIAN(robotParams) where robotParams is a 
    % struct containing the robot's DH table.
    
    % Set up symbols for the joint variables
    syms d0 a1 d2 a3 q1 q2 q3
    
    % Compute the direct kinematics from the DH parameters
    [Ti] = myDirectKinematics(robotParams.DH_table_sym);
    % Analytical jacobian - gradients of the positions
    JA = [diff(Ti(1:3,4,4), q1), diff(Ti(1:3,4,4), q2), diff(Ti(1:3,4,4), q3)];
    % Add the constant lines relative to angular velocity
    JA = [JA; 1 0 0; 0 0 0; 0 0 1];
end