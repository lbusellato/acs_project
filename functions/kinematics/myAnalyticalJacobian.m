function JA = myAnalyticalJacobian(robotParams)
    % MYANALYTICALJACOBIAN  Computes the analytical jacobian from direct
    % kinematics.
    %   JA = MYANALYTICALJACOBIAN(robotParams) where robotParams is a 
    % struct containing the robot's DH table.
        
    % Set up symbols for the joint variables
    syms d0 a1 d2 a3 q1(t) q2(t) q3(t) h1 a2 h3 c2 b2 r1 r3 m1 m2 m3 ...
        dq1(t) dq2(t) dq3(t) ddq1(t) ddq2(t) ddq3(t) t
    assume([d0 a1 d2 a3 q1(t) q2(t) q3(t) h1 a2 h3 c2 b2 r1 r3 m1 m2 m3 ...
        dq1(t) dq2(t) dq3(t) ddq1(t) ddq2(t) ddq3(t)], 'real');

    % Compute the direct kinematics from the DH parameters
    [Ti] = myDirectKinematics(robotParams.DH_table_sym);
    % Analytical jacobian - gradients of the positions
    JA = [diff(Ti(1:3,4,4), q1(t)), diff(Ti(1:3,4,4), q2(t)), diff(Ti(1:3,4,4), q3(t))];
    % Add the constant lines relative to angular velocity
    JA = [JA; 1 0 0; 0 0 0; 0 0 1];
end