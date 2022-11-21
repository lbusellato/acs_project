function JG = myGeometricJacobian(robotParams)
    % MYGEOMETRICJACOBIAN  Computes the geometric jacobian from direct
    % kinematics. To keep consistency with the toolbox, the angular and
    % linear velocity rows are swapped.
    %   JG = MYGEOMETRICJACOBIAN(robotParams) where robotParams is a 
    % struct containing the robot's DH table.
    
    % Set up symbols for the joint variables
    syms d0 a1 d2 a3 q1 q2 q3
    
    % Compute the direct kinematics from the DH parameters
    [Ti] = myDirectKinematics(robotParams.DH_table_sym);
    % JG will hold the jacobian matrix
    JG = [];
    % Get the ee's position
    p3 = Ti(1:3,4,4);
    % Iteratively construct the matrix
    for i = 1:robotParams.dof
        % Get the i-th joint's z-axis versor
        zi = Ti(1:3,3,i);
        if robotParams.joint_config(i) == 'R'
            % Apply the formula for a revolute joint
            pi = Ti(1:3,4,i);
            % Compute the skew matrix of zi for the cross product
            Szi = skew(zi); 
            JGi = [Szi*(p3-pi); zi];
            % Add the column to the matrix
            JG = horzcat(JG, JGi);
        else
            % Apply the formula for a prismatic joint
            JGi = [zi; 0; 0; 0];
            % Add the column to the matrix
            JG = horzcat(JG, JGi);
        end
    end
    % Simplify if possible
    JG = simplify(JG);
end