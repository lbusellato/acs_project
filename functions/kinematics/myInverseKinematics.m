function [q1, q2, q3] = myInverseKinematics(robotParams, p, w)
    % MYINVERSEKINEMATICS Computes the direct kinematics from a DH parameter
    % table.
    %   [q1, q2, q3] = MYINVERSEKINEMATICS(robotParams) where robotParams 
    % is a struct containing the robot's parameters, p is the position of
    % the end-effector and w is the position of the joint before the
    % end-effector.

    % Position of the end-effector
    px = p(1);
    py = p(2);
    pz = p(3) - robotParams.d0;
    % Position of the last revolute joint
    wx = w(1);
    wy = w(2);
    % Compute both solutions for q2
    q2p = -robotParams.d2-real(sqrt(wx^2+wy^2-robotParams.a1^2));
    q2n = -robotParams.d2+real(sqrt(wx^2+wy^2-robotParams.a1^2));
    % Pick the solution that respects the joint limits
    if robotParams.jointLimits(2,1) <= q2p && q2p <= robotParams.jointLimits(2,2)
        q2 = q2p;
    else
        q2 = q2n;
    end
    % Compute all four solutions for q2 using q3 and the ee's position
    S3 = pz / robotParams.a3;
    C3 = real(sqrt(1-S3^2));
    theta3 = [atan2(S3, C3), atan2(S3, -C3)];
    d23 = [-robotParams.a3*C3-robotParams.d2+real(sqrt(px^2+py^2-robotParams.a1^2));
           robotParams.a3*C3-robotParams.d2+real(sqrt(px^2+py^2-robotParams.a1^2));
          -robotParams.a3*C3-robotParams.d2-real(sqrt(px^2+py^2-robotParams.a1^2));
           robotParams.a3*C3-robotParams.d2-real(sqrt(px^2+py^2-robotParams.a1^2))];
    % Get the index of the solution closest to the actual value of q2 computed before
    [~,closestIndex] = min(abs(d23 - q2));
    % Choose the angle corresponding to the closest solution as q3
    if closestIndex == 1 || closestIndex == 3
        q3 = theta3(1);
    else
        q3 = theta3(2);
    end
    % Compute q1
    d = q2 + robotParams.d2;
    a1 = robotParams.a1;
    C1 = (a1*wx-d*wy)/(d^2+a1^2);
    S1 = (wx - a1*C1)/d;
    q1 = atan2(S1, C1);
end