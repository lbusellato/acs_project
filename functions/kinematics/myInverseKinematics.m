function [q1, q2, q3] = myInverseKinematics(robotParams, p)

    % Set up symbols for the joint variables
    syms d0 a1 d2 a3 q1 q2 q3 h1 a2 h3 c2 b2 r1 r3

    % Compute the direct kinematics from the DH parameters
    [Ti] = myDirectKinematics(robotParams.DH_table_sym);
    % Position of the end-effector
    px = p(1);
    py = p(2);
    pz = p(3) - robotParams.d0;
    % Computation for theta3
    S3 = pz / robotParams.a3;
    C3 = abs(sqrt(1 - S3^2));
    q3 = [atan2(S3, C3), atan2(S3, -C3)];
    % Computation for d2
    q2 = [
        robotParams.a3 * C3 - robotParams.d2 + abs(sqrt(px^2 + py^2 - robotParams.a1^2));
        -robotParams.a3 * C3 - robotParams.d2 + abs(sqrt(px^2 + py^2 - robotParams.a1^2))];
    % Computation for theta1
    C1 = (py * S3 -pz * C3 + (q2(1))*C3 - robotParams.a3)/robotParams.a1;
    S1 = (-py * C3 -pz * S3 + (q2(1) + robotParams.d2)*S3)/robotParams.a1;
    q1 = [atan2(S1, C1), atan2(S1, C1)];
end