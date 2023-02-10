%--------------------------------------------------------------------------
%
%   myInverseKinematics.m
%
% This script implements the computation of inverse kinematics for the
% manipulator.
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------
function config = myInverseKinematics(robot)
    % Get the toolbox's transformation matrix from the ee to the base link
    toolbox_base_to_ee = getTransform(robot.urdf, robot.config, 'ee', 'base_link');
    ee = toolbox_base_to_ee(1:3,4);
    % Get the toolbox's transformation from the last revolute to the base link
    toolbox_base_to_joint3 = getTransform(robot.urdf, robot.config, 'Link4', 'base_link');
    j3 = toolbox_base_to_joint3(1:3,4);
    d0 = 0.15; a1 = 0.4; d2 = 0.3; a3 = 0.24; % Fixed distances between frames
    px = ee(1); py = ee(2); pz = ee(3) - d0;
    wx = j3(1); wy = j3(2);
    % q2
    q2p = -d2-real(sqrt(wx^2+wy^2-a1^2));
    q2n = -d2+real(sqrt(wx^2+wy^2-a1^2));
    % Pick the solution that respects the joint limits of the prismatic joint
    if q2p >= robot.joint_limits(2,1) && q2p <= robot.joint_limits(2,1)
        q2 = q2p;
    else
        q2 = q2n;
    end
    % Compute all four solutions for q2 using q3 and the ee's position
    S3 = pz / a3;
    C3 = real(sqrt(1-S3^2));
    theta3 = [atan2(S3, C3), atan2(S3, -C3)];
    d23 = [-a3*C3-d2+real(sqrt(px^2+py^2-a1^2));
           a3*C3-d2+real(sqrt(px^2+py^2-a1^2));
          -a3*C3-d2-real(sqrt(px^2+py^2-a1^2));
           a3*C3-d2-real(sqrt(px^2+py^2-a1^2))];
    % Get the index of the solution closest to the actual value of q2 computed before
    [~,closestIndex] = min(abs(d23 - q2));
    % Choose the angle corresponding to the closest solution as q3
    if closestIndex == 1 || closestIndex == 3
        q3 = theta3(1);
    else
        q3 = theta3(2);
    end
    % q1
    d = q2 + d2;
    C1 = (a1*wx - d*wy)/((q2 + d2)^2 + a1^2);
    S1 = (wx - a1*C1)/(q2 + d2);
    q1 = atan2(S1, C1);
    config = [q1, q2, q3];