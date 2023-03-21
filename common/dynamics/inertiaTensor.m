%--------------------------------------------------------------------------
%
% inertiaTensor.m
%
% This script implements the computation of the inertia tensor for a given
% robot link, applying Huygens-Steiner's theorem.
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------
function I = inertiaTensor(robot, link)
    % Inertia wrt CoM
    m = robot.link_mass(link);
    if robot.link_shape(link) == 'C' % cylinder
        a = robot.link_dim(link,2);
        b = robot.link_dim(link,3);
        h = robot.link_dim(link,1);
        Ic = 0.5*m*diag([(a^2+b^2), 3*(a^2+b^2)^2+h^2, 3*(a^2+b^2)^2+h^2]);
        r = [h/2 0 0]';
    else % prism
        a = robot.link_dim(link,1);
        b = robot.link_dim(link,2);
        c = robot.link_dim(link,3);
        Ic = m*diag([(b^2+c^2), (a^2+c^2), (a^2+b^2)])/12;
        r = [a/2 0 0]';
    end
    % Huygens-Steiner
    I = Ic + m*(r'*r*eye(3) - r*r');
end