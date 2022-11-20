function I = inertialTensor(robot, link, r)
    % INERTIALTENSOR  Computes the inertial tensor wrt the base frame of
    % the specified link.
    %   i = INERTIALTENSOR(robot, link, r) where robot is the struct of
    %   robot parameters, link is the desired link (i-1 wrt the toolbox
    %   index) and r is the vector from the origin of frame i-1 and the
    %   origin of the link frame i.
    
    syms d0 a1 d2 a3 q1 q2 q3 h1 a2 h3 c2 b2 r1 r3
    % Inertia matrix wrt com frame
    if robot.link_config(link) == 'C' % Cylinder
        Ic = diag([robot.link_masses(link)*(3*robot.link_geometry(link,2)^2+robot.link_geometry(link,1)^2)/12, 
                  robot.link_masses(link)*(3*robot.link_geometry(link,2)^2+robot.link_geometry(link,1)^2)/12,
                  0.5*robot.link_masses(link)*robot.link_geometry(link,2)^2]);
    else % Prism
        Ic = diag([robot.link_masses(link)*(robot.link_geometry(link,1)^2+robot.link_geometry(link,2)^2)/12,
                   robot.link_masses(link)*(robot.link_geometry(link,3)^2+robot.link_geometry(link,1)^2)/12,
                   robot.link_masses(link)*(robot.link_geometry(link,3)^2+robot.link_geometry(link,2)^2)/12]);
    end
    % Inertial tensor wrt link frame
    S = skew(r);
    % Huygens-Steiner theorem
    I = Ic + robot.link_masses(link)*S.'*S;
    % Simplify if possible
    I = simplify(I);
end