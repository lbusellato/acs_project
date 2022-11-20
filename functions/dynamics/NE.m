function tau = NE(robot, q, dq, ddq, ddP0)

    % Set up symbols for the joint variables
    syms d0 a1 d2 a3 q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3 h1 a2 h3 c2 b2 r1 r3 m1 m2 m3

    % Initial conditions
    fn1 = sym([0 0 0]).';
    mun1 = sym([0 0 0]).';
    z0 = sym([0 0 1]).';
    omega0 = sym([0 0 0]).';
    dOmega0 = sym([0 0 0]).';
    tau = sym([0 0 0]).';
    % Link masses
    m = robot.link_masses;
    % Results of forward algorithm
    omega  = sym(zeros(3));  
    dOmega = sym(zeros(3));  
    ddP    = sym(zeros(3));  
    ddPC   = sym(zeros(3));  
    % Get the transformation matrices from direct kinematics
    Ti = myDirectKinematics(robot.DH_table_sym);
    T0_1 = Ti(:,:,1);
    T1_2 = simplify(inv(Ti(:,:,1))*Ti(:,:,2));
    T2_3 = simplify(inv(Ti(:,:,2))*Ti(:,:,3));
    T3_4 = simplify(inv(Ti(:,:,3))*Ti(:,:,4));
    % Pull the rotation matrices from the transformation matrices
    R0_1 = T0_1(1:3,1:3);
    R1_2 = T1_2(1:3,1:3);
    R2_3 = T2_3(1:3,1:3);
    R3_4 = T3_4(1:3,1:3);
    R = cat(3, R0_1, R1_2, R2_3, R3_4);
    % Pull the translation vectors between the frames from the Ts
    r0_1 = R0_1.'*T0_1(1:3,4);
    r1_2 = R1_2.'*T1_2(1:3,4);
    r2_3 = R2_3.'*T2_3(1:3,4);
    ri = cat(3, r0_1, r1_2, r2_3);
    % Centers of mass wrt the base frame
    link2_com = Ti(1:3,1:3,2)*robot.link2_com' + Ti(1:3,4,2);
    link3_com = Ti(1:3,1:3,3)*robot.link3_com' + Ti(1:3,4,3);
    link4_com = Ti(1:3,1:3,4)*robot.link4_com' + Ti(1:3,4,4);
    rCi = [link2_com, link3_com, link4_com];
    % Initial variables for recursion
    omega_i1 = omega0; % omega_i-1^i-1
    dOmega_i1 = dOmega0; % dOmega_i-1^i-1
    ddP_i1 = ddP0; % ddP_i-1^i-1
    % Forward equations
    for i = 1:robot.dof
        % Check joint type to correctly apply the equations
        if robot.joint_config(i) == 'P' % Prismatic joint
            p = 1; r = 0;
        else % Revolute joint
            p = 0; r = 1;
        end
        % Angular velocity of frame i
        omega(:,i) = R(:,:,i).'*omega_i1 + r*R(:,:,i).'*dq(i)*z0;
        % Angular acceleration of frame i
        dOmega(:,i) = R(:,:,i).'*dOmega_i1 + ...
            r*R(:,:,i).'*(ddq(i)*z0+cross(dq(i)*omega_i1, z0));
        % Linear acceleration of the origin of frame i
        ddP(:,i) = R(:,:,i).'*ddP_i1 + cross(dOmega(:,i),ri(:,:,i)) ...
            + cross(omega(:,i),cross(omega(:,i),ri(:,:,i))) ...
            + p*R(:,:,i).'*ddq(i)*z0+cross(2*dq(i)*omega(:,i), R(:,:,i).'*z0);
        % Linear acceleration of the center of mass Ci
        ddPC(:,i) = ddP(:,i) + cross(dOmega(:,i), rCi(:,i)) ...
            + cross(omega(:,i),cross(omega(:,i),rCi(:,i)));
        % Update for recursion
        omega_i1 = omega(:,i);
        dOmega_i1 = dOmega(:,i);
        ddP_i1 = ddP(:,i);
    end
    % Compute the inertial tensors wrt the link frames
    link2_I = inertialTensor(robot, 1, robot.link2_com);
    link3_I = inertialTensor(robot, 2, robot.link3_com);
    link4_I = inertialTensor(robot, 3, robot.link4_com);
    I = cat(3, link2_I, link3_I, link4_I);
    % Initial variables for recursion
    f_n1 = fn1; % f_n+1^n+1
    mu_n1 = mun1; % mu_n+1^n+1
    % Backward equations
    for i = robot.dof:-1:1
        % Check joint type to correctly apply the equations
        if robot.joint_config(i) == 'P' % Prismatic joint
            p = 1; r = 0;
        else % Revolute joint
            p = 0; r = 1;
        end
        fi = R(:,:,i+1)*f_n1 + m(i)*ddPC(:,i);
        mui = -cross(fi, (ri(:,i)+rCi(:,i))) + R(:,:,i+1)*mu_n1 ...
            + R(:,:,i+1)*cross(f_n1,rCi(:,i)) + I(:,:,i)*dOmega(:,i) ...
            + cross(omega(:,i),I(:,:,i)*omega(:,i));
        tau(:,i) = p*fi.'*R(:,:,i).'*z0 + r*mui.'*R(:,:,i).'*z0;
        % Update for recursion
        f_n1 = fi;
        mu_n1 = mui;
    end
end