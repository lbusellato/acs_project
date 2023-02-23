%--------------------------------------------------------------------------
%
% RNE.m
%
% This script implements the recursive Newton-Euler formulation for the
% computation of the equations of motion of the manipulator.
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------
function tau = RNE(robot, q, dq, ddq, g0)   
    assume([q,dq,ddq],'real');
    w = sym(zeros(robot.dof, robot.dof+1));
    ddp = sym(zeros(robot.dof, robot.dof+1));
    dw = sym(zeros(robot.dof, robot.dof+1));
    ddpc = sym(zeros(robot.dof, robot.dof+1));
    % Initial conditions
    ddp(:,1) = -g0;
    z0 = [0;0;1];
    % Forward step
    for i = 2:robot.dof + 1
        T = robot.partialT(:,:,i);
        R = T(1:3,1:3);
        r = T(1:3,4);
        rc = robot.pli(:,i-1);
        w(:,i) = R.'*w(:,i-1);
        dw(:,i) = R.'*dw(:,i-1);
        if robot.joint_config(i-1) == 'R' % revolute
            w(:,i) = w(:,i) + R.'*dq(i-1)*z0;
            dw(:,i) = dw(:,i) + R.'*(ddq(i-1)*z0 + cross(dq(i-1)*w(:,i-1),z0));
        end
        ddp(:,i) = R.'*ddp(:,i-1) + cross(dw(:,i), r) + cross(w(:,i), cross(w(:,i),r));
        if robot.joint_config(i-1) == 'P' % prismatic
            ddp(:,i) = ddp(:,i) + R.'*ddq(i-1)*z0 + cross(2*dq(i-1)*w(:,i), R.'*z0);
        end
        ddpc(:,i) = ddp(:,i) + cross(dw(:,i), rc) + cross(w(:,i),cross(w(:,i), rc));
    end
    % Initial conditions
    f = sym(zeros(robot.dof, robot.dof+1));
    mu = sym(zeros(robot.dof, robot.dof+1));
    tau = sym(zeros(robot.dof, 1));
    % Backward step
    for i = robot.dof:-1:1
        T = robot.partialT(:,:,i+2);
        Tm1 = robot.partialT(:,:,i+1);
        R = T(1:3,1:3);
        Rm1 = Tm1(1:3,1:3);
        r = Tm1(1:3,4);
        I = robot.I(:,:,i);
        rc = robot.pli(:,i);
        m = robot.link_mass(i);
        f(:,i) = R*f(:,i+1) + m*ddpc(:,i+1);
        mu(:,i) = -cross(f(:,i), r+rc) + R*mu(:,i+1) + cross(R*f(:,i+1), rc) + ...
            I*dw(:,i+1) + cross(w(:,i+1), I*w(:,i+1));
        if robot.joint_config(i) == 'P' % prismatic
            tau(i) = f(:,i)'*Rm1'*z0;
        else % revolute
            tau(i) = mu(:,i)'*Rm1'*z0;
        end
    end
    tau = simplify(tau);
end