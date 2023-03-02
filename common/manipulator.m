%--------------------------------------------------------------------------
%
% manipulator.m
%
% S-Function that implements the manipulator dynamics.
%
% Author: Lorenzo Busellato, VR472249, 2022
%
%--------------------------------------------------------------------------
function [sys,x0,str,ts] = manipulator(t,x,u,flag,robot)
      switch flag,

      %%%%%%%%%%%%%%%%%%
      % Initialization %
      %%%%%%%%%%%%%%%%%%
      case 0,
        [sys,x0,str,ts]=mdlInitializeSizes(robot);

      %%%%%%%%%%%%%%%
      % Derivatives %
      %%%%%%%%%%%%%%%
      case 1,
        sys=mdlDerivatives(t,x,u,robot);

      %%%%%%%%%%%
      % Outputs %
      %%%%%%%%%%%
      case 3,
        sys=mdlOutputs(t,x,u);

      %%%%%%%%%%%%%%%%%%%
      % Unhandled flags %
      %%%%%%%%%%%%%%%%%%%
      case { 2, 4, 9 },
        sys = [];

      %%%%%%%%%%%%%%%%%%%%
      % Unexpected flags %
      %%%%%%%%%%%%%%%%%%%%
      otherwise
        DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

    end
    % end csfunc

    %
    %=============================================================================
    % mdlInitializeSizes
    % Return the sizes, initial conditions, and sample times for the S-function.
    %=============================================================================
    %
    function [sys,x0,str,ts]=mdlInitializeSizes(robot)

        sizes = simsizes;
        sizes.NumContStates  = 2*robot.dof; 
        sizes.NumDiscStates  = 0;
        sizes.NumOutputs     = 2*robot.dof;
        sizes.NumInputs      = robot.dof;
        sizes.DirFeedthrough = 0;
        sizes.NumSampleTimes = 1;

        sys = simsizes(sizes);    
        x0  = [robot.q0; robot.dq0];
        
        str = [];
        ts  = [0 0];
    end
    % end mdlInitializeSizes
    %
    %=============================================================================
    % mdlDerivatives
    % Return the derivatives for the continuous states.
    %=============================================================================
    %
    function sys=mdlDerivatives(t,x,u,robot)
        q = x(1:robot.dof);
        dq = x(robot.dof+1:2*robot.dof);
        tau1 = u(1); tau2 = u(2); tau3 = u(3); 
        q1 = q(1); q2 = q(2); q3 = q(3);
        dq1 = dq(1); dq2 = dq(2); dq3 = dq(3);
        Binv = getBinv(q);
        Cdq = getCdq(q,dq);
        g = getG(q);
        ddq = Binv*(u-Cdq-g);
        sys = [dq; ddq(1); ddq(2); ddq(3)];
    end
    % end mdlDerivatives
    %
    %=============================================================================
    % mdlOutputs
    % Return the block outputs.
    %=============================================================================
    %
    function sys=mdlOutputs(t,x,u)
        sys = x;
    end

    % end mdlOutputs
end