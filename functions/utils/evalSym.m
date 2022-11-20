function out = evalSym(in, robot, config)
    % EVALSYM Evaluates a symbolic expression.
    %   out = EVALSYM(in, robot, config) where in is the symbolic
    %   expression, robot is the robot parameter struct and config is the
    %   current configuration of the robot.
    
    if class(config) == 'struct'
        config = [config.JointPosition];
    end
    out = double(vpa(subs(in, robot.old, [robot.fixed_params, config]), 4));
end