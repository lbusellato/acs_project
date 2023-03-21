config = randomConfiguration(robot.urdf)
B = vpa(subs(robot.B, robot.q, [config.JointPosition]'))
eig(B)