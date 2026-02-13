q = [0 0 0];
qd = [0 0 0];
qdd = [0 0 0];
tau1 = inverse_dynamics_example_RRR(q, qd, qdd)
robot = load_robot('example', 'example_RRR');
tau1_ = inversedynamic(robot, q, qd, qdd)


q = [pi/4 pi/4 pi/4];
qd = [1 1 1];
qdd = [0 0 0];
tau2 = inverse_dynamics_example_RRR(q, qd, qdd)
tau2_ = inversedynamic(robot, q, qd, qdd)

q = [pi/4 pi/4 pi/4];
qd = [1 1 1];
qdd = [2 2 2];
tau3 = inverse_dynamics_example_RRR(q, qd, qdd)
tau3_ = inversedynamic(robot, q, qd, qdd)

