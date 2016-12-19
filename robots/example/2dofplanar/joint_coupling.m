close all
%robot=load_robot('example', '2dofplanar');

q=[0 0]';
qd=[0 0]';
qdd = [0 6]';
g=[0 -9.81 0]';
fext = [0 0 0 0 0 0]';

%Do not account for friction
robot.dynamics.friction = 0

figure, drawrobot3d(robot, q)

tau = inversedynamic(robot, q, qd, qdd, g, fext)

