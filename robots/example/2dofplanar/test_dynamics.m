

close all

q=[pi/2 pi/2]';
qd=[10 10]';
qdd = [10 10]';
g=[0 -9.81 0]';
fext = [1 2 3 3 2 1]';

figure, drawrobot3d(robot, q)
%robot=load_robot('example', '2dofplanar');
tau1 = inversedynamic(robot, q, qd, qdd, g, fext)
tau2 = inversedynamics_2dofplanar(robot, q, qd, qdd, norm(g), fext)

tau1-tau2