

close all

q=[0]';
qd=[0]';
qdd=[0]';
g=[0 -9.81 0]';
%fext = [1 2 3 3 2 1]';
fext = [0 0 0 0 0 0]';

figure, drawrobot3d(robot, q)
%robot=load_robot('example', '2dofplanar');
tau1 = inversedynamic(robot, q, qd, qdd, g, fext)
tau2 = inversedynamics_1dofplanar(robot, q, qd, qdd, norm(g), fext)



tau1-tau2