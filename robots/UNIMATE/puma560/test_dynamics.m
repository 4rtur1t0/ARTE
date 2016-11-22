
robot=load_robot('UNIMATE', 'puma560');
close all

qz = [0 0 0 0 0 0];
qr = [0 pi/2 -pi/2 0 0 0];
qs = [0 0 -pi/2 0 0 0];
g=[0 0 -9.81]';
fext = [15 15 15 15 15 15]';

figure, drawrobot3d(robot, qz)
tauz2 = inversedynamic(robot, qz, 1*ones(1,6), 1*ones(1,6), g, fext)
taur2 = inversedynamic(robot, qr, 1*ones(1,6), 1*ones(1,6), g, fext)
taus2 = inversedynamic(robot, qs, 1*ones(1,6), 1*ones(1,6), g, fext)

