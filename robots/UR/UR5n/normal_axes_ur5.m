
close all

robot = load_robot('UR', 'UR5n');
q = pi/8*[1 1 1 1 1 1]';

A01 = dh(robot, q,  1)
A12 = dh(robot, q,  2)
A23 = dh(robot, q,  3)
A34 = dh(robot, q,  4)
A45 = dh(robot, q,  5)
A56 = dh(robot, q,  6)

A03 = A01*A12*A23;
A04 = A03*A34;
A05 = A04*A45;

% remove position
A03(1:3, 4) = [0 0 0]';
A04(1:3, 4) = [0 0 0]';
A05(1:3, 4) = [0 0 0]';

z3 = A03(1:3, 3);
z4 = A04(1:3, 3);
y5 = A05(1:3, 2);
z5 = A05(1:3, 3);

%figure, hold on
close all
h=figure(configuration.figure.robot);
hold on, grid
draw_axes(A03, 'X_3', 'Y_3', 'Z_3', 1)
draw_axes(A04, 'X_4', 'Y_4', 'Z_4', 1)
draw_axes(A05, 'X_5', 'Y_5', 'Z_5', 1)

%drawrobot3d(robot, q0)
z4u = cross(z3, z5);
z4u = z4u/norm(z4u);
z4-z4u
z4+z4u
y5-z4
y5+z4

