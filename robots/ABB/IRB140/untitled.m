% T01 = dh(robot, [0 0 0 0 0 0], 1)
% T12 = dh(robot, [0 0 0 0 0 0], 2)
% T23 = dh(robot, [0 0 0 0 0 0], 3)
% T34 = dh(robot, [0 0 0 0 0 0], 4)
% T45 = dh(robot, [0 0 0 0 0 0], 5)
% T56 = dh(robot, [0 0 0 0 0 0], 6)

% move last three joints
robot = load_robot('ABB', 'IRB140');
robot.graphical.draw_transparent = 1;
pause
j = 0;
p06 = [];
p04 = [];
for i=1:30
   q = [pi/4, pi/8, pi/8, j, -j, -j];
   T = directkinematic(robot, q);
   p06 = [p06 T(1:3, 4)];
   drawrobot3d(robot, q)
   
   T01 = dh(robot, q, 1);
   T12 = dh(robot, q, 2);
   T23 = dh(robot, q, 3);
   T34 = dh(robot, q, 4);
   T04 = T01*T12*T23*T34;
   p04 = [p04 T04(1:3, 4)];
   j=j+0.05;
end
drawrobot3d(robot, q)
plot3(p06(1,:), p06(2,:), p06(3,:), '.')
plot3(p04(1,:), p04(2,:), p04(3,:), '*')
figure, hold
plot3(p06(1,:), p06(2,:), p06(3,:), '.')
plot3(p04(1,:), p04(2,:), p04(3,:), '*')


