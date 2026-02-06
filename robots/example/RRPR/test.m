
robot = load_robot('example', 'RRPR');
% q = [0 0 1 0];
% T1 = directkinematic(robot, q)
% q = [pi/2 0 1 pi/2];
% T2 = directkinematic(robot, q)
% q = [pi/2 pi/2 1 -pi/2];
% T3 = directkinematic(robot, q)
q = [-pi/4 -pi/4 1 pi/4]
T = directkinematic(robot, q)
qi = inversekinematic(robot, T)

T1 = directkinematic(robot, qi(:,1))
T2 = directkinematic(robot, qi(:,2))
% 
% q = [0 0 0.5 0];
% A01 = dh(robot, q, 1)
% A12 = dh(robot, q, 2)
% A23 = dh(robot, q, 3)
% A34 = dh(robot, q, 4)
% 
% A01
% A02 = A01*A12
% A03 = A02*A23
% A04 = A03*A34
% 
% q = [pi/4 pi/4 0.5 pi/4];
% drawrobot3d(robot, q)