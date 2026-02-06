function test_kinematics_PRR
robot = load_robot('example', 'PRR');
q = [1 0 pi/2];
drawrobot3d(robot, q);
T = directkinematic(robot, q)

qi = inversekinematic(robot, T);

T1 = directkinematic(robot, qi(:,1))
T2 = directkinematic(robot, qi(:,2))
drawrobot3d(robot, qi(:,1));
drawrobot3d(robot, qi(:,2));