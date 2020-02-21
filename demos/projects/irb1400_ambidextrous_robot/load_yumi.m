
%For this special case, the mechanism is divided into two different serial
%arms, Namely 2 two dof planar arms.
%Thus, the robot structure contains two different arms named robot1 and
%robot2

robot1=load_robot('ABB','IRB14000\right');
robot2=load_robot('ABB','IRB14000\left');
cd ..

robot=[];
robot.name='IRB14000';
robot.robot1=robot1;
robot.robot2=robot2;
robot.nserial=2; %number of serial links connecting base and end effector



%drawrobot3d(robot.robot1, [0 0 0 0 0 0 0])
%drawrobot3d(robot.robot2, [0 0 0 0 0 0 0], 1)

%Function name to compute inverse kinematic
%robot.inversekinematic_fn = 'inversekinematic_irb14000(robot, T)';
%robot.directkinematic_fn = 'directkinematic(robot, q)';


