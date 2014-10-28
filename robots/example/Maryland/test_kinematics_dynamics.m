%Test kinematics

%Test robot at this position
px=0.0;
py=0.0;
pz=0.6;
%pz=1;


T=eye(4);
T(1,4)=px;
T(2,4)=py;
T(3,4)=pz;


%call inverse kinematics in position
q=inversekinematic(robot, T);
robot.robot1.graphical.draw_transparent=1;
robot.robot2.graphical.draw_transparent=1;
robot.robot3.graphical.draw_transparent=1;

%draw the robot!
drawrobot3d(robot, q(:,8))


%define position as computed by the inverse kinematic function. Only active
%joints
th1=q(1,8);
th2=q(4,8);
th3=q(7,8);

%Joint speeds
th1d=0;
th2d=0;
th3d=0;

%Joint accelerations
th1dd=0;
th2dd=0;
th3dd=0;

% This inverse dynamic model uses a Lagrangian formulation
% thus, we need to use a different approach to define the kinematic state
% of the arm, we first solve the directkinematic problem in position

% Next, we solve it for speed and acceleration
% Two different approaches are here valid. Consider first that we want the
% mechanism to be placed at q1, q2 and q3 known, and we want each joint to
% accelerate at a rate qdd1, qdd2, qdd3. The Lagrangian formulation is
% based on a redundant set of coordinates, thus, we need to specify the
% position px, py, pz of the end effector. Also, the acceleration of the
% end effector pddx pddy pddz must be specified
%[T, pxyzd, pxyzdd]=directkinematic_Maryland3DOF(robot, q(:,8), qd, qdd)
[T, pxyzd, pxyzdd]=directkinematic_Maryland3DOF(robot, [th1 0 0 th2 0 0 th3 0 0], [th1d 0 0 th2d 0 0 th3d 0 0], [th1dd 0 0 th2dd 0 0 th3dd 0 0]);

%We then use pxdd, pydd, pzdd, th1, th2, th3, th1dd,th2dd,th3dd 
%tau=inversedynamic_Maryland3DOF(robot, [px, py, pz, th1 th2 th3], [0 0 0 0 0 0 0 0 0], [0 0 0 0 0 0 0 0 0], [0 0 0])
% You can also specify a set of external forces applied at the end
% effector. These forces are specified in the base reference system.
% Typically, the force exerted by a load should be specified as -m*g being
% g=9.81 m/s^2. For example fext = [0 0 -10*9.81] for a 10kg load placed on
% the end effector's centre.
%
% gc=9.81 if the robot is placed upwards
% gc=-9.81 if the robot is hanging.
%
% Consider a load of 10kg placed on the end effector. Write:
%   fext = [0 0 -10*9.81] if the robot is placed upwards, or
%   fext = [0 0 10*9.81] if the robot is hanging.

%Example 1: robot placed upwards
gc = 9.81;
fext = [0 0 -10*9.81];
tau=inversedynamic_Maryland3DOF(robot, [px, py, pz, th1 th2 th3], [pxyzdd(1) pxyzdd(2) pxyzdd(3) th1dd th2dd th3dd], gc, fext)

%Example 2: robot hanging
gc = -9.81;
fext = [0 0 10*9.81];
tau=inversedynamic_Maryland3DOF(robot, [px, py, pz, th1 th2 th3], [pxyzdd(1) pxyzdd(2) pxyzdd(3) th1dd th2dd th3dd], gc, fext)

