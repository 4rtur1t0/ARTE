%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   KUKA KR 16 2.
%
%   Author:  
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function robot = parameters()


robot.name= 'KR16_2';

robot.DH.theta= '[  q(1)  q(2)-pi/2     q(3)    q(4)    q(5)   q(6)]';
robot.DH.d='[       0.675     0           0       0.670   0    0.115]';
robot.DH.a='[       0.260   0.680         -0.035    0       0      0]';
robot.DH.alpha= '[  -pi/2   0           -pi/2    pi/2    -pi/2   0]';
robot.J=[];


robot.inversekinematic_fn = 'inversekinematic_kuka_kr16_2(robot, T)';

%number of degrees of freedom
robot.DOF = 6;

%rotational: 0, translational: 1
robot.kind=['R' 'R' 'R' 'R' 'R' 'R'];

%minimum and maximum rotation angle in rad
robot.maxangle =[deg2rad(-185) deg2rad(185); %Axis 1, minimum, maximum
                deg2rad(-65) deg2rad(125); %Axis 2, minimum, maximum
                deg2rad(-220) deg2rad(64); %Axis 3
                deg2rad(-350) deg2rad(350); %Axis 4: Unlimited (400� default)
                deg2rad(-130) deg2rad(130); %Axis 5
                deg2rad(-350) deg2rad(350)]; %Axis 6: Unlimited (800� default)

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [deg2rad(156); %Axis 1, rad/s
                deg2rad(156); %Axis 2, rad/s
                deg2rad(156); %Axis 3, rad/s
                deg2rad(330); %Axis 4, rad/s
                deg2rad(330); %Axis 5, rad/s
                deg2rad(615)];%Axis 6, rad/s
% end effectors maximum velocity
robot.linear_velmax = 1.0; %m/s, not specified

robot.accelmax=robot.velmax/0.1; % 0.1 is here an acceleration time
%base reference system
robot.T0 = eye(4);

%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
robot=init_sim_variables(robot);
robot.path = pwd;

% GRAPHICS
robot.graphical.has_graphics=1;
robot.graphical.color = [25 20 40];
%for transparency
robot.graphical.draw_transparent=0;
%draw DH systems
robot.graphical.draw_axes=1;
%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
robot.graphical.axes_scale=1;
%adjust for a default view of the robot
robot.axis=[-1.5 1.5 -1.5 1.5 0 2];
%read graphics files
robot = read_graphics(robot);

%DYNAMICS
robot.has_dynamics=1;

%consider friction in the computations
robot.dynamics.friction=0;

%link masses (kg)
robot.dynamics.masses=[94 47 35.25 23.5 23.5 11.75];

%COM of each link with respect to own reference system
robot.dynamics.r_com=[0       0          0; %(rx, ry, rz) link 0
                       -0.35	0.252	 0; %(rx, ry, rz) link 1
                       -0.675	   0	 -0.225;  %(rx, ry, rz) link 2
                        0       0       0.150;%(rx, ry, rz) link 4
                        0       0           0;%(rx, ry, rz) link 5
                        0       0         0.1075];%(rx, ry, rz) link 6

%Inertia matrices of each link with respect to its D-H reference system.
% Ixx   Iyy	Izz	Ixy	Iyz	Ixz, for each row
robot.dynamics.Inertia=[0      0.35	0   	0	0	0;
                        9.89     9.89	16	    0	0	0;
                        38.78	38.78	2.56	0	0	0;
                        34.2	34.2	0.9	    0	0	0;
                        0.214	0.214	0.162	0	0	0;
                        0.0356	0.0356	0.07	0	0	0];



robot.motors=load_motors([2 4 3 1 3 1]);
%Speed reductor at each joint
robot.motors.G=[600 600 600 100 100 50];
