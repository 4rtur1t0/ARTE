%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   KUKA KR 30 3.
%
%   Author:  
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function robot = parameters()


robot.name= 'KR30_3';

robot.DH.theta= '[  q(1)  q(2)-pi/2     q(3)    q(4)    q(5)   q(6)]';
robot.DH.d='[       0.815     0           0       0.820   0    0.17]';
robot.DH.a='[       0.350   0.850         0.145    0       0      0]';
robot.DH.alpha= '[  -pi/2   0           -pi/2    pi/2    -pi/2   0]';
robot.J=[];


robot.inversekinematic_fn = 'inversekinematic_kuka_kr303(robot, T)';

%number of degrees of freedom
robot.DOF = 6;

%rotational: 0, translational: 1
robot.kind=['R' 'R' 'R' 'R' 'R' 'R'];

%minimum and maximum rotation angle in rad
robot.maxangle =[deg2rad(-185) deg2rad(185); %Axis 1, minimum, maximum
                deg2rad(-35) deg2rad(135); %Axis 2, minimum, maximum
                deg2rad(-120) deg2rad(158); %Axis 3
                deg2rad(-350) deg2rad(350); %Axis 4: Unlimited (400� default)
                deg2rad(-119) deg2rad(119); %Axis 5
                deg2rad(-350) deg2rad(350)]; %Axis 6: Unlimited (800� default)

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [deg2rad(140); %Axis 1, rad/s
                deg2rad(126); %Axis 2, rad/s
                deg2rad(140); %Axis 3, rad/s
                deg2rad(260); %Axis 4, rad/s
                deg2rad(245); %Axis 5, rad/s
                deg2rad(322)];%Axis 6, rad/s
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
robot.graphical.color = [255 102 51]./255;
%for transparency
robot.graphical.draw_transparent=0;
%draw DH systems
robot.graphical.draw_axes=0;
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
robot.dynamics.masses=[266 133 99.75 66.5 66.5 33.25];

%COM of each link with respect to own reference system
robot.dynamics.r_com=[0       0          0; %(rx, ry, rz) link 0
                       -0.35	0.252	 0; %(rx, ry, rz) link 1
                       -0.675	   0	 -0.225;  %(rx, ry, rz) link 2
                        0       0       0.150;%(rx, ry, rz) link 4
                        0       0           0;%(rx, ry, rz) link 5
                        0       0         0.1075];%(rx, ry, rz) link 6

%Inertia matrices of each link with respect to its D-H reference system.
% Ixx   Iyy	Izz	Ixy	Iyz	Ixz, for each row
% % % % % robot.dynamics.Inertia=[0      0.35	0   	0	0	0;
% % % % %                         9.89     9.89	16	    0	0	0;
% % % % %                         38.78	38.78	2.56	0	0	0;
% % % % %                         34.2	34.2	0.9	    0	0	0;
% % % % %                         0.214	0.214	0.162	0	0	0;
% % % % %                         0.0356	0.0356	0.07	0	0	0];

robot.dynamics.Inertia=[0      0.35	0   	0	0	0;
    .13     .524	.539	0	0	0;
    .066	.086	.0125	0	0	0;
    1.8e-3	1.3e-3	1.8e-3	0	0	0;
    .3e-3	.4e-3	.3e-3	0	0	0;
    .15e-3	.15e-3	.04e-3	0	0	0];


robot.motors=load_motors([2 4 3 1 3 1]);
%Speed reductor at each joint
robot.motors.G=[600 600 600 100 100 50];
