%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   FANUC LR MATE 10iA, FANUC Robotics Europe.
%   Author: David García Muñoz, Néstor Gómez López, Teresa Pomares
%   Palorames.
%   Universidad Miguel Hernandez de Elche. 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function robot = parameters()


robot.name= 'Fanuc_Lr_Mate_10iA';

%Path where everything is stored for this robot
robot.path = 'robots/fanuc/LR_MATE_10iA';
%Tabla de D-H
robot.DH.theta= '[  q(1)  q(2)-pi/2       q(3)     q(4)     q(5)     q(6)]';
robot.DH.d='[       0.450     0           0        0.600    0        0.1]';
robot.DH.a='[       0.150   0.600         0.2       0        0        0]';
robot.DH.alpha='[   -pi/2      0          -pi/2     pi/2    -pi/2     0]';
robot.J=[];


robot.inversekinematic_fn = 'inversekinematic_fanuc_mate(robot, T)';
robot.directkinematic_fn = 'directkinematic(robot, q)';

%number of degrees of freedom
robot.DOF = 6;

%rotational: 0, translational: 1
robot.kind=['R' 'R' 'R' 'R' 'R' 'R'];

%minimum and maximum rotation angle in rad
robot.maxangle =[deg2rad(-340) deg2rad(340); %Axis 1, minimum, maximum
                deg2rad(-250) deg2rad(250); %Axis 2, minimum, maximum
                deg2rad(-447) deg2rad(447); %Axis 3
                deg2rad(-380) deg2rad(380); %Axis 4: Unlimited (400� default)
                deg2rad(-280) deg2rad(280); %Axis 5
                deg2rad(-540) deg2rad(540)]; %Axis 6: Unlimited (800� default)

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [deg2rad(230); %Axis 1, rad/s
                deg2rad(225); %Axis 2, rad/s
                deg2rad(230); %Axis 3, rad/s
                deg2rad(430); %Axis 4, rad/s
                deg2rad(430); %Axis 5, rad/s
                deg2rad(630)];%Axis 6, rad/s
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
robot.graphical.color = [153 255 0];%./255;
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%DYNAMIC PARAMETERS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robot.has_dynamics=1;


%consider friction in the computations
robot.dynamics.friction=0;

%link masses (kg)
robot.dynamics.masses=[0 32.5 26 7.8 19.5 5.2];

%COM of each link with respect to own reference system
robot.dynamics.r_com=[ 0       0          0; %(rx, ry, rz) link 1
                      -0.1     0          0; %(rx, ry, rz) link 2
                      -0.3     0          0.05; %(rx, ry, rz) link 3
                       0       0          0; %(rx, ry, rz) link 4
                       0       0.2        0; %(rx, ry, rz) link 5
                       0       0          0]; %(rx, ry, rz) link 6

%Inertia matrices of each link with respect to its D-H reference system.
% Ixx	Iyy	Izz	Ixy	Iyz	Ixz, for each row
robot.dynamics.Inertia=[0      0.41	0   	0	0	0;
                        0.33    0.424	0.539	0	0	0;
                        0.056	0.096	0.012	0	0	0;
                        0.0018	0.0013	0.00179	0	0	0;
                        3e-4	3.99e-4	3e-4	0	0	0;
                        1.5e-4	1.49e-4	4e-5	0	0	0];


%Please note that we are simulating the motors as presented in MAXON
%catalog
robot.motors=load_motors([5 5 5 5 5 5]);


%Actuator rotor inertia
robot.motors.Inertia=[200e-6 200e-6 200e-6 33e-6 33e-6 33e-6];
%Speed reductor at each joint
%robot.motors.G=[-62.6111 107.815 -53.7063 76.0364 71.923 76.686];
%Please note that, for simplicity in control, we consider that the gear
%ratios are all positive
robot.motors.G=[62.6111 107.815 53.7063 76.0364 71.923 76.686];


