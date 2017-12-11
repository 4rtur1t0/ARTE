function robot = parameters()

robot.name= 'ABB_IRB7600_340';

%Path where everything is stored for this robot
robot.path = 'robots/abb/IRB7600_340';

robot.DH.theta= '[q(1) q(2)-pi/2 q(3) q(4) q(5) q(6)+pi]';
robot.DH.d='[0.78 0 0 1.306 0 0.25]';
robot.DH.a='[0.41 1.075 0.165 0 0 0]';
robot.DH.alpha= '[-pi/2 0 -pi/2 pi/2 -pi/2 0]';

robot.J=[];


robot.inversekinematic_fn = 'inversekinematic_IRB7600_340(robot, T)';

%number of degrees of freedom
robot.DOF = 6;

%rotational: 0, translational: 1
robot.kind=['R' 'R' 'R' 'R' 'R' 'R'];

%minimum and maximum rotation angle in rad
robot.maxangle =[deg2rad(-180) deg2rad(180); %Axis 1, minimum, maximum
                deg2rad(-60) deg2rad(85); %Axis 2, minimum, maximum
                deg2rad(-175) deg2rad(60); %Axis 3
                deg2rad(-300) deg2rad(300); %Axis 4: 
                deg2rad(-100) deg2rad(100); %Axis 5
                deg2rad(-360) deg2rad(360)]; %Axis 6: 

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [deg2rad(75); %Axis 1, rad/s
                deg2rad(50); %Axis 2, rad/s
                deg2rad(55); %Axis 3, rad/s
                deg2rad(100); %Axis 4, rad/s
                deg2rad(100); %Axis 5, rad/s
                deg2rad(160)];%Axis 6, rad/s
            
            robot.accelmax=robot.velmax/0.1; % 0.1 is here an acceleration time
% end effectors maximum velocity
robot.linear_velmax = 1.0; %m/s, unavailable from datasheet             %%%BUSCAR O PREGUNTAR

%base reference system 
robot.T0 = eye(4);

%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
robot=init_sim_variables(robot);


% GRAPHICS
robot.graphical.has_graphics=1;
robot.graphical.color = [255 102 51]./255;
%for transparency
robot.graphical.draw_transparent=0;
%draw DH systems
robot.graphical.draw_axes=1;
%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
robot.graphical.axes_scale=1;
%adjust for a default view of the robot
robot.axis=[-3 3 -3 3 0 3];
%read graphics files
robot = read_graphics(robot);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%
%DYNAMIC PARAMETERS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%
robot.has_dynamics=1;

%consider friction in the computations
robot.dynamics.friction=0;

%link masses (kg)
robot.dynamics.masses=[4.462 12.4936 6.2868 3.1234  2.231 0.4462];

%COM of each link with respect to own reference system
robot.dynamics.r_com=[0       0          0; %(rx, ry, rz) link 1
    -0.3638	 0.006	 0.2275; %(rx, ry, rz) link 2
    -0.0203	-0.0141	 0.070;  %(rx, ry, rz) link 3
    0       0.019       0;%(rx, ry, rz) link 4
    0       0           0;%(rx, ry, rz) link 5
    0       0         0.032];%(rx, ry, rz) link 6

%Inertia matrices of each link with respect to its D-H reference system.
% Ixx	Iyy	Izz	Ixy	Iyz	Ixz, for each row
robot.dynamics.Inertia=[0      0.35	0   	0	0	0;
    .13     .524	.539	0	0	0;
    .066	.086	.0125	0	0	0;
    1.8e-3	1.3e-3	1.8e-3	0	0	0;
    .3e-3	.4e-3	.3e-3	0	0	0;
    .15e-3	.15e-3	.04e-3	0	0	0];

robot.motors=load_motors([1 2 3 4 5 5]);
%Speed reductor at each joint
robot.motors.G=[400 375 375 300 300 188];



