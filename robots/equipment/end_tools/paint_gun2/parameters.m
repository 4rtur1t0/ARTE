function robot = parameters()

robot.name= 'PAINT';

%Path where everything is stored for this robot
%robot.path = 'robots/end_tools/laser_welding';

robot.DH.theta= '[]';
robot.DH.d='[]';
robot.DH.a='[]';
robot.DH.alpha= '[]';
robot.J=[];


robot.inversekinematic_fn = '';

%number of degrees of freedom
robot.DOF = 0;

%rotational: 0, translational: 1
robot.kind=[];

%minimum and maximum rotation angle in rad
robot.maxangle =[]; %Axis 6: Unlimited (800º default)

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [];%Axis 6, rad/s
% end effectors maximum velocity
robot.linear_velmax = 0; %m/s

%base reference system
robot.T0 = eye(4);
% robot.T0 = [-1 0 0 0;
%               0 -1 0 0;
%               0 0 1 0;
%               0 0 0 0.5];

%definition of the tool center point with respect to the last reference
%system.
%for tools, this TCP usually means the transformation from system 
%(X_tool0,Y_tool0,Z_tool0) to (X_tool1,Y_tool1,Z_tool1)
robot.TCP = [1 0 0 0;
             0 1 0 0;
             0 0 1 0.425;
             0 0 0 0.1]; 

%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
robot=init_sim_variables(robot);
robot.path = pwd;
%needed by the simulation. Consider that the piece is not yet gripped (gripped = 0)
robot.piece_gripped=0;
%consider that the tool is closed (open=0)
robot.tool_open=0;


% GRAPHICS
robot.graphical.has_graphics=1;
robot.graphical.color = [100 102 100]./255;
%for transparency
robot.graphical.draw_transparent=0;
%draw DH systems
robot.graphical.draw_axes=0;
%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
robot.graphical.axes_scale=0;
%adjust for a default view of the robot
robot.axis=[-0.75 0.75 -0.75 0.75 0 1.2];
%read graphics files
robot = read_graphics(robot);

%DYNAMICS
robot.has_dynamics=0;