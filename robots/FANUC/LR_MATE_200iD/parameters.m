%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   FANUC LR MATE 200iD, FANUC Robotics Europe.
%
%   Author: Pedro Pagán Pallarés, Jose Antonio Bascuñana, David Sansano Pérez,
%   Jose Ramos Ruiz
%   Universidad Miguel Hernandez de Elche. 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function robot = parameters()


robot.name= 'Fanuc_Lr_Mate_200iD';

%Path where everything is stored for this robot
robot.path = 'robots/fanuc/LR_MATE_200iD';
%Tabla de D-H
robot.DH.theta= '[  q(1)  q(2)-pi/2     q(3)    q(4)    q(5)   q(6)]';
robot.DH.d='[       0.33     0           0       0.335   0    0.08]';
robot.DH.a='[       0.05   0.33         0.035    0       0      0]';
robot.DH.alpha= '[-pi/2  0          -pi/2    pi/2    -pi/2   0]';
robot.J=[];


robot.inversekinematic_fn = 'inversekinematic200id(robot, T)';

%number of degrees of freedom
robot.DOF = 6;

%rotational: 0, translational: 1
robot.kind=['R' 'R' 'R' 'R' 'R' 'R'];

%minimum and maximum rotation angle in rad
robot.maxangle =[deg2rad(-180) deg2rad(170); %Axis 1, minimum, maximum
                deg2rad(-122.5) deg2rad(122.5); %Axis 2, minimum, maximum
                deg2rad(-210) deg2rad(210); %Axis 3
                deg2rad(-190) deg2rad(190); %Axis 4: Unlimited (400ï¿½ default)
                deg2rad(-125) deg2rad(125); %Axis 5
                deg2rad(-360) deg2rad(360)]; %Axis 6: Unlimited (800ï¿½ default)

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [deg2rad(450); %Axis 1, rad/s
                deg2rad(380); %Axis 2, rad/s
                deg2rad(520); %Axis 3, rad/s
                deg2rad(550); %Axis 4, rad/s
                deg2rad(545); %Axis 5, rad/s
                deg2rad(720)];%Axis 6, rad/s
% end effectors maximum velocity
robot.linear_velmax = 1.0; %m/s, not specified
robot.accelmax=robot.velmax/0.1; % 0.1 is here an acceleration time

%base reference system
robot.T0 = eye(4);

%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
robot=init_sim_variables(robot);


% GRAPHICS
robot.graphical.has_graphics=1;
robot.graphical.color = [0.7 0.65 0];
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
% DYNAMIC PARAMETERS
%  OJO, no son los datos que representan la dinámica real del robot, ya que
%  no hemos encontrado suficiente información, la hemos tenido que estimar.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robot.has_dynamics=0;

