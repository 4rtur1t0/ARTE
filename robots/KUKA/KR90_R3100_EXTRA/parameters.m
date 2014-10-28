%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   KR 90 R3100 Extra, arc welding robot.
%
%   Author:  
%Antonio Gines Contreras Miñano
%Elena Albors Reig            
%Jesus Burbano Rodriguez
%Carlos Clausi Marin
%Jorge Company Selma
%   email: a.ginescm@gmail.com date:   12/09/2012
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Copyright (C) 2012, by  Arturo Gil Aparicio
%
% This file is part of ARTE (A Robotics Toolbox for Education).
% 
% ARTE is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% ARTE is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with ARTE.  If not, see <http://www.gnu.org/licenses/>.
function robot = parameters()


robot.name= 'KR_90_R3100_EXTRA';

%Introducimos la tabla D-H segun los parametros Denavit-Hartenberg.
robot.path = 'robots/kuka/KR_90_R3100_EXTRA';

robot.DH.theta= '[  q(1)  q(2)-pi/2     q(3)    q(4)    q(5)   q(6)]';
robot.DH.d='[       0.675     0           0      1.400   0   0.215]';
robot.DH.a='[      0.350   1.350         -0.041    0       0      0]';
robot.DH.alpha= '[  -pi/2   0        -pi/2    pi/2    -pi/2   0]';
robot.J=[];


robot.inversekinematic_fn = 'inversekinematic_kuka_KR90_R3100_EXTRA(robot, T)';


%number of degrees of freedom
robot.DOF = 6;

%rotational: 0, translational: 1
robot.kind=['R' 'R' 'R' 'R' 'R' 'R'];
%Restricciones Sobre el robot:
%minimum and maximum rotation angle in rad
robot.maxangle =[deg2rad(-185) deg2rad(185); %Axis 1, minimum, maximum -185 a 185
                deg2rad(-140) deg2rad(-5); %Axis 2, minimum, maximum
                deg2rad(-120) deg2rad(155); %Axis 3
                deg2rad(-350) deg2rad(350); %Axis 4: Unlimited (400º default)
                deg2rad(-125) deg2rad(125); %Axis 5
                deg2rad(-350) deg2rad(350)]; %Axis 6: Unlimited (800º default)

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [deg2rad(156); %Axis 1, rad/s
                deg2rad(156); %Axis 2, rad/s
                deg2rad(156); %Axis 3, rad/s
                deg2rad(343); %Axis 4, rad/s
                deg2rad(362); %Axis 5, rad/s
                deg2rad(659)];%Axis 6, rad/s
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
robot.graphical.color = [255 20 40]./255;
%for transparency
robot.graphical.draw_transparent=1;
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
robot.has_dynamics=0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%DYNAMIC PARAMETERS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robot.has_dynamics=1;
%Masa total del robot repartida segun el tipo de eslabon al que hacemos
%referencia.
%consider friction in the computations
robot.dynamics.friction=0;

%link masses (kg)
robot.dynamics.masses=[0 200 200 180 40 22];
%Distancia al centro de inercia de cada eslabon respecto a su eje anterior.
%COM of each link with respect to own reference system
robot.dynamics.r_com=[0       0          0; %(rx, ry, rz) link 0
    -0.35	0.252	 0; %(rx, ry, rz) link 1
    -0.675	   0	 -0.225;  %(rx, ry, rz) link 2
    0       0       0.150;%(rx, ry, rz) link 4
    0       0           0;%(rx, ry, rz) link 5
    0       0         0.1075];%(rx, ry, rz) link 6
%Matrices de inercia basadas en la  geometria de un cilindro. 
%Inertia matrices of each link with respect to its D-H reference system.
% Ixx	Iyy	Izz	Ixy	Iyz	Ixz, for each row
robot.dynamics.Inertia=[0      0.35	0   	0	0	0;
    9.89     9.89	16	    0	0	0;
    38.78	38.78	2.56	0	0	0;
    34.2	34.2	0.9	    0	0	0;
    0.214	0.214	0.162	0	0	0;
    0.0356	0.0356	0.07	0	0	0];


robot.motors=load_motors([5 5 5 4 4 4]);
%Speed reductor at each joint
robot.motors.G=[300 300 300 300 300 300];
