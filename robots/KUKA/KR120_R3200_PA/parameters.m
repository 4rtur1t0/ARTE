%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   KR 120 R3200 PA.
%   Authors: Francisco Nadal Aparicio
%            Pablo Valea Trueba
%            Carlos Tendero Juan 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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


robot.name= 'KR_120_R3200_PA';

%Introducimos la tabla D-H segun los parametros Denavit-Hartenberg.
robot.path = 'robots/KUKA/KR_120_R3200_PA';

robot.DH.theta= '[  q(1)  q(2)-pi/2     q(3)+pi/2    -q(3)-q(2)    q(5)]';
robot.DH.d='[       0.675     0           0      0   0.25]';
robot.DH.a='[      0.350   1.350         1.22    0.28       0]';
robot.DH.alpha= '[  -pi/2   0        0    -pi/2    0]';
robot.J=[];


robot.inversekinematic_fn = 'inversekinematic_kuka_KR120_R3200_PA(robot, T)';


%number of degrees of freedom
robot.DOF = 5;

%rotational: 0, translational: 1
robot.kind=['R' 'R' 'R' 'R' 'R'];
%Restricciones Sobre el robot:
%minimum and maximum rotation angle in rad
robot.maxangle =[deg2rad(-185) deg2rad(185); %Axis 1, minimum, maximum -185 a 185
                deg2rad(-50) deg2rad(85); %Axis 2, minimum, maximum
                deg2rad(-90) deg2rad(65); %Axis 3
                deg2rad(-350) deg2rad(350); %Axis 4: Unlimited (400º default)
                deg2rad(-350) deg2rad(350)]; %Axis 6: Unlimited (800º default)

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [deg2rad(124); %Axis 1, rad/s
                deg2rad(115); %Axis 2, rad/s
                deg2rad(112); %Axis 3, rad/s
                deg2rad(217); %Axis 4, rad/s
                deg2rad(242)];%Axis 6, rad/s
% end effectors maximum velocity
robot.linear_velmax = 5.0; %m/s, not specified

robot.accelmax=robot.velmax/0.1; % 0.1 is here an acceleration time
%base reference system
robot.T0 = eye(4);
%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
robot=init_sim_variables(robot);


% GRAPHICS
robot.graphical.has_graphics=1;
robot.graphical.color = [196, 138, 39]./255;
%for transparency
robot.graphical.draw_transparent=0;
%draw DH systems
robot.graphical.draw_axes=0;
%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
robot.graphical.axes_scale=1;
%adjust for a default view of the robot
robot.axis=[-2 2 -2 2 0 3];
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
robot.dynamics.masses=[100 375 325 150 50];
%Distancia al centro de inercia de cada eslabon respecto a su eje anterior.
%COM of each link with respect to own reference system
robot.dynamics.r_com=[0       0          0;  %(rx, ry, rz) link 1
                      0	      -0.375	 0;  %(rx, ry, rz) link 2
                      0	      0.61	     0;  %(rx, ry, rz) link 3
                      0.28    0.25       0;  %(rx, ry, rz) link 4
                      0       0          0]; %(rx, ry, rz) link 5
%Matrices de inercia basadas en la  geometria de un cilindro. 
%Inertia matrices of each link with respect to its D-H reference system.
% Ixx	Iyy	Izz	Ixy	Iyz	Ixz, for each row
robot.dynamics.Inertia=[15.52  15.52    0.32   	0	0	0;
                        64.36    64.36	1.2	    0	0	0;
                        46.57	46.57	1.05	0	0	0;
                        1.77	1.77	0.3	    0	0	0;
                        0.25	0.25	0.0625	0	0	0];


robot.motors=load_motors([4 5 4 1 1]);
%Speed reductor at each joint
robot.motors.G=[300 300 300 300 300];
