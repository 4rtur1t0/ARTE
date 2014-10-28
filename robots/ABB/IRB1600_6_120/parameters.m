%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   ABB IRB1600_6_120.
%
%   Author: Encarnacion Sanchez Luis
%           David Barrero Nunez
%           Raquel Blazquez Corral
%           Universidad Miguel Hernandez de Elche. 
%  date:   20/12/2012
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% En este fichero hemos introducido parametros del robot
% ABB IRB 1600 6/120 tales como la matriz D-H, los rangos de trabajo y 
% las velocidades maximas de las articulaciones.
% Si introducimos los siguiente comando calcularemos la matriz T.

% q=[q1 q2 q3 q4 q5 q6]
% T=directkinematic(robot,q)


% Copyright (C) 2012, by Arturo Gil Aparicio
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

robot.name= 'IRB1600_6_120';

%Path where everything is stored for this robot
robot.path = 'robots/abb/IRB1600_6_120';

robot.DH.theta= '[q(1)   q(2)-pi/2 q(3)   q(4)   q(5)   q(6)+pi]';
robot.DH.d='[    0.4865     0        0    0.600    0    0.065]';
robot.DH.a='[    0.150    0.475      0     0       0     0]';
robot.DH.alpha= '[-pi/2      0     -pi/2  pi/2   -pi/2   0]';

robot.J=[];


robot.inversekinematic_fn = 'inversekinematic_irb1600_6_120(robot, T)';

%number of degrees of freedom
robot.DOF = 6;

%rotational: 0, translational: 1
robot.kind=['R' 'R' 'R' 'R' 'R' 'R'];

%minimum and maximum rotation angle in rad
robot.maxangle =[deg2rad(-200) deg2rad(200); %Axis 1, minimum, maximum
                deg2rad(-63) deg2rad(136); %Axis 2, minimum, maximum
                deg2rad(-235) deg2rad(55); %Axis 3
                deg2rad(-200) deg2rad(200); %Axis 4: 
                deg2rad(-115) deg2rad(115); %Axis 5
                deg2rad(-400) deg2rad(400)]; %Axis 6: 

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [deg2rad(150); %Axis 1, rad/s
                deg2rad(160); %Axis 2, rad/s
                deg2rad(170); %Axis 3, rad/s
                deg2rad(320); %Axis 4, rad/s
                deg2rad(400); %Axis 5, rad/s
                deg2rad(460)];%Axis 6, rad/s
            
robot.accelmax=robot.velmax/0.1; % 0.1 is here an acceleration time
            
            % end effectors maximum velocity
robot.linear_velmax = 1.0; %m/s, unavailable from datasheet

%base reference system 
robot.T0 = eye(4);

%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
robot=init_sim_variables(robot);


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
robot.axis=[-2 2 -2 2 0 2.5];
%read graphics files
robot = read_graphics(robot);

%DYNAMICS
robot.has_dynamics=0;