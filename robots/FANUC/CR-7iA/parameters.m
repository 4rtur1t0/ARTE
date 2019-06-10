%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   FANUC CR-7iA, FANUC Robotics Europe.
%
%   Author: Rubén Alcaraz Poblet
%   Universidad Miguel Hernandez de Elche.
%   date:   01/01/2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Copyright (C) 2019, by Rubén Alcaraz Poblet.
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
robot.name= 'Fanuc_CR_7iA';

%Path where everything is stored for this robot
robot.path = 'robots\FANUC\CR-7iA';
%Tabla de D-H
robot.DH.theta= '[q(1)  q(2)+pi/2   q(3)    q(4)    q(5)    q(6)]';
robot.DH.d='[0.457  0   0   0.335   0   0.08]';
robot.DH.a='[0.05   0.33    0.035   0   0   0]';
robot.DH.alpha= '[pi/2  0   pi/2    -pi/2   pi/2    0]';
robot.J=[];


robot.inversekinematic_fn = 'inversekinematic_fanuc_cr_7ia(robot, T)';
robot.directkinematic_fn = 'directkinematic(robot, q)';
%number of degrees of freedom
robot.DOF = 6;

%rotational: 0, translational: 1
robot.kind=['R' 'R' 'R' 'R' 'R' 'R'];

%minimum and maximum rotation angle in rad
robot.maxangle =[deg2rad(-170) deg2rad(170); %Axis 1, minimum, maximum
                deg2rad(-83) deg2rad(83); %Axis 2, minimum, maximum
                deg2rad(-186.5) deg2rad(186.5); %Axis 3
                deg2rad(-190) deg2rad(190); %Axis 4
                deg2rad(-120) deg2rad(120); %Axis 5
                -2*pi 2*pi]; %Axis 6

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [deg2rad(350); %Axis 1, rad/s
                deg2rad(350); %Axis 2, rad/s
                deg2rad(400); %Axis 3, rad/s
                deg2rad(450); %Axis 4, rad/s
                deg2rad(450); %Axis 5, rad/s
                deg2rad(720)];%Axis 6, rad/s
% end effectors maximum velocity
robot.linear_velmax = 1.0; % m/s
robot.accelmax=robot.velmax/0.1; % 0.1 is here an acceleration time

%base reference system
robot.T0 = eye(4);

%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
robot=init_sim_variables(robot);
robot.path = pwd;

% GRAPHICS
robot.graphical.has_graphics=0;
robot.graphical.color = [0 1 0];
%for transparency
robot.graphical.draw_transparent=0;
%draw DH systems
robot.graphical.draw_axes=1;
%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
robot.graphical.axes_scale=1;
%adjust for a default view of the robot
robot.axis=[-0.3 0.6 -0.3 0.3 0 1];
%read graphics files
robot = read_graphics(robot);

%DYNAMICS
robot.has_dynamics=1;

%No ponemos fricciÃ³n
robot.dynamics.friction=0;

%link masses (kg)
% Como no sabemos el material del que estÃ¡ hecho el robot y en las especificaciones no pone
% cuÃ¡nto pesa cada eslabÃ³n, se reparte la masa total del robot en proporción a la distancia
% de cada eslabón respecto a la distancia total (todos juntos en serie).
longitud = [330 330 150 335 80 20]./1000; % Longitudes de cada eslabón en metros
longitud_total = 0.127 + sum(longitud); % Se suma la longitud de la base o eslabón 0.
mass_factor = 53 / longitud_total;
robot.dynamics.masses = longitud*mass_factor;

%COM of each link with respect to own reference system
% Se considera que la unión entre eslabones se localiza siempre en la base
% de los cilindros
robot.dynamics.r_com = zeros(6,3); %(rx, ry, rz) link i

%Inertia matrices of each link with respect to its D-H reference system.
% Ixx	Iyy	Izz	Ixy	Iyz	Ixz, for each row
% A falta de archivos CAD, se simplifica cada eslabón a la geometría de un
% cilindro de radio 0,05 m.
radio = 0.05;
robot.dynamics.Inertia=[inercia_extremo(radio,robot.dynamics.masses(1),longitud(1))     inercia_eje_central(radio,robot.dynamics.masses(1))             inercia_extremo(radio,robot.dynamics.masses(1),longitud(1))     0       0       0;
                        inercia_eje_central(radio,robot.dynamics.masses(2))             inercia_extremo(radio,robot.dynamics.masses(2),longitud(2))     inercia_extremo(radio,robot.dynamics.masses(2),longitud(2))     0       0       0;
                        inercia_extremo(radio,robot.dynamics.masses(3),longitud(3))     inercia_extremo(radio,robot.dynamics.masses(3),longitud(3))     inercia_eje_central(radio,robot.dynamics.masses(3))             0    	0       0;
                        inercia_extremo(radio,robot.dynamics.masses(4),longitud(4))     inercia_eje_central(radio,robot.dynamics.masses(4))             inercia_extremo(radio,robot.dynamics.masses(4),longitud(4))     0	    0       0;
                        inercia_extremo(radio,robot.dynamics.masses(5),longitud(5))     inercia_extremo(radio,robot.dynamics.masses(5),longitud(5))	    inercia_eje_central(radio,robot.dynamics.masses(5))             0	    0	    0;
                        inercia_extremo(radio,robot.dynamics.masses(6),longitud(6))     inercia_extremo(radio,robot.dynamics.masses(6),longitud(6))	    inercia_eje_central(radio,robot.dynamics.masses(6))             0	    0	    0];
                    
% Se han seleccionados los siguientes motores para poder cumplir con las
% especificaciones de las velocidades máximas del robot.
% Se han evitado las reductoras al ser caras y pesadas, por lo que se ha
% obtado por motores más potentes.
robot.motors = motores([1 2 1 1 1 1]);
% Reductoras
robot.motors.G = [1 1 1 1 1 1];