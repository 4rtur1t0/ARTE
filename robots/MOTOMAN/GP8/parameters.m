%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the YASKAWA GP8
%
%   Authors: Joaquín Carrasco Palazón and Javier Marín Morcillo
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function robot = parameters()

robot.name= 'GP8';

%Path where everything is stored for this robot
robot.path = 'robots/YASKAWA/GP8';

robot.DH.theta =    '[q(1)      q(2)+pi/2   q(3)    q(4)    q(5)    q(6)]';
robot.DH.d =        '[0.330     0           0       -0.340  0       -0.080]';
robot.DH.a =        '[0.040     0.345       0.040   0       0       0]';
robot.DH.alpha =    '[pi/2      0           -pi/2   pi/2    -pi/2   0]';

robot.J=[];


robot.inversekinematic_fn = 'inversekinematic_GP8(robot, T)';
robot.directkinematic_fn = 'directkinematic(robot, q)';


%number of degrees of freedom
robot.DOF = 6;

%rotational: 0, translational: 1
robot.kind=['R' 'R' 'R' 'R' 'R' 'R'];

%minimum and maximum rotation angle in rad
robot.maxangle =[   deg2rad(-170) deg2rad(170);
                    deg2rad(-65) deg2rad(150);
                    deg2rad(-113) deg2rad(255);
                    deg2rad(-190) deg2rad(190);
                    deg2rad(-135) deg2rad(135);
                    deg2rad(-360) deg2rad(360)];

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [deg2rad(455);  %Axis 1, rad/s  75.83 rpm
                deg2rad(385);  %Axis 2, rad/s  64.17 rpm
                deg2rad(520);  %Axis 3, rad/s  86.67 rpm
                deg2rad(550);  %Axis 4, rad/s  91.67 rpm
                deg2rad(550);  %Axis 5, rad/s  91.67 rpm
                deg2rad(1000)];%Axis 6, rad/s  166.67 rpm
% end effectors maximum velocity
robot.linear_velmax = 3.0; % m/s
robot.accelmax = robot.velmax / 0.1; % 0.1 is here an acceleration time



%base reference system
robot.T0 = eye(4); 



%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
robot=init_sim_variables(robot);
robot.path = pwd;


% GRAPHICS
robot.graphical.has_graphics=1;
robot.graphical.color = [23 30 243]./255;
%for transparency
robot.graphical.draw_transparent=0;
%draw DH systems
robot.graphical.draw_axes=1;
%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
robot.graphical.axes_scale=1;
%adjust for a default view of the robot
robot.axis=[-0.5 1 -0.5 1 0 1];
%read graphics files
robot = read_graphics(robot);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parámetros de la dinámica del robo
% La información dinámica en cuanto a las masas y las inercias se refiere
% las hemos extraido del solid works. Dicha información se adjunta como pdf
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robot.has_dynamics=1;

%No ponemos fricción
robot.dynamics.friction=0;

%link masses (kg)
% Como no sabemos el material del que está hecho el robot y en las especificaciones no pone
% cuánto pesa cada eslabón, hemos puesto la siguiente fórmula para repartir la masa del robot
% la cual es de 32 Kg, en funcion de lo que nos ha dicho el solidworks.
mass_factor = 32 / ((2464.42 + 5353.42 + 2608.24 + 2087.33 + 400.90 + 16.07));
robot.dynamics.masses=[(2464.42 * mass_factor)  (5353.42 * mass_factor) (2608.24 * mass_factor) (2087.33 * mass_factor) (400.90 * mass_factor) (16.07 * mass_factor)];

%COM of each link with respect to own reference system
size_factor = 1 / 1000; % El solid works nos saca milímetros, por lo tanto dividimos por 
                        % 1000 para pasar a metros
robot.dynamics.r_com=[  (-29.98 * size_factor)      (-46.50 * size_factor)      (0.45 * size_factor);   %(rx, ry, rz) link 1
                        (-162.72 * size_factor)     (29.19 * size_factor)       (2.44 * size_factor);   %(rx, ry, rz) link 2
                        (19.87 * size_factor)       (-0.98 * size_factor)       (22.33 * size_factor);  %(rx, ry, rz) link 3
                        (-0.22 * size_factor)       (148.34 * size_factor)      (1.16 * size_factor);   %(rx, ry, rz) link 4
                        (-0.11 * size_factor)       (-0.02 * size_factor)       (-17.22 * size_factor); %(rx, ry, rz) link 5
                        (0.00 * size_factor)        (-0.22 * size_factor)       (4.66 * size_factor)];  %(rx, ry, rz) link 6

%Inertia matrices of each link with respect to its D-H reference system.
% Ixx	Iyy	Izz	Ixy	Iyz	Ixz, for each row
intertia_factor = 1 / (1000  * 1000 * 1000);  % El solid works nos saca mil�metros^2 y gramos
robot.dynamics.Inertia=[(13782869.81 * intertia_factor) (9789239.23 * intertia_factor)      (18374891.67 * intertia_factor)     (4806926.23 * intertia_factor)      (-13736.36 * intertia_factor)	(-7425.15 * intertia_factor);
                        (43933685.46 * intertia_factor) (231969173.67 * intertia_factor)    (208909704.41 * intertia_factor)    (-26305468.52 * intertia_factor)	(26254.69 * intertia_factor)    (-827615.83 * intertia_factor);
                        (9165010.01 * intertia_factor)  (11499392.36 * intertia_factor)     (8521027.15 * intertia_factor)      (-1185.38 * intertia_factor)    	(36312.29 * intertia_factor)	(605655.44 * intertia_factor);
                        (58934726.03 * intertia_factor) (3442622.78 * intertia_factor)	    (58396681.09 * intertia_factor)     (-71578.98 * intertia_factor)	    (420309.01 * intertia_factor)	(-1131.89 * intertia_factor);
                        (582557.40 * intertia_factor)   (598347.64 * intertia_factor)	    (258584.89 * intertia_factor)	    (-284.56 * intertia_factor)	        (50.96 * intertia_factor)	    (1971.39 * intertia_factor);
                        (3896.87 * intertia_factor)     (3828.25 * intertia_factor)	        (6717.18 * intertia_factor)	        (-0.01 * intertia_factor)	        (-8.78 * intertia_factor)	    (0.00 * intertia_factor)];


% Se han seleccionados los siguientes motores para poder cumplir con las especificaciones de las
% velocidades m�ximas que se publicitan en el documento del robot.
% Para la selecci�n de los motores, hemos visto incluso que usando motores
% m�s ligeros, la cosa podr�a funcionar, ya que estamos ante un robot muy
% peque�o. Lo que ocurre es que hay que tener en cuenta que las reductoras
% son bastante caras y pesan tambi�n.
% Sobre todo, lo que hemos tenido presente es que el payload del robot es
% de 8 Kg como m�ximo (robot peque�o), y hemos dado cierto margen los
% motores y las reductoras (la combinaci�n entre ambos).
% Con respecto a las reductoras, podr�amos incluso quitar alguna de ellas,
% pero para que en los puntos muertos puedan las reductoras ayudar un poco
% a los motores, hemos decidido poner una m�nima reducci�n.
robot.motors=load_motors([3 2 2 2 1 1]);
% Reductoras
robot.motors.G=[10 15 5 4 4 4];
%robot.motors.G=[100 160 120 115 220 128];
