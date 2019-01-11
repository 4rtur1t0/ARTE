%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   FANUC M-20iA, FANUC Robotics Europe.
%
%   Author: María Rosario Abellán Baeza, Cristina Sánchez Noguera y Eugenio
%   Canals Orts. Universidad Miguel Hernandez de Elche. 
%   email: maria.abellan03@goumh.umh.es , cristina.sanchez11@goumh.umh.es  , eugenio.canals@goumh.umh.es
%   date: 07/01/2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Copyright (C) 2012, by Arturo Gil Aparicio
% 
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

%Nombre del robot
robot.name= 'Fanuc_M_20iA';

%Path where everything is stored for this robot
%Ruta donde se guarda toda la informacion de este robot
robot.path = 'robots/fanuc/M_20iA';

%Table of D-H
%Tabla de D-H
robot.DH.theta= '[  q(1)+pi/2  q(2)-pi/2     q(3)    q(4)    q(5)   q(6)]';
robot.DH.d='[       0.525           0         0      0.835     0    0.100]';
robot.DH.a='[       0.150       0.790      0.250       0       0      0]';
robot.DH.alpha= '  [-pi/2          0       -pi/2      pi/2    -pi/2   0]';

%Matriz Jacobiana
robot.J=[];

%Llamamos a la funcion inversa
robot.inversekinematic_fn = 'inversekinematic_fanuc_m_20ia(robot, T)';

%Number of degrees of freedom
%Numero de grados de libertad
robot.DOF = 6;

%Rotational: R, translational: T
%Rotación: R, Traslación: T 
robot.kind=['R' 'R' 'R' 'R' 'R' 'R'];

%Minimum and maximum rotation angle in rad COMPROBAR 
%Mínimos y máximos ángulos de rotacíon en radianes
robot.maxangle =[deg2rad(-185) deg2rad(185); %Axis 1, MINIMO, MÁXIMO
                deg2rad(-130) deg2rad(130); %Axis 2,  
                deg2rad(-229) deg2rad(229); %Axis 3, 
                deg2rad(-200) deg2rad(200); %Axis 4: 
                deg2rad(-180) deg2rad(180); %Axis 5, 
                deg2rad(-450) deg2rad(450)]; %Axis 6: 

%Maximum absolute speed of each joint rad/s or m/s
%Máxima velocidad absoluta en cada articulación en rad/s o en m/s
robot.velmax = [deg2rad(195); %Axis 1, rad/s
                deg2rad(175); %Axis 2, rad/s
                deg2rad(180); %Axis 3, rad/s
                deg2rad(360); %Axis 4, rad/s
                deg2rad(360); %Axis 5, rad/s
                deg2rad(550)];%Axis 6, rad/s
            
%End effectors maximum velocity
%Velocidad máxima en los efectores finales
robot.linear_velmax = 1.0; %m/s, No especificada

%End effectors maximum acceleration
%Aceleración máxima en los efectores finales
robot.accelmax=robot.velmax/0.1; % 0.1 es el tiempo de acelaración

%Base reference system
%Sistema de referencia de la base
robot.T0 = eye(4);

%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%INICIALIZACIÓN DE VARIABLES REQUERIDAS PARA LA SIMULACIÓN
%Position, velocity and acceleration
%Posición, velocidad y aceleración
robot=init_sim_variables(robot);
%robot.path = pwd;

%GRAPHICS
%GRAFICOS
robot.graphical.has_graphics=1;
robot.graphical.color = [255 255 0]./255;

%For transparency
%Para hacer transparente
robot.graphical.draw_transparent=0;

%Draw DH systems
%Dibujar sistema de D-H
robot.graphical.draw_axes=1;

%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
%La escala del sistema D-H y el tamaño de fuente es 1710. Para robots más
%grandes selecciona 2/20, 3/30
robot.graphical.axes_scale=1;

%Adjust for a default view of the robot
%Vista por defecto del robots
robot.axis=[-1.5 1.5 -1.5 1.5 0 2];

%Read graphics files
%Leer archivos gráficos
robot = read_graphics(robot);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%DYNAMIC PARAMETERS
%PARÁMETROS DINÁMICOS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Activamos los parámetros dinámicos
robot.has_dynamics=1;

%Consider friction in the computations
%Considerar la fricción en los cálculos
robot.dynamics.friction=0;

%link masses (kg)
%Masas (kg)
robot.dynamics.masses=[84 38 44 28 1.8 0.2];

%COM of each link with respect to own reference system
%Centros de masas de cada link con respecto a su propio sistema de
%referencia
robot.dynamics.r_com=[-0.125     0.05    0.05; %(rx, ry, rz) link 1
                      -0.425	  0	    -0.15; %(rx, ry, rz) link 2
                      -0.125	-0.025  0.075;  %(rx, ry, rz) link 3
                       0        -0.26   -0.03;%(rx, ry, rz) link 4
                       0         0      0.04;%(rx, ry, rz) link 5
                       0         0      -0.02];%(rx, ry, rz) link 6

%Inertia matrices of each link with respect to its D-H reference system.
% Ixx	Iyy	Izz	Ixy	Iyz	Ixz, for each row
%Matrices de inercia de cada link con respecto a su sist. de referencia D-H
% Ixx Iyy Izz Ixy Iyz Ixz, para cada fila
robot.dynamics.Inertia=[2.1700      3.5000	    3.0100      0.5250  	-0.2100	 0.5250;
                        1.0133      10.6083 	9.8483	    0	         0  	-2.4225;
                        0.7333  	1.6133	    1.3933	   -0.1375	     0.0825	 0.4125;                   
                        3.0417	    0.1148	    3.0165	    0	        -0.2184	 0;
                        0.0055	    0.0055	    0.0032	    0          	0	     0;
                        1.5170e-04	1.5170e-04	9.0000e-05	0	        0	     0];

%Speed reductor at each joint
%Reductor de velocidad en cada articulación
robot.motors.G = [10 6 42 11 2 2];
%Please note that, for simplicity in control, we consider that the gear
%ratios are all positive
%Tenga en cuenta que, para simplificar el control, consideramos que las 
%relaciones de transmisión son todas positivas

%LOS MOTORES QUE HEMOS USADO SON:
%M1 -> PowerTec -> E-254
%M2 -> PowerTec -> E-254
%M3 -> ABB -> 9C4.4.40-M
%M4 -> Maxon -> 136210
%M5 -> Maxon -> 136210
%M6 -> Maxon -> 136210


