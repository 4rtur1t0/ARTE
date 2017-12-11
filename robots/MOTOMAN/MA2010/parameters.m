%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   YASKAWA MOTOMAN MA2010 robot.
%
%   Author: Saúl Fernández Candela y David García Egío
%           Universidad Miguel Hernï¿½ndez de Elche. 
%   email:  saul.fernandez01@umh.es 
%           david.garcia32@umh.es
%   date:   27/10/2016
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
function robot = parameters()
%KYNEMATICS
robot.name= 'MOTOMAN_MA2010';

robot.DH.theta = '[q(1) q(2)-pi/2 q(3) q(4) q(5)+pi q(6)]';
robot.DH.d='[0.505  0   0 1.082     0    0.1]';
robot.DH.a='[0.150 0.760  0.200      0       0    0]';
robot.DH.alpha= '[-pi/2  0   -pi/2  pi/2   pi/2 0]';
robot.J=[];

robot.name='Yaskawa MOTOMAN MA2010 welding arc arm';

robot.directkinematic_fn = 'directkinematic(robot, q)';
robot.inversekinematic_fn = 'inversekinematic_motomanMA2010(robot, T)';


%R: rotational, T: translational
robot.kind=['R' 'R' 'R' 'R' 'R' 'R'];

%number of degrees of freedom
robot.DOF = 6;

%minimum and maximum rotation angle in rad
robot.maxangle =[deg2rad(-180) deg2rad(180); %Axis 1, minimum, maximum
                deg2rad(-105) deg2rad(155); %Axis 2, minimum, maximum
                deg2rad(-170) deg2rad(220); %Axis 3
                deg2rad(-150) deg2rad(150); %Axis 4: Unlimited (400ï¿½ default)
                deg2rad(-135) deg2rad(90); %Axis 5
                deg2rad(-210) deg2rad(210)]; %Axis 6: Unlimited (800ï¿½ default)

%maximum absolute speed of each joint rad/s
robot.velmax = [deg2rad(197)
                deg2rad(190)
                deg2rad(210)
                deg2rad(410)
                deg2rad(410)
                deg2rad(610)];
% end effectors maximum velocity
robot.linear_velmax = 1; %not available from datasheet
robot.accelmax=robot.velmax/0.1; % 0.1 is here an acceleration time

%base reference system
robot.T0 = eye(4);

%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
robot=init_sim_variables(robot);
robot.path = pwd;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GRAPHICS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%read graphics files
robot.graphical.has_graphics=1;
robot.graphical.color = [80 120 210]./255;
%for transparency
robot.graphical.draw_transparent=0;
%draw DH systems
robot.graphical.draw_axes=1;
%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
robot.graphical.axes_scale=1;
%adjust for a default view of the robot
robot.axis=[-3 3 -3 3 0 3];
robot = read_graphics(robot);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%DYNAMIC PARAMETERS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robot.has_dynamics=0;

%Estos parámetros dinámicos son los correspondientes a las Práctica3 y han
%sido copiados del Script NewtonEuler6.m

% ------------------------------------------------------------
%   Factores de posicionamiento de los centros de gravedad   
% ------------------------------------------------------------
%factor1 = -0.5; factor2 = -0.5; factor3 = -0.5; 
%factor4 = -0.5; factor5 = -0.5; factor6 = -0.5;

% ------------------------------------------------------------
% 	         Masa de cada elemento (Kg)
% ------------------------------------------------------------

%El peso total del Robot son 240 Kg

%Para elegir el motor de corriente continua hemos dimensionado los
%distintos eslabones dividiendolos entre 100

%m1 = 90/100;  m2 = 80/100;  m3 = 45/100;  
%m4 = 30/100;  m5 = 25/100;  m6 = 10/100;

%La carga exterior del robot es 10Kg

%Siguiendo el mismo procedimiento que con los eslabones, dicha carga
%tambíén ha sido divida entre 100

%masaext=10/100;

% ------------------------------------------------------------
%   Coeficiente de rozamiento viscoso de cada articulacion
% ------------------------------------------------------------
%b1 = 0.05;  b2 = 0.05;  b3 = 0.05;  
%b4 = 0.05;  b5 = 0.05;  b6= 0.05;

% ------------------------------------------------------------
% 	 Matrices de Inercia (Kg-m^2)
% ------------------------------------------------------------
%Las matrices de inercia han sido completadas a partir de los datos
%proporcionados por el Datasheet del robot

%r10I_r01 = zeros(3,3);
%r20I_r02 = zeros(3,3);
%r30I_r03 = zeros(3,3);
%r40I_r04 = 0.65*ones(3,3);
%r50I_r05 = 0.65*ones(3,3);
%r60I_r06 = 0.17*ones(3,3);


