%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   STAUBLI TP80.
%
%   Author: Pablo Martínez Turiso y Manuel Juan Santonja 
%   email: 
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

robot.DH.theta= '[ q(1) q(2)    0      q(4)]';
robot.DH.d='[     0  0      q(3)-0.1 0]'; 
robot.DH.a='[     0.406  0.394   0     0]'; %DISTANCIA ENTRE EJES
robot.DH.alpha= '[0      pi       pi     0]'; %"PORQUE PONE PI"? QUE ES? ES LA ROTACION DE LOS SISTEMAS DE COORDENADAS?
robot.J=[];
robot.name= 'SATUBLI_TP80';

robot.inversekinematic_fn = 'inversekinematic_KUKA_KR5_scara_R350_Z200(robot, T)'; %"AUN NO LA HEMOS CREADO"

%number of degrees of freedom
robot.DOF = 4;

%rotational: R, translational: T YA ACTUALIZADO
robot.kind=['R' 'R' 'T' 'R'];

%minimum and maximum rotation angle in rad YA ACTUALIZADO
robot.maxangle =[deg2rad(-117) deg2rad(117); %Axis 1, minimum, maximum
                deg2rad(-153) deg2rad(153); %Axis 2, minimum, maximum
                0.000                0.100; %Axis 3, translational, max 200 mm 
                deg2rad(-500) deg2rad(500)]; %Axis 4
                
%maximum absolute speed of each joint rad/s or m/s "NO LO SABEMOS AUN" DE
%AQUI EN ADELANTE NO HEMOS HECHO NADA AUN
robot.velmax = [deg2rad(525); %Axis 1, rad/s
                deg2rad(525); %Axis 2, rad/s
                deg2rad(2); %Axis 3, m/s
                deg2rad(2400)]; %Axis 4, rad/s
               
            robot.accelmax=robot.velmax/0.1; % 0.1 is here an acceleration time
% end effectors maximum velocity
robot.linear_velmax = 1.0; %m/s, not specified

%base reference system
robot.T0 = eye(4);
robot.T0(1,4)=0.330;
robot.T0(3,4)=-0.4238;


%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
robot=init_sim_variables(robot);
robot.path = pwd;

% GRAPHICS
%read graphics files
robot.graphical.has_graphics=1;
robot.graphical.color = [255 20 40]./255;
%for transparency
robot.graphical.draw_transparent=0;
%draw DH systems
robot.graphical.draw_axes=1;
%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
robot.graphical.axes_scale=1;
%adjust for a default view of the robot
robot.axis=[-1.5 1.5 -1.5 1.5 -0.8 0.8];
robot = read_graphics(robot);

%DYNAMICS
robot.has_dynamics=0;