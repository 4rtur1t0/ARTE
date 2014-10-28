%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   SCARA example arm with 4 DOF.
%
%   Author: Arturo Gil. Universidad Miguel Hernández de Elche. 
%   email: arturo.gil@umh.es date:   03/01/2012
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

%Kinematic parameters
robot.DH.theta= '[q(1) q(2) 0 q(4)]';
robot.DH.d='[0.5  0      q(3)  0]';
robot.DH.a='[0.5    0.3  0    0]';
robot.DH.alpha= '[ 0 pi  0 0]';

%Jacobian matrix. Variation of (X, Y, Z) as a function of (w1, w2, w3)
robot.J='[-a(2)*sin(q(1)+q(2))-a(1)*sin(q(1)) -a(2)*sin(q(1)+q(2)) 0; a(2)*cos(q(1)+q(2))+a(1)*cos(q(1)) a(2)*cos(q(1)+q(2)) 0; 0 0 -1];';
robot.name='Scara example 4GDL arm';

robot.inversekinematic_fn = 'inversekinematic_scara(robot, T)';

%number of degrees of freedom
robot.DOF = 4;

%rotational: R, translational: T
robot.kind=['R' 'R' 'T' 'R'];

%minimum and maximum rotation angle in rad
robot.maxangle =[-pi pi; %Axis 1, minimum, maximum
                deg2rad(-150) deg2rad(150); %Axis 2, translational, minimum, maximum
                0 0.5; %Axis 3, translational
                deg2rad(-360) deg2rad(360)]; %Axis 4
             

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [deg2rad(200); %Axis 1, rad/s
                deg2rad(200); %Axis 2, m/s
                2; %Axis 3, m/s
                deg2rad(360)]; %Axis 4, rad/s
             
% end effectors maximum velocity
robot.linear_velmax = 1.0; %m/s

robot.accelmax=robot.velmax/0.1; % 0.1 is here an acceleration time

%base reference system
robot.T0 = eye(4);

%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
robot=init_sim_variables(robot);
robot.path = pwd;

% GRAPHICS
robot.graphical.has_graphics=0;
robot.graphical.color = [25 20 40];
%for transparency
robot.graphical.draw_transparent=0;
%draw DH systems
robot.graphical.draw_axes=1;
%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
robot.graphical.axes_scale=1;
%adjust for a default view of the robot
robot.axis=[-0.75 0.75 -0.75 0.75 0 2];
%read graphics files
robot = read_graphics(robot);

%DYNAMICS
robot.has_dynamics=0;