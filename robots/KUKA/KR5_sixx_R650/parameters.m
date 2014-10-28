%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   KUKA KR5 SIXX R850.
%
%   Author: Arturo Gil. Universidad Miguel Hernández de Elche. 
%   email: arturo.gil@umh.es date:   05/01/2012
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

robot.DH.theta= '[q(1) q(2)-pi/2 q(3) q(4) q(5) q(6)]';
robot.DH.d='[0.335 0 0 -0.295 0 -0.080]';
robot.DH.a='[0.075 0.270 0.090 0 0 0]';
robot.DH.alpha= '[-pi/2 0 pi/2 -pi/2 pi/2 pi]';
robot.J=[];
robot.name= 'KUKA_KR5_sixx_R650';

robot.inversekinematic_fn = 'inversekinematic_kuka_kr5_sixx_r650(robot, T)';

%number of degrees of freedom
robot.DOF = 6;

%rotational: 0, translational: 1
robot.kind=['R' 'R' 'R' 'R' 'R' 'R'];

%minimum and maximum rotation angle in rad
robot.maxangle =[deg2rad(-170) deg2rad(170); %Axis 1, minimum, maximum
                deg2rad(-190) deg2rad(45); %Axis 2, minimum, maximum
                deg2rad(-119) deg2rad(165); %Axis 3
                deg2rad(-190) deg2rad(190); %Axis 4: Unlimited (400º default)
                deg2rad(-120) deg2rad(120); %Axis 5
                deg2rad(-358) deg2rad(358)]; %Axis 6: Unlimited (800º default)

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [deg2rad(375); %Axis 1, rad/s
                deg2rad(300); %Axis 2, rad/s
                deg2rad(375); %Axis 3, rad/s
                deg2rad(410); %Axis 4, rad/s
                deg2rad(410); %Axis 5, rad/s
                deg2rad(660)];%Axis 6, rad/s
% end effectors maximum velocity
robot.linear_velmax = 1.0; %m/s, not specified
robot.accelmax=robot.velmax/0.1; % 0.1 is here an acceleration time
%base reference system
robot.T0 = eye(4);

%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
robot=init_sim_variables(robot);
robot.path = pwd;

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
robot.axis=[-0.75 0.75 -0.75 0.75 0 2];
%read graphics files
robot = read_graphics(robot);

%DYNAMICS
robot.has_dynamics=0;