%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   KUKA KR30 L16 2
%
%   Author: Arturo Gil. Universidad Miguel Hern�ndez de Elche. 
%   email: arturo.gil@umh.es date:   08/01/2012
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

robot.DH.theta= '[-q(1) q(2)-pi/2 q(3)   q(4) q(5) q(6)]';
robot.DH.d='[0.815          0     0    1.545 0 0.158]';
robot.DH.a='[0.350         1.2    0.145    0 0 0]';
robot.DH.alpha= '[-pi/2 0 -pi/2 pi/2 -pi/2 0]';
robot.J=[];
robot.name= 'KUKA_KR30_L16_2';

robot.inversekinematic_fn = 'inversekinematic_kuka_kr_30_L16_2(robot, T)';

%number of degrees of freedom
robot.DOF = 6;

%rotational: 0, translational: 1
robot.kind=['R' 'R' 'R' 'R' 'R' 'R'];

%minimum and maximum rotation angle in rad
robot.maxangle =[deg2rad(-185) deg2rad(185); %Axis 1, minimum, maximum
                deg2rad(-135) deg2rad(35); %Axis 2, minimum, maximum
                deg2rad(-120) deg2rad(158); %Axis 3
                deg2rad(-350) deg2rad(350); %Axis 4
                deg2rad(-130) deg2rad(130); %Axis 5
                deg2rad(-350) deg2rad(350)]; %Axis 6

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [deg2rad(100); %Axis 1, rad/s
                deg2rad(80); %Axis 2, rad/s
                deg2rad(80); %Axis 3, rad/s
                deg2rad(230); %Axis 4, rad/s
                deg2rad(165); %Axis 5, rad/s
                deg2rad(249)];%Axis 6, rad/s
% end effectors maximum velocity
robot.linear_velmax = 2.0; %m/s
robot.accelmax=robot.velmax/0.1; % 0.1 is here an acceleration time
%base reference system
robot.T0 = eye(4);

%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
robot=init_sim_variables(robot);

% GRAPHICS
robot.graphical.has_graphics=1;
robot.graphical.color = [200 20 40]./255;
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