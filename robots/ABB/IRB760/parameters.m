%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   ABB IRB760.
%
%   The IRB760 is mostly used as a palletizing robot.
%   The robot possesses 4 DOF, though there exist actually 5 axes.
%   A paralellogram (double) mechanism connects the base with the end link,
%   thus restricting the orientation of the Z5 vector to a plane normal to
%   the X0Y0 plane.
%   
%   No graphic files are yet available for this robot.   
%
%   Author: Arturo Gil. Universidad Miguel Hernández de Elche. 
%   email: arturo.gil@umh.es date:   20/11/2013
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

robot.name= 'ABB_IRB760';

robot.DH.theta= '[q(1)  q(2)-pi/2      q(3)+pi/2     -q(2)-q(3)     q(5)]';
robot.DH.d='[0.8145  0      0     0     0.27]';
robot.DH.a='[0.3      1.28   1.35     0.29       0]';
robot.DH.alpha= '[-pi/2  0    0  -pi/2   0]';



robot.J=[];

robot.inversekinematic_fn = 'inversekinematic_irb760(robot, T)';

%number of degrees of freedom
robot.DOF = 5;

%rotational: 0, translational: 1
robot.kind=['R' 'R' 'R' 'R' 'R'];

%minimum and maximum rotation angle in rad
robot.maxangle =[deg2rad(-165) deg2rad(165); %Axis 1, minimum, maximum
                deg2rad(-110) deg2rad(110); %Axis 2, minimum, maximum
                deg2rad(-110) deg2rad(70); %Axis 3
                deg2rad(-160) deg2rad(160); %Axis 4: 
                deg2rad(-120) deg2rad(120)]; %Axis 5: 

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [deg2rad(250); %Axis 1, rad/s
                deg2rad(90); %Axis 2, rad/s
                deg2rad(90); %Axis 3, rad/s
                deg2rad(150); %Axis 4, rad/s
                deg2rad(120)]; %Axis 5, rad/s
               

robot.accelmax=robot.velmax/0.1; % 0.1 is here an acceleration time
            % end effectors maximum velocity
robot.linear_velmax = 1.0; %m/s, unavailable from datasheet

%base reference system 
robot.T0 = eye(4);

%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
robot=init_sim_variables(robot);
robot.path = pwd;

% GRAPHICS
robot.graphical.has_graphics=0;
robot.graphical.color = [255 102 51]./255;
%for transparency
robot.graphical.draw_transparent=0;
%draw DH systems
robot.graphical.draw_axes=1;
%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
robot.graphical.axes_scale=1;
%adjust for a default view of the robot
robot.axis=[-2.5 2.5 -2.5 2.5 0 2.5];

%read graphics files
%robot = read_graphics(robot);

%DYNAMICS
robot.has_dynamics=0;