%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   Antimisiles
%
%   Author: Tamer Kayal Kharrat/Jose Enrique Meseguer Plaza. Universidad Miguel Hernández de Elche. 
%   email: date:   22/12/2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Copyright (C) 2019, by Tamer Kayal Kharrat and Jose Enrique Meseguer
% Plaza
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

robot.name= 'anti_misiles';

robot.DH.theta='[q(1) q(2)]';
robot.DH.d='[2.100017631 0]';
robot.DH.a='[0 7.5]';
robot.DH.alpha='[pi/2 0]';

robot.J=[];


robot.inversekinematic_fn = 'inversekinematic_anti_misiles(robot, T)';

%number of degrees of freedom
robot.DOF = 2;

%rotational: 2, translational: 0
robot.kind=['R' 'R'];

%minimum and maximum rotation angle in rad
robot.maxangle =[deg2rad(-180) deg2rad(180); %Axis 1, minimum, maximum
                deg2rad(0) deg2rad(90)]; %Axis 2: 

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [deg2rad(pi/20); %Axis 1, rad/s
                deg2rad(pi/20)];%Axis 2, rad/s
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
robot.graphical.color = [207 218 222]./255;
%for transparency
robot.graphical.draw_transparent=0;
%draw DH systems
robot.graphical.draw_axes=1;
%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
robot.graphical.axes_scale=1;
%adjust for a default view of the robot
robot.axis=[-5 10 -2 2 0 5];
%read graphics files
robot = read_graphics(robot);

%DYNAMICS
robot.has_dynamics=0;