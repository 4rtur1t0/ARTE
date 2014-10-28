%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   example 3RRR parallel mechanism
%
%   Author: Arturo Gil. Universidad Miguel Hernández de Elche. 
%   email: arturo.gil@umh.es date:   28/09/2013
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
%global robot


%For this special case, the mechanism is divided into three different serial
%arms, that is three two dof planar arms.
%Thus, the robot structure contains three different arms named robot1,
%robot2 and robot3
robot1=load_robot('example','2dofplanar');
robot2=load_robot('example','2dofplanar');
robot3=load_robot('example','2dofplanar');


robot=[];
robot.name='Example 3RRR parallel';
robot.robot1=robot1;
robot.robot2=robot2;
robot.robot3=robot3;

robot.nserial=3; %number of serial links connecting base and end


robot.T0=eye(4);

robot.h=0.5; %triangle side length

%displacement along x in the second arm
L=2.5; %m%this value can be edited 

T=eye(4);
T(1,4)=L; 
robot.robot2.T0=T;

T=eye(4);
T(1,4)=L/2;
T(2,4)=L;
robot.robot3.T0=T;

%yes, consider it as a special case
%treated separately in the plotting tools
robot.parallel=1;

robot.graphical=1;

%this considers the 4 rotational joints and misses the rotational joint
%that connects both 2dof arms
robot.DOF=6; 

robot.axis=[-1.2 4.2 -1.2 3.2 0 2.2]

robot.debug=1;

%Function name to compute inverse kinematic
robot.inversekinematic_fn = 'inversekinematic_3RRR(robot, T)';
robot.directkinematic_fn = 'directkinematic_3RRR(robot, q)';

robot.q=zeros(1, robot.DOF);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%GRAPHICS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%read graphics files
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
robot.axis=[-2.2 2.2 -2.2 2.2 0 2.2]

