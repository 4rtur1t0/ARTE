%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   example 5R parallel mechanism
%
%   Author: Arturo Gil. Universidad Miguel Hernández de Elche. 
%   email: arturo.gil@umh.es date:   05/03/2012
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


%For this special case, the mechanism is divided into two different serial
%arms, Namely 2 two dof planar arms.
%Thus, the robot structure contains two different arms named robot1 and
%robot2
robot1=load_robot('example','2dofplanar');
robot2=load_robot('example','2dofplanar');

robot=[];
robot.name='Example 5R parallel';
robot.robot1=robot1;
robot.robot2=robot2;
robot.nserial=2; %number of serial links connecting base and end effector

%displacement along x of the second arm
robot.L=2.5; %m%this value can be edited 

T=eye(4);
T(1,4)=robot.L; 
robot.robot2.T0=T;

%yes, consider it as a special case
%treated separately in the plotting tools
robot.parallel=1;

%this considers the 4 rotational joints and misses the rotational joint
%that connects both 2dof arms
robot.DOF=4; 

robot.axis=[-1.2 4.2 -2.2 2.2 0 2.2]

robot.debug=1;

%Function name to compute inverse kinematic
robot.inversekinematic_fn = 'inversekinematic_5R(robot, T)';
robot.directkinematic_fn = 'directkinematic_5R(robot, q)';

robot.q=zeros(1, robot.DOF);

