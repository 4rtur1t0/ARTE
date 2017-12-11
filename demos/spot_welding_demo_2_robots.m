% SCRIPT TO TEST THE GRAPHIC CAPABILITIES OF THE TOOLBOX: 
% TWO ROBOTS IN A WELDING CELL

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
close all

fprintf('\nLOAD A ROBOT:\n')

robot1=load_robot('KUKA', 'KR5_2ARC_HW');

q = [0 0 0 0 0 0];

fprintf('\nNOW LOAD AUXILIAR EQUIPMENT')
robot1.equipment{1}=load_robot('equipment', 'bodywork'); 

fprintf('\nNOW LOAD A GRIPPER')
robot1.tool=load_robot('equipment/end_tools', 'spot_welding'); 

%Load a different robot
robot2=load_robot('KUKA', 'KR10_R900');

fprintf('\nNOW LOAD A GRIPPER')
robot2.tool=load_robot('equipment/end_tools', 'spot_welding'); 

%Place the second robot at a different base position
%Move along X and Y directions
A = eye(4);
A(1,4) = 1;
A(2,4) = -2;
robot2.T0 = A;

%draw the first robot
drawrobot3d(robot1, q)

%Draw the 2nd robot with the NOCLEAR OPTION activated
drawrobot3d(robot2, q, 1)



